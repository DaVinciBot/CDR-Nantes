/*
 * 6-state EKF [x, y, theta, vx, vy, omega]. See include/ekf6.h for the model,
 * the measurements and the diagnostics.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "ekf6.h"

#include <math.h>

namespace kalman {

namespace {

/** @brief One non-identity term of F: F(row, col) = value, on top of I. */
struct SparseTerm {
    size_t row;
    size_t col;
    float value;
};

/** Number of non-identity terms in F. */
constexpr size_t F_TERMS = 9;

/** @brief The mid-tick linearisation shared by the mean, F and Q. */
struct Linearization {
    float c;   // cos(m), m = theta + omega dt / 2
    float s;   // sin(m)
    float a;   // d x' / d theta = (-sin(m) vx - cos(m) vy) dt
    float b;   // d y' / d theta = ( cos(m) vx - sin(m) vy) dt
};

Linearization linearize(const Vec<Ekf6::N> &state, float dt) {
    const float mid = state[Ekf6::THETA] + 0.5f * state[Ekf6::OMEGA] * dt;
    const float vx = state[Ekf6::VX];
    const float vy = state[Ekf6::VY];
    Linearization lin;
    lin.c = cosf(mid);
    lin.s = sinf(mid);
    lin.a = (-lin.s * vx - lin.c * vy) * dt;
    lin.b = (lin.c * vx - lin.s * vy) * dt;
    return lin;
}

/**
 * @brief The terms of F - I.
 *
 * Rows x and y depend on theta, on both velocities, and on omega through the
 * mid-tick heading (hence the dt / 2); theta depends on omega. The velocity
 * rows are the identity.
 */
void f_terms(const Linearization &lin, float dt, SparseTerm terms[F_TERMS]) {
    const SparseTerm list[F_TERMS] = {
        {Ekf6::X, Ekf6::THETA, lin.a},
        {Ekf6::X, Ekf6::VX, lin.c * dt},
        {Ekf6::X, Ekf6::VY, -lin.s * dt},
        {Ekf6::X, Ekf6::OMEGA, 0.5f * lin.a * dt},
        {Ekf6::Y, Ekf6::THETA, lin.b},
        {Ekf6::Y, Ekf6::VX, lin.s * dt},
        {Ekf6::Y, Ekf6::VY, lin.c * dt},
        {Ekf6::Y, Ekf6::OMEGA, 0.5f * lin.b * dt},
        {Ekf6::THETA, Ekf6::OMEGA, dt},
    };
    for (size_t i = 0; i < F_TERMS; ++i) {
        terms[i] = list[i];
    }
}

/** True if a is strictly later than b on a wrapping microsecond clock. */
bool is_later(uint32_t a, uint32_t b) {
    return static_cast<int32_t>(a - b) > 0;
}

}  // namespace

Ekf6Params provisional_ekf6_params() {
    Ekf6Params p;
    p.accel_noise = 200.0f;
    p.alpha_noise = 2.0f;
    p.wheel_sigma_counts = 10.0f;
    p.wheel_slip_ratio = 0.02f;
    p.optical_sigma_mm_s = 10.0f;
    p.heading_sigma_rad = 0.01f;
    p.optical_lever_x_mm = 0.0f;
    p.optical_lever_y_mm = 0.0f;
    // The link is declared degraded after 50 ms without a tick; a longer tick is
    // not something to predict across.
    p.max_dt_s = 0.05f;
    p.max_sigma_theta_rad = 0.2f;
    p.max_sigma_v_mm_s = 200.0f;
    p.max_sigma_omega_rad_s = 1.0f;
    // 10 samples = 100 ms of a sensor that disagrees with everything else.
    p.optical_max_consecutive_rejections = 10;
    // The SparkFun OTOS linear scalar normally sits within 0.95-1.05; a ratio
    // outside +-7 % over a second is a physical problem, not noise.
    p.optical_ratio_min = 0.93f;
    p.optical_ratio_max = 1.07f;
    p.optical_ratio_min_speed_mm_s = 50.0f;
    p.optical_ratio_window = 100;
    return p;
}

Ekf6::Ekf6(const Geometry &geometry, const Ekf6Params &params)
    : geometry_(geometry), params_(params) {
    reset(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void Ekf6::reset(float x_mm, float y_mm, float theta_rad, float sigma_xy_mm,
                 float sigma_theta_rad, float sigma_v_mm_s, float sigma_omega_rad_s) {
    x_ = Vec<N>::zero();
    x_[X] = x_mm;
    x_[Y] = y_mm;
    x_[THETA] = wrap_pi(theta_rad);

    Vec<N> variance;
    variance[X] = sigma_xy_mm * sigma_xy_mm;
    variance[Y] = sigma_xy_mm * sigma_xy_mm;
    variance[THETA] = sigma_theta_rad * sigma_theta_rad;
    variance[VX] = sigma_v_mm_s * sigma_v_mm_s;
    variance[VY] = sigma_v_mm_s * sigma_v_mm_s;
    variance[OMEGA] = sigma_omega_rad_s * sigma_omega_rad_s;
    P_ = Mat<N>::diagonal(variance);
}

void Ekf6::reset(const Vec<N> &state, const Mat<N> &covariance) {
    x_ = state;
    x_[THETA] = wrap_pi(x_[THETA]);
    P_ = covariance;
    symmetrize(P_);
}

bool Ekf6::process_tick(const OdometryTick &tick) {
    // A first frame's deltas are measured against nothing, and its dt against
    // no previous tick either.
    if (tick.first_frame) {
        ++ticks_skipped_;
        return false;
    }
    if (!predict(static_cast<float>(tick.dt_us) * 1e-6f)) {
        ++ticks_skipped_;
        return false;
    }
    update_wheels(tick);
    ++ticks_used_;
    return true;
}

bool Ekf6::predict(float dt) {
    // Written so that a NaN dt falls on the refusing side.
    if (!(dt > 0.0f && dt <= params_.max_dt_s)) {
        ++predictions_refused_;
        return false;
    }
    wheel_speed_fresh_ = false;

    const Linearization lin = linearize(x_, dt);
    SparseTerm terms[F_TERMS];
    f_terms(lin, dt, terms);

    // The noise is linearised at the state BEFORE it moves, like F.
    const Mat<N> Q = process_noise(x_, dt);

    // Mean.
    const float vx = x_[VX];
    const float vy = x_[VY];
    x_[X] += (lin.c * vx - lin.s * vy) * dt;
    x_[Y] += (lin.s * vx + lin.c * vy) * dt;
    x_[THETA] = wrap_pi(x_[THETA] + x_[OMEGA] * dt);

    // F P F^T with F = I + E, E sparse:
    //   M = F P      -> row r of M gets value * row col of P, for each term;
    //   P' = M F^T   -> column r of P' gets value * column col of M.
    // About a hundred multiply-adds instead of two dense 6x6 products.
    Mat<N> M = P_;
    for (size_t t = 0; t < F_TERMS; ++t) {
        for (size_t j = 0; j < N; ++j) {
            M(terms[t].row, j) += terms[t].value * P_(terms[t].col, j);
        }
    }
    Mat<N> next = M;
    for (size_t t = 0; t < F_TERMS; ++t) {
        for (size_t i = 0; i < N; ++i) {
            next(i, terms[t].row) += terms[t].value * M(i, terms[t].col);
        }
    }

    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            next(i, j) += Q(i, j);
        }
    }
    symmetrize(next);
    P_ = next;
    return true;
}

void Ekf6::update_wheels(const OdometryTick &tick) {
    if (tick.first_frame || !tick.encoders_valid || tick.dt_us == 0) {
        return;
    }
    const float dt = static_cast<float>(tick.dt_us) * 1e-6f;
    const int32_t counts[3] = {tick.d1, tick.d2, tick.d3};
    const float floor_mm_s = params_.wheel_sigma_counts * geometry_.counts_to_mm / dt;

    for (size_t wheel = 0; wheel < 3; ++wheel) {
        const float z = static_cast<float>(counts[wheel]) * geometry_.counts_to_mm / dt;
        const float slip = params_.wheel_slip_ratio * z;
        const Vec<N> h = wheel_row(wheel);
        float predicted = 0.0f;
        for (size_t i = 0; i < N; ++i) {
            predicted += h[i] * x_[i];
        }
        apply(h, z - predicted, floor_mm_s * floor_mm_s + slip * slip, wheel_stats_);
    }

    // Reference speed for the optical ratio: the robot centre, from the wheels
    // alone, before any fusion.
    const BodyDelta delta = forward_kinematics(geometry_, tick.d1, tick.d2, tick.d3);
    wheel_speed_mm_s_ = sqrtf(delta.dx * delta.dx + delta.dy * delta.dy) / dt;
    wheel_speed_fresh_ = true;
}

OpticalUpdate Ekf6::update_optical(float vx_mm_s, float vy_mm_s, uint32_t timestamp_us) {
    OpticalUpdate out;
    out.x = UpdateResult{0.0f, 0.0f, 0.0f, false};
    out.y = out.x;
    out.stale = false;

    if (optical_seen_ && !is_later(timestamp_us, optical_last_us_)) {
        ++stale_samples_;
        out.stale = true;
        return out;
    }
    optical_seen_ = true;
    optical_last_us_ = timestamp_us;

    // The ratio sees every sample, gated or not: an under-counting sensor is
    // precisely the one whose samples might pass the gate.
    feed_ratio(vx_mm_s, vy_mm_s);

    const float r = params_.optical_sigma_mm_s * params_.optical_sigma_mm_s;
    const float rx = params_.optical_lever_x_mm;
    const float ry = params_.optical_lever_y_mm;

    // Velocity at the sensor = centre velocity + omega x r.
    Vec<N> hx = Vec<N>::zero();
    hx[VX] = 1.0f;
    hx[OMEGA] = -ry;
    out.x = apply(hx, vx_mm_s - (x_[VX] - x_[OMEGA] * ry), r, optical_stats_);

    Vec<N> hy = Vec<N>::zero();
    hy[VY] = 1.0f;
    hy[OMEGA] = rx;
    out.y = apply(hy, vy_mm_s - (x_[VY] + x_[OMEGA] * rx), r, optical_stats_);

    if (out.x.accepted && out.y.accepted) {
        optical_consecutive_rejections_ = 0;
    } else {
        ++optical_consecutive_rejections_;
    }
    return out;
}

UpdateResult Ekf6::update_heading(float heading_rad, uint32_t timestamp_us) {
    if (heading_seen_ && !is_later(timestamp_us, heading_last_us_)) {
        ++stale_samples_;
        return UpdateResult{0.0f, 0.0f, 0.0f, false};
    }
    heading_seen_ = true;
    heading_last_us_ = timestamp_us;

    Vec<N> h = Vec<N>::zero();
    h[THETA] = 1.0f;
    return apply(h, angle_innovation(heading_rad, x_[THETA]),
                 params_.heading_sigma_rad * params_.heading_sigma_rad, heading_stats_);
}

bool Ekf6::covariance_healthy() const {
    for (size_t i = 0; i < N; ++i) {
        // Negative or NaN: the covariance is broken, whatever the state.
        if (!(P_(i, i) >= 0.0f)) {
            return false;
        }
    }
    // x and y: finite, and nothing more. Their growth is expected.
    if (!(P_(X, X) < INFINITY) || !(P_(Y, Y) < INFINITY)) {
        return false;
    }
    const float max_v = params_.max_sigma_v_mm_s * params_.max_sigma_v_mm_s;
    return P_(THETA, THETA) <= params_.max_sigma_theta_rad * params_.max_sigma_theta_rad &&
           P_(VX, VX) <= max_v && P_(VY, VY) <= max_v &&
           P_(OMEGA, OMEGA) <= params_.max_sigma_omega_rad_s * params_.max_sigma_omega_rad_s;
}

bool Ekf6::optical_ok() const {
    return optical_consecutive_rejections_ < params_.optical_max_consecutive_rejections &&
           !ratio_out_of_band_;
}

Vec<Ekf6::N> Ekf6::transition(const Vec<N> &state, float dt) {
    const Linearization lin = linearize(state, dt);
    Vec<N> next = state;
    next[X] += (lin.c * state[VX] - lin.s * state[VY]) * dt;
    next[Y] += (lin.s * state[VX] + lin.c * state[VY]) * dt;
    next[THETA] = wrap_pi(state[THETA] + state[OMEGA] * dt);
    return next;
}

void Ekf6::jacobian(const Vec<N> &state, float dt, Mat<N> &F) {
    const Linearization lin = linearize(state, dt);
    SparseTerm terms[F_TERMS];
    f_terms(lin, dt, terms);
    F = Mat<N>::identity();
    for (size_t t = 0; t < F_TERMS; ++t) {
        F(terms[t].row, terms[t].col) += terms[t].value;
    }
}

Mat<Ekf6::N> Ekf6::process_noise(const Vec<N> &state, float dt) const {
    // White acceleration in the robot frame. Per axis, the classic
    // [dt^3/3, dt^2/2; dt^2/2, dt] block. Translation is isotropic, so its
    // position block survives the rotation to the world frame unchanged, and
    // only the position / velocity cross terms carry the rotation R(m).
    const Linearization lin = linearize(state, dt);
    const float qa = params_.accel_noise * params_.accel_noise;
    const float qw = params_.alpha_noise * params_.alpha_noise;
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;

    Mat<N> Q = Mat<N>::zero();
    Q(X, X) = qa * dt3 / 3.0f;
    Q(Y, Y) = qa * dt3 / 3.0f;
    Q(VX, VX) = qa * dt;
    Q(VY, VY) = qa * dt;

    const float cross = qa * dt2 / 2.0f;
    Q(X, VX) = cross * lin.c;
    Q(X, VY) = -cross * lin.s;
    Q(Y, VX) = cross * lin.s;
    Q(Y, VY) = cross * lin.c;
    Q(VX, X) = Q(X, VX);
    Q(VY, X) = Q(X, VY);
    Q(VX, Y) = Q(Y, VX);
    Q(VY, Y) = Q(Y, VY);

    Q(THETA, THETA) = qw * dt3 / 3.0f;
    Q(THETA, OMEGA) = qw * dt2 / 2.0f;
    Q(OMEGA, THETA) = Q(THETA, OMEGA);
    Q(OMEGA, OMEGA) = qw * dt;
    return Q;
}

Vec<Ekf6::N> Ekf6::wheel_row(size_t wheel) const {
    // Inverse kinematics, as speeds: w = a vx + b vy - R omega.
    static const float A[3] = {-0.5f, -0.5f, 1.0f};
    static const float B[3] = {0.5f * SQRT3_F, -0.5f * SQRT3_F, 0.0f};
    Vec<N> h = Vec<N>::zero();
    h[VX] = A[wheel];
    h[VY] = B[wheel];
    h[OMEGA] = -geometry_.robot_radius_mm;
    return h;
}

UpdateResult Ekf6::apply(const Vec<N> &h, float innovation, float r, NisStats &stats) {
    const UpdateResult result = scalar_update(x_, P_, h, innovation, r, CHI2_1DOF_99);
    // A correction can push theta just past +-pi, through its correlations
    // even when h does not touch it.
    x_[THETA] = wrap_pi(x_[THETA]);
    stats.record(result);
    return result;
}

void Ekf6::feed_ratio(float optical_vx, float optical_vy) {
    if (!wheel_speed_fresh_ || wheel_speed_mm_s_ < params_.optical_ratio_min_speed_mm_s) {
        return;
    }
    // Bring the optical velocity back to the robot centre before comparing.
    const float cx = optical_vx + x_[OMEGA] * params_.optical_lever_y_mm;
    const float cy = optical_vy - x_[OMEGA] * params_.optical_lever_x_mm;
    ratio_optical_sum_ += sqrtf(cx * cx + cy * cy);
    ratio_wheel_sum_ += wheel_speed_mm_s_;
    ++ratio_samples_;

    if (ratio_samples_ >= params_.optical_ratio_window) {
        ratio_last_ = ratio_optical_sum_ / ratio_wheel_sum_;
        ratio_out_of_band_ =
            !(ratio_last_ >= params_.optical_ratio_min && ratio_last_ <= params_.optical_ratio_max);
        ++ratio_windows_;
        if (ratio_out_of_band_) {
            ++ratio_windows_out_;
        }
        ratio_optical_sum_ = 0.0f;
        ratio_wheel_sum_ = 0.0f;
        ratio_samples_ = 0;
    }
}

}  // namespace kalman
