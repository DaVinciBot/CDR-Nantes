/*
 * 3-state EKF [x, y, theta]. See include/ekf3.h for the model.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "ekf3.h"

#include <math.h>

namespace kalman {

namespace {

/** @brief Everything the prediction needs from the mid-tick heading. */
struct Linearization {
    float c;  // cos(m)
    float s;  // sin(m)
    float a;  // d x' / d theta = -sin(m) dx - cos(m) dy
    float b;  // d y' / d theta =  cos(m) dx - sin(m) dy
};

Linearization linearize(float theta, const BodyDelta &u) {
    const float mid = theta + 0.5f * u.dtheta;
    Linearization lin;
    lin.c = cosf(mid);
    lin.s = sinf(mid);
    lin.a = -lin.s * u.dx - lin.c * u.dy;
    lin.b = lin.c * u.dx - lin.s * u.dy;
    return lin;
}

}  // namespace

Ekf3::Ekf3(const Geometry &geometry, const Ekf3Params &params)
    : geometry_(geometry), params_(params) {
    reset(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void Ekf3::reset(float x_mm, float y_mm, float theta_rad, float sigma_xy_mm,
                 float sigma_theta_rad) {
    x_[X] = x_mm;
    x_[Y] = y_mm;
    x_[THETA] = wrap_pi(theta_rad);
    P_ = Mat<N>::zero();
    P_(X, X) = sigma_xy_mm * sigma_xy_mm;
    P_(Y, Y) = sigma_xy_mm * sigma_xy_mm;
    P_(THETA, THETA) = sigma_theta_rad * sigma_theta_rad;
}

void Ekf3::reset(const Vec<N> &state, const Mat<N> &covariance) {
    x_ = state;
    x_[THETA] = wrap_pi(x_[THETA]);
    P_ = covariance;
    symmetrize(P_);
}

bool Ekf3::predict_tick(const OdometryTick &tick) {
    if (tick.first_frame || !tick.encoders_valid) {
        ++ticks_skipped_;
        return false;
    }
    predict(forward_kinematics(geometry_, tick.d1, tick.d2, tick.d3),
            odometry_covariance(tick.d1, tick.d2, tick.d3));
    ++ticks_used_;
    return true;
}

void Ekf3::predict(const BodyDelta &u, const Mat<N> &sigma_u) {
    // Linearise at the state BEFORE it moves.
    const Linearization lin = linearize(x_[THETA], u);

    x_[X] += lin.c * u.dx - lin.s * u.dy;
    x_[Y] += lin.s * u.dx + lin.c * u.dy;
    x_[THETA] = wrap_pi(x_[THETA] + u.dtheta);

    // F P F^T. F is the identity except its theta column, (a, b, 1), so
    // F = I + v e_theta^T with v = (a, b, 0), and
    //     F P F^T = P + v p^T + p v^T + p_theta_theta v v^T,   p = P e_theta.
    // Nine multiply-adds instead of two 3x3 products.
    const float v[N] = {lin.a, lin.b, 0.0f};
    const float p[N] = {P_(X, THETA), P_(Y, THETA), P_(THETA, THETA)};
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i; j < N; ++j) {
            const float value =
                P_(i, j) + v[i] * p[j] + p[i] * v[j] + p[THETA] * v[i] * v[j];
            P_(i, j) = value;
            P_(j, i) = value;
        }
    }

    // + G Sigma_u G^T, with G = d transition / d u:
    //     | c  -s  a/2 |
    //     | s   c  b/2 |
    //     | 0   0   1  |
    const float G[N][N] = {
        {lin.c, -lin.s, 0.5f * lin.a},
        {lin.s, lin.c, 0.5f * lin.b},
        {0.0f, 0.0f, 1.0f},
    };
    float g_sigma[N][N];
    for (size_t i = 0; i < N; ++i) {
        for (size_t k = 0; k < N; ++k) {
            float sum = 0.0f;
            for (size_t l = 0; l < N; ++l) {
                sum += G[i][l] * sigma_u(l, k);
            }
            g_sigma[i][k] = sum;
        }
    }
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i; j < N; ++j) {
            float q = 0.0f;
            for (size_t k = 0; k < N; ++k) {
                q += g_sigma[i][k] * G[j][k];
            }
            P_(i, j) += q;
            if (j != i) {
                P_(j, i) = P_(i, j);
            }
        }
    }
}

UpdateResult Ekf3::update_heading(float heading_rad) {
    Vec<N> h = Vec<N>::zero();
    h[THETA] = 1.0f;
    const float r = params_.heading_sigma_rad * params_.heading_sigma_rad;

    const UpdateResult result =
        scalar_update(x_, P_, h, angle_innovation(heading_rad, x_[THETA]), r,
                      CHI2_1DOF_99);
    // The correction can push theta just past +-pi.
    x_[THETA] = wrap_pi(x_[THETA]);
    heading_stats_.record(result);
    return result;
}

Mat<Ekf3::N> Ekf3::odometry_covariance(int32_t d1, int32_t d2, int32_t d3) const {
    const int32_t counts[3] = {d1, d2, d3};
    const float floor_mm = params_.wheel_sigma_counts * geometry_.counts_to_mm;
    float variance[3];
    for (size_t k = 0; k < 3; ++k) {
        const float slip_mm =
            params_.wheel_slip_ratio * static_cast<float>(counts[k]) * geometry_.counts_to_mm;
        variance[k] = floor_mm * floor_mm + slip_mm * slip_mm;
    }
    return body_delta_covariance(geometry_, variance);
}

Vec<Ekf3::N> Ekf3::transition(const Vec<N> &state, const BodyDelta &u) {
    const Linearization lin = linearize(state[THETA], u);
    Vec<N> next;
    next[X] = state[X] + lin.c * u.dx - lin.s * u.dy;
    next[Y] = state[Y] + lin.s * u.dx + lin.c * u.dy;
    next[THETA] = wrap_pi(state[THETA] + u.dtheta);
    return next;
}

void Ekf3::jacobians(const Vec<N> &state, const BodyDelta &u, Mat<N> &F,
                     Mat<N> &G) {
    const Linearization lin = linearize(state[THETA], u);

    F = Mat<N>::identity();
    F(X, THETA) = lin.a;
    F(Y, THETA) = lin.b;

    G = Mat<N>::zero();
    G(X, 0) = lin.c;
    G(X, 1) = -lin.s;
    G(X, 2) = 0.5f * lin.a;
    G(Y, 0) = lin.s;
    G(Y, 1) = lin.c;
    G(Y, 2) = 0.5f * lin.b;
    G(THETA, 2) = 1.0f;
}

}  // namespace kalman
