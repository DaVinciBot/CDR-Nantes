/*
 * 6-state EKF [x, y, theta, vx, vy, omega]: the production filter.
 *
 * STATE
 *   x, y   world position, mm
 *   theta  heading, rad, in [-pi, pi)
 *   vx, vy velocity in the ROBOT frame, mm/s
 *   omega  angular rate, rad/s
 * Velocities are kept in the robot frame on purpose: every sensor measures in
 * that frame, so every measurement is LINEAR in the state and only the
 * prediction depends on theta.
 *
 * PREDICTION, constant velocity over dt, integrated at the mid-tick heading:
 *     x     += (cos(m) vx - sin(m) vy) dt        m = theta + omega dt / 2
 *     y     += (sin(m) vx + cos(m) vy) dt
 *     theta += omega dt
 * Process noise is white acceleration, in the robot frame, with two scalars:
 * accel_noise (translation) and alpha_noise (rotation). They are the only two
 * numbers to tune on the table. alpha_noise is deliberately inflated for now:
 * the gyro bias is not modelled (a 7th state may come later), and until it is,
 * the extra noise on omega is what absorbs it.
 * F is the identity plus nine terms; F P F^T is computed from that sparse list,
 * not as two dense 6x6 products.
 *
 * MEASUREMENTS, all scalar, in this order at every tick:
 *   wheels  : three wheel speeds, counts * counts_to_mm / dt. Row = inverse
 *             kinematics, same convention as the motor commands (ekf_kinematics.h).
 *   optical : two velocities at the sensor, vx - omega r_y and vy + omega r_x,
 *             r = lever arm from the centre of rotation. Robot frame; rotating
 *             the sensor's own axes into it is the sensor layer's job.
 *   heading : the IMU heading, innovation wrapped. NOT the gyro rate as well:
 *             the game rotation vector already integrates it, adding it would
 *             count the same measurement twice.
 *
 * Known approximations, both conservative or small:
 *   - a wheel speed is the AVERAGE over the tick, used as the velocity at its
 *     end;
 *   - wheel count jitter is treated as white velocity noise, while a count that
 *     lands in the next tick actually cancels out in position. The filter is
 *     therefore a little pessimistic on position from the encoders.
 *
 * DIAGNOSTICS, none of which ever touches the estimate:
 *   - NIS per source (wheels, optical, heading): the consistency check;
 *   - consecutive optical rejections: the gate catches a sensor that loses
 *     track (it reports near-zero motion while the wheels turn);
 *   - optical / wheel speed ratio over a window: the gate does NOT catch a
 *     sensor that under-counts by 10-30 %, the typical failure of a dusty or
 *     badly mounted optical sensor; this ratio does. What the filter should DO
 *     about it (drop the optical, rescale it) is a decision for the table tests,
 *     so for now it is reported, not acted upon;
 *   - covariance health: theta and velocity variances must stay bounded. x and
 *     y are NOT checked: nothing measures them, their variance grows for the
 *     whole match and that is correct.
 *
 * TIME: the tick carries the dt measured by the control loop, the time master.
 * A zero dt or one longer than max_dt_s is refused: past that, the loop has
 * stalled and the watchdog owns the situation. Sensor samples carry a local
 * timestamp, and one that is not newer than the last applied sample of its
 * source is dropped: the filter never goes back in time.
 *
 * Arduino-free: builds and is tested in the native environment.
 */

#ifndef EKF6_H
#define EKF6_H

#include <stdint.h>

#include "ekf.h"
#include "ekf_kinematics.h"
#include "matrix.h"

namespace kalman {

/** @brief Every tunable of the 6-state filter. */
struct Ekf6Params {
    // --- Process noise ---
    /** White acceleration spectral density, robot frame, mm/s^2/sqrt(Hz). */
    float accel_noise;
    /** White angular acceleration spectral density, rad/s^2/sqrt(Hz). */
    float alpha_noise;

    // --- Measurement noise ---
    /** Std dev of one wheel's count delta over a tick, counts. */
    float wheel_sigma_counts;
    /** Std dev of a wheel speed, as a fraction of that speed (slip). */
    float wheel_slip_ratio;
    /** Std dev of each optical velocity axis, mm/s. */
    float optical_sigma_mm_s;
    /** Std dev of the IMU heading, rad. */
    float heading_sigma_rad;

    // --- Optical sensor mounting ---
    /** Lever arm from the centre of rotation to the optical sensor, mm. */
    float optical_lever_x_mm;
    float optical_lever_y_mm;

    // --- Time ---
    /** Longest tick accepted, s. */
    float max_dt_s;

    // --- Covariance health (theta and velocities only, never x and y) ---
    float max_sigma_theta_rad;
    float max_sigma_v_mm_s;
    float max_sigma_omega_rad_s;

    // --- Optical diagnostics ---
    /** Consecutive gated optical samples before optical_ok() drops. */
    uint32_t optical_max_consecutive_rejections;
    /** Acceptable band of the optical / wheel speed ratio. */
    float optical_ratio_min;
    float optical_ratio_max;
    /** Samples below this wheel speed do not enter the ratio, mm/s. */
    float optical_ratio_min_speed_mm_s;
    /** Samples per ratio window. */
    uint32_t optical_ratio_window;
};

/**
 * @brief Starting values, NOT tuned values.
 *
 * Measurement noises are placeholders until the bench measurement of R (IMU
 * first, optical once the sensor is available). The two process noises are set
 * on the table by NIS. Everything else is a reasoned default.
 */
Ekf6Params provisional_ekf6_params();

/** @brief Outcome of one optical sample (two scalar updates). */
struct OpticalUpdate {
    UpdateResult x;
    UpdateResult y;
    /** True if the sample was dropped for not being newer than the last one. */
    bool stale;
};

class Ekf6 {
public:
    static constexpr size_t N = 6;
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t THETA = 2;
    static constexpr size_t VX = 3;
    static constexpr size_t VY = 4;
    static constexpr size_t OMEGA = 5;

    /**
     * @brief Builds the filter at rest at the origin, with a perfectly known
     * state. Call reset() with the real starting pose before the match.
     */
    Ekf6(const Geometry &geometry, const Ekf6Params &params);

    /**
     * @brief Robot at rest at a known pose.
     *
     * @param x_mm Position x, mm.
     * @param y_mm Position y, mm.
     * @param theta_rad Heading, rad; wrapped on entry.
     * @param sigma_xy_mm Std dev of x and y, mm.
     * @param sigma_theta_rad Std dev of theta, rad.
     * @param sigma_v_mm_s Std dev of vx and vy, mm/s.
     * @param sigma_omega_rad_s Std dev of omega, rad/s.
     */
    void reset(float x_mm, float y_mm, float theta_rad, float sigma_xy_mm,
               float sigma_theta_rad, float sigma_v_mm_s, float sigma_omega_rad_s);

    /** @brief Sets the state and a full covariance (symmetrised on entry). */
    void reset(const Vec<N> &state, const Mat<N> &covariance);

    /**
     * @brief One control-loop tick: prediction, then wheel updates.
     *
     * Skipped, and counted, on a first frame and when the prediction is refused
     * (see predict()). Invalid encoders still predict, but skip the wheel
     * updates: the velocity model carries the pose through a missed read.
     *
     * @return True if the tick was used.
     */
    bool process_tick(const OdometryTick &tick);

    /**
     * @brief Constant-velocity prediction.
     *
     * @param dt_s Tick duration, s. Refused (no-op, counted) unless
     *     0 < dt_s <= max_dt_s; a NaN is refused too.
     * @return True if the prediction ran.
     */
    bool predict(float dt_s);

    /** @brief The three wheel-speed updates of a tick. No-op if unusable. */
    void update_wheels(const OdometryTick &tick);

    /**
     * @brief One optical velocity sample, at the sensor, robot frame.
     *
     * @param vx_mm_s Velocity along robot x, mm/s.
     * @param vy_mm_s Velocity along robot y, mm/s.
     * @param timestamp_us Local time of the sample, us (wraps).
     */
    OpticalUpdate update_optical(float vx_mm_s, float vy_mm_s, uint32_t timestamp_us);

    /**
     * @brief One IMU heading sample.
     *
     * @param heading_rad Heading in the TABLE frame; removing the IMU's boot
     *     origin is the sensor layer's job.
     * @param timestamp_us Local time of the sample, us (wraps).
     * @return The update; accepted is false if the sample was stale or gated.
     */
    UpdateResult update_heading(float heading_rad, uint32_t timestamp_us);

    // === ESTIMATE ===

    const Vec<N> &state() const { return x_; }
    const Mat<N> &covariance() const { return P_; }

    // === DIAGNOSTICS ===

    /** @return False if theta or a velocity variance is out of bounds or NaN. */
    bool covariance_healthy() const;

    /** @return False after too many consecutive gated optical samples, or
     *  while the last optical / wheel speed ratio is out of its band. */
    bool optical_ok() const;

    const NisStats &wheel_stats() const { return wheel_stats_; }
    const NisStats &optical_stats() const { return optical_stats_; }
    const NisStats &heading_stats() const { return heading_stats_; }

    uint32_t ticks_used() const { return ticks_used_; }
    uint32_t ticks_skipped() const { return ticks_skipped_; }
    uint32_t predictions_refused() const { return predictions_refused_; }
    uint32_t stale_samples() const { return stale_samples_; }
    uint32_t optical_consecutive_rejections() const { return optical_consecutive_rejections_; }

    /** @return Ratio of the last complete window, 0 before the first one. */
    float optical_ratio() const { return ratio_last_; }
    uint32_t optical_ratio_windows() const { return ratio_windows_; }
    uint32_t optical_ratio_windows_out_of_band() const { return ratio_windows_out_; }

    // === MODEL, EXPOSED FOR THE TESTS ===
    // predict() uses the same linearisation, from a sparse list of the terms of
    // F. The tests check it against these dense forms, and these against finite
    // differences of transition().

    /** @return The state after dt, without noise. */
    static Vec<N> transition(const Vec<N> &state, float dt_s);

    /** @brief Dense Jacobian of transition() at (state, dt). */
    static void jacobian(const Vec<N> &state, float dt_s, Mat<N> &F);

    /** @return Process noise over dt, linearised at state. */
    Mat<N> process_noise(const Vec<N> &state, float dt_s) const;

    /** @return Measurement row of wheel 0, 1 or 2 (a speed, mm/s). */
    Vec<N> wheel_row(size_t wheel) const;

private:
    /** Applies one scalar update, keeps theta wrapped, records the NIS. */
    UpdateResult apply(const Vec<N> &h, float innovation, float r, NisStats &stats);

    /** Feeds one optical sample to the optical / wheel speed ratio window. */
    void feed_ratio(float optical_vx, float optical_vy);

    Geometry geometry_;
    Ekf6Params params_;
    Vec<N> x_;
    Mat<N> P_;

    NisStats wheel_stats_;
    NisStats optical_stats_;
    NisStats heading_stats_;

    uint32_t ticks_used_ = 0;
    uint32_t ticks_skipped_ = 0;
    uint32_t predictions_refused_ = 0;
    uint32_t stale_samples_ = 0;

    bool optical_seen_ = false;
    uint32_t optical_last_us_ = 0;
    bool heading_seen_ = false;
    uint32_t heading_last_us_ = 0;

    uint32_t optical_consecutive_rejections_ = 0;

    /** Robot-centre speed from the wheels, this tick; the ratio's reference. */
    bool wheel_speed_fresh_ = false;
    float wheel_speed_mm_s_ = 0.0f;

    float ratio_optical_sum_ = 0.0f;
    float ratio_wheel_sum_ = 0.0f;
    uint32_t ratio_samples_ = 0;
    float ratio_last_ = 0.0f;
    bool ratio_out_of_band_ = false;
    uint32_t ratio_windows_ = 0;
    uint32_t ratio_windows_out_ = 0;
};

}  // namespace kalman

#endif  // EKF6_H
