/*
 * 3-state EKF [x, y, theta]: wheel odometry as the control input, IMU heading as
 * the only measurement.
 *
 * WHY IT EXISTS: it is the assumed fallback if the 6-state filter is not tuned
 * in time, and a model trivial enough to debug the whole plumbing on (tick,
 * flags, pose output) before the real filter goes in.
 *
 * MODEL
 *   u = (dx, dy, dtheta), the robot-frame displacement of one tick, from the
 *       wheel counts through the forward kinematics (ekf_kinematics.h).
 *   Prediction integrates u at the MID-TICK heading:
 *       x     += cos(m) dx - sin(m) dy        m = theta + dtheta / 2
 *       y     += sin(m) dx + cos(m) dy
 *       theta += dtheta
 *   Process noise Q = G Sigma_u G^T, where Sigma_u comes from per-wheel noise:
 *   a floor in counts (quantisation, read jitter) plus a fraction of the travel
 *   (slip). Nothing else feeds Q: this filter has no velocity, so it has no
 *   acceleration noise either.
 *   Measurement: the IMU heading, theta, with a wrapped innovation.
 *
 * TIME IS NOT USED. The model is driven by displacement, so dt_us plays no part:
 * a late or bunched tick still carries a valid displacement, and cannot corrupt
 * this filter. That is a property of this model only; the 6-state one needs dt.
 *
 * X and Y are unobservable (there is no absolute position measurement), so
 * their variance grows for the whole match. That is correct, and there is no
 * guard on it.
 *
 * Arduino-free: builds and is tested in the native environment.
 */

#ifndef EKF3_H
#define EKF3_H

#include <stdint.h>

#include "ekf.h"
#include "ekf_kinematics.h"
#include "matrix.h"

namespace kalman {

/** @brief Noise parameters of the 3-state filter. */
struct Ekf3Params {
    /** Std dev of one wheel's count delta over a tick, counts. */
    float wheel_sigma_counts;
    /** Std dev of a wheel's travel error, as a fraction of that travel (slip). */
    float wheel_slip_ratio;
    /** Std dev of the IMU heading measurement, rad. */
    float heading_sigma_rad;
};

class Ekf3 {
public:
    static constexpr size_t N = 3;
    static constexpr size_t X = 0;
    static constexpr size_t Y = 1;
    static constexpr size_t THETA = 2;

    /**
     * @brief Builds the filter at the origin, with a perfectly known pose.
     *
     * Call reset() with the real starting pose before the match.
     */
    Ekf3(const Geometry &geometry, const Ekf3Params &params);

    /**
     * @brief Sets the pose and a diagonal covariance.
     *
     * @param x_mm Position x, mm.
     * @param y_mm Position y, mm.
     * @param theta_rad Heading, rad; wrapped on entry.
     * @param sigma_xy_mm Std dev of x and y, mm.
     * @param sigma_theta_rad Std dev of theta, rad.
     */
    void reset(float x_mm, float y_mm, float theta_rad, float sigma_xy_mm,
               float sigma_theta_rad);

    /** @brief Sets the state and a full covariance (symmetrised on entry). */
    void reset(const Vec<N> &state, const Mat<N> &covariance);

    /**
     * @brief Prediction from one odometry tick.
     *
     * Skipped, and counted, on a first frame (its deltas are measured against
     * nothing) and when the encoders are flagged invalid. A skipped tick freezes
     * the pose; reporting that downstream is the caller's job.
     *
     * @param tick The tick, as built by the control loop.
     * @return True if the tick was used.
     */
    bool predict_tick(const OdometryTick &tick);

    /**
     * @brief Prediction from a known displacement and its covariance.
     *
     * @param u Robot-frame displacement of the tick.
     * @param sigma_u Covariance of u.
     */
    void predict(const BodyDelta &u, const Mat<N> &sigma_u);

    /**
     * @brief Measurement update from the IMU heading.
     *
     * @param heading_rad Heading in the TABLE frame. The IMU yaw has an
     *     arbitrary origin at boot; removing it is the sensor layer's job.
     * @return What the update did; also recorded in heading_stats().
     */
    UpdateResult update_heading(float heading_rad);

    /** @return Covariance of the displacement computed from these counts. */
    Mat<N> odometry_covariance(int32_t d1, int32_t d2, int32_t d3) const;

    const Vec<N> &state() const { return x_; }
    const Mat<N> &covariance() const { return P_; }
    const NisStats &heading_stats() const { return heading_stats_; }
    uint32_t ticks_used() const { return ticks_used_; }
    uint32_t ticks_skipped() const { return ticks_skipped_; }

    // === MODEL, EXPOSED FOR THE TESTS ===
    // predict() does not call these: it uses the same linearisation, expanded
    // by hand. The tests check the expansion against these dense matrices, and
    // these matrices against finite differences of transition().

    /** @return The state after one displacement, without noise. */
    static Vec<N> transition(const Vec<N> &state, const BodyDelta &u);

    /**
     * @brief Dense Jacobians of transition() at (state, u).
     *
     * @param F Out: d transition / d state.
     * @param G Out: d transition / d u.
     */
    static void jacobians(const Vec<N> &state, const BodyDelta &u, Mat<N> &F,
                          Mat<N> &G);

private:
    Geometry geometry_;
    Ekf3Params params_;
    Vec<N> x_;
    Mat<N> P_;
    NisStats heading_stats_;
    uint32_t ticks_used_ = 0;
    uint32_t ticks_skipped_ = 0;
};

}  // namespace kalman

#endif  // EKF3_H
