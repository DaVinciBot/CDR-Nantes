/*
 * Extended Kalman filter for the holonomic base: the core shared by every
 * model (angle wrapping, scalar measurement update, gate, NIS statistics).
 *
 * Models, from the simplest:
 *   3 states [x, y, theta], wheel odometry as the control input u. Assumed
 *           fallback if the 6-state version is not tuned in time, and a trivial
 *           model on which to debug the whole plumbing.
 *   6 states [x, y, theta, vx, vy, omega], velocities in the robot frame, every
 *           sensor as scalar measurements. The target.
 *   A 7th state b_omega (gyro bias) may come later, not before the 6-state
 *   filter is tuned on the table. Until then, the process noise on omega is
 *   deliberately inflated to absorb the residual bias.
 *
 * Units: mm and rad, in float. Angles normalized to [-pi, pi).
 *
 * THREE TRAPS THIS FILTER MUST AVOID, all of them already sprung once:
 *
 * 1. An INTEGRATED pose is never a measurement. The 2026 prototype fed the
 *    Teensy odometry pose in as z[0..2], which makes the filter correct itself
 *    with its own prediction. Raw sensor quantities are fine: wheel speeds, an
 *    optical displacement, an IMU heading. A pose built by integrating them is
 *    not.
 *
 * 2. Do not double-count correlated measurements. The prototype also fed
 *    z[3..4] = sqrt(px^2+py^2), atan2(py, px), calling it a LiDAR reading: the
 *    same x and y restated in polar form, with an independent R. The result is
 *    guaranteed overconfidence. The live versions of this trap: the OTOS fuses
 *    its own IMU internally, so its (dx, dy) is taken as robot-frame
 *    displacement and its theta is IGNORED; and the BNO085 game rotation vector
 *    already integrates the gyro, so its heading is used and its rate is NOT
 *    added on top.
 *
 * 3. The P_MAX guard must NOT cover x and y. With the LiDAR out of the filter
 *    (decided 05/09/2026) there is no absolute position measurement at all, so X
 *    and Y are unobservable and their variance grows for the whole match. That is
 *    correct behaviour; a guard on it would trip every single match.
 *
 * VALIDATION IS BY CONSISTENCY, NOT BY ERROR ALONE. A filter can have a good
 * RMSE and a wrong covariance: in the Q/R sweep of
 * FERBIN12/robot-sensor-fusion-ekf (GitHub) the lowest-RMSE tuning was
 * overconfident (mean NIS 16.4) while the consistent one (NIS 1.005) cost
 * only 32 mm more. Sigma is the interface the high level will act on, so a wrong
 * covariance is a bug even when the trajectory looks right. Every update
 * therefore reports its NIS, and NisStats accumulates it per source.
 *
 * No sigma is published on the link yet: it only means something once Q is
 * tuned, which is planned for January. An unvalidated sigma is worse than
 * none, because it gets trusted.
 *
 * This header must stay free of any Arduino include, so lib/kalman builds in the
 * "native" environment.
 */

#ifndef EKF_H
#define EKF_H

#include <stdint.h>

#include "matrix.h"

namespace kalman {

// === CONSTANTS ===

constexpr float PI_F = 3.14159265358979f;
constexpr float TWO_PI_F = 6.28318530717959f;

/**
 * 99th percentile of a chi-square distribution with one degree of freedom.
 *
 * Every update is scalar, so this is the gate for every source. 99 % rejects one
 * good measurement in a hundred; a tighter gate rejects good measurements often
 * enough to degrade the estimate, which is the documented failure mode.
 */
constexpr float CHI2_1DOF_99 = 6.635f;

// === ANGLES ===

/**
 * @brief Wraps an angle into [-pi, pi).
 *
 * @param angle Any finite angle, in rad.
 * @return The same direction, in [-pi, pi).
 */
float wrap_pi(float angle);

/**
 * @brief Innovation between two angles, taking the short way round.
 *
 * Measured +179 deg against predicted -179 deg is an innovation of -2 deg, not
 * +358 deg. Getting this wrong makes the filter fight itself every time the
 * heading crosses +-pi, and an RMSE on a trajectory that never crosses it will
 * not show it.
 *
 * @param measured The measured angle, in rad.
 * @param predicted The predicted angle, in rad.
 * @return wrap_pi(measured - predicted).
 */
inline float angle_innovation(float measured, float predicted) {
    return wrap_pi(measured - predicted);
}

// === SCALAR UPDATE ===

/** @brief What a scalar update did, for the caller and for NisStats. */
struct UpdateResult {
    /** z - h(x), as passed in. */
    float innovation;
    /** Innovation variance s = h P h^T + r. */
    float variance;
    /** Normalized innovation squared, innovation^2 / variance. */
    float nis;
    /** False if the gate or a numerical check rejected the update. */
    bool accepted;
};

/**
 * @brief Applies one scalar measurement to the state and its covariance.
 *
 * The measurement is z = h x + v, v ~ N(0, r), and the caller passes the
 * innovation z - h(x) already computed (wrapped, for an angle). For a nonlinear
 * h, pass its Jacobian row as h.
 *
 * The covariance update is the Joseph form,
 *     P <- (I - K h) P (I - K h)^T + r K K^T,
 * expanded for a scalar measurement into
 *     P <- P - K PHt^T - PHt K^T + s K K^T,   PHt = P h^T, s = h PHt + r.
 * The expansion is exact for any K, which is the whole point of Joseph: the K
 * computed in float32 is not exactly optimal, and the short form P - K h P
 * silently loses positive-definiteness when it is not. Only the upper triangle
 * is computed and then mirrored, so P comes out exactly symmetric.
 *
 * Cost is O(N^2) and nothing is inverted.
 *
 * The update is REJECTED, leaving x and P untouched, when:
 *   - nis > gate: an outlier, e.g. an optical sensor losing track;
 *   - the variance is not strictly positive, or anything is NaN: the comparison
 *     is written so that a NaN always rejects, never accepts.
 *
 * @param x State, updated in place.
 * @param P Covariance, updated in place. Must be symmetric on entry.
 * @param h Measurement row (Jacobian row for a nonlinear measurement).
 * @param innovation z - h(x).
 * @param r Measurement noise variance.
 * @param gate NIS threshold; CHI2_1DOF_99 in production, INFINITY to disable.
 * @return What happened, including the NIS even when rejected.
 */
template <size_t N>
UpdateResult scalar_update(Vec<N> &x, Mat<N> &P, const Vec<N> &h,
                           float innovation, float r, float gate) {
    float pht[N];
    float variance = r;
    for (size_t i = 0; i < N; ++i) {
        float sum = 0.0f;
        for (size_t j = 0; j < N; ++j) {
            sum += P.m[i][j] * h.v[j];
        }
        pht[i] = sum;
        variance += h.v[i] * sum;
    }

    UpdateResult result;
    result.innovation = innovation;
    result.variance = variance;
    result.nis = 0.0f;
    result.accepted = false;

    if (!(variance > 0.0f)) {
        return result;
    }
    result.nis = innovation * innovation / variance;
    // Written as !(nis <= gate) rather than (nis > gate): a NaN compares false
    // either way, and it must fall on the rejecting side.
    if (!(result.nis <= gate)) {
        return result;
    }

    float gain[N];
    for (size_t i = 0; i < N; ++i) {
        gain[i] = pht[i] / variance;
        x.v[i] += gain[i] * innovation;
    }
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i; j < N; ++j) {
            const float value = P.m[i][j] - gain[i] * pht[j] - pht[i] * gain[j] +
                                variance * gain[i] * gain[j];
            P.m[i][j] = value;
            P.m[j][i] = value;
        }
    }

    result.accepted = true;
    return result;
}

// === ODOMETRY TICK ===

/**
 * @brief One tick of wheel odometry, as the Teensy Moteur sends it.
 *
 * The fields mirror the EncoderFrame, but lib/kalman does not include
 * teensy_link: the filter must not depend on the wire format, and the caller
 * copies the fields across.
 */
struct OdometryTick {
    /** Raw count deltas of wheels 1, 2 and 3 since the previous tick. */
    int32_t d1;
    int32_t d2;
    int32_t d3;
    /** Measured time since the previous tick, us. */
    uint32_t dt_us;
    /** First tick after boot or a tracking reset: the deltas mean nothing. */
    bool first_frame;
    /** False when the Moteur could not read its encoders this tick. */
    bool encoders_valid;
};

// === NIS STATISTICS ===

/**
 * @brief Running NIS statistics for one measurement source.
 *
 * For a consistent filter the NIS of a scalar measurement follows a chi-square
 * with one degree of freedom, so its mean is 1. The mean here is taken over the
 * ACCEPTED updates only, so that a sensor dropout does not drown the statistic;
 * truncating a chi-square(1) at the 99 % gate lowers its mean to about 0.93,
 * which stays well inside the [0.7, 1.3] consistency band used on the table.
 * Rejections are counted separately: their rate is its own diagnostic.
 */
struct NisStats {
    uint32_t accepted = 0;
    uint32_t rejected = 0;
    float nis_sum = 0.0f;

    /** @brief Accounts for one update. */
    void record(const UpdateResult &result);

    /** @return Mean NIS over accepted updates, 0 when there are none. */
    float mean_nis() const;

    /** @brief Clears the statistics. */
    void reset();
};

}  // namespace kalman

#endif  // EKF_H
