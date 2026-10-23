/*
 * Pose estimation of the holonomic base: the contract shared by every estimator.
 *
 * The control loop owns the sensors and the time; an estimator owns nothing but
 * its state. Once per tick the loop fills an EstimatorInput from its buffers
 * (encoders read over RS485, IMU and optical sensor read over I2C, all of it in
 * loop(), never in the ISR) and hands it to the active estimator, then reads the
 * pose back. Two estimators implement the contract:
 *
 *   ComplementaryEstimator  complementary filter, 20 % encoders / 80 % optical
 *                           on the displacement of the tick;
 *   EkfEstimator            the 6-state EKF of lib/kalman.
 *
 * Both are compiled in, only one runs: the board picks one at boot and may
 * switch at rest, the new one starting from the pose of the old one.
 *
 * Units: mm, rad, us. Angles in [-pi, pi).
 *
 * This header must stay free of any Arduino include, so lib/pose_estimator
 * builds and is tested in the "native" environment.
 */

#ifndef POSE_ESTIMATOR_H
#define POSE_ESTIMATOR_H

#include <stdint.h>

namespace pose {

/**
 * @brief Everything an estimator may use from one control-loop tick.
 *
 * Each sensor block carries its own validity flag: a sensor that has nothing to
 * say clears its flag, it never reports zeros.
 */
struct EstimatorInput {
    // --- Wheel encoders ---
    /** Count deltas of wheels 1, 2 and 3 since the previous tick, sign already
     *  flipped so that forward wheel rotation is positive (kinematics.h). */
    int32_t d1;
    int32_t d2;
    int32_t d3;
    /** Time since the previous tick, us. */
    uint32_t dt_us;
    /** First tick after boot or a reset: the deltas mean nothing. */
    bool first_frame;
    /** True when the deltas span exactly one tick: a fresh encoder read this
     *  tick and the previous one. False after a missed read, when the deltas
     *  are zero (stale buffer) or cover two ticks at once. */
    bool encoders_fresh;
    /** Age of the last successful encoder read, us. */
    uint32_t encoder_age_us;

    // --- IMU (BNO085) ---
    bool imu_valid;
    /** Heading in the TABLE frame, boot origin already removed, rad. */
    float heading_rad;
    /** Local time of the sample, us (wraps). */
    uint32_t imu_timestamp_us;

    // --- Optical sensor: one NEW sample, or both flags cleared ---
    /** Local time of the sample, us (wraps). */
    uint32_t optical_timestamp_us;

    /** Robot-frame step since the previous sample (what the complementary
     *  filter reads), and the time it covers. */
    bool optical_step_valid;
    float optical_dx_mm;
    float optical_dy_mm;
    uint32_t optical_step_dt_us;

    /** Robot-frame velocity (what the EKF reads). */
    bool optical_velocity_valid;
    float optical_vx_mm_s;
    float optical_vy_mm_s;
};

/** @return An input with every flag cleared and every value at zero. */
EstimatorInput empty_input();

/** @brief Pose and velocity of the robot centre, table frame. */
struct PoseEstimate {
    float x_mm;
    float y_mm;
    float theta_rad;
    float vx_mm_s;
    float vy_mm_s;
    float omega_rad_s;
};

/** Health flags, OR-ed into the byte returned by health_flags(). */
enum HealthFlag : uint8_t {
    /** The estimate can be trusted (for the EKF: covariance healthy). */
    FILTER_READY = 1u << 0,
    /** The optical sensor is in use and agrees with the rest. */
    OPTICAL_OK = 1u << 1,
    /** At least one measurement was rejected during the last tick. */
    MEAS_REJECTED = 1u << 2,
};

/** @brief The interface the control loop talks to. */
class PoseEstimator {
public:
    virtual ~PoseEstimator() = default;

    /** @brief Robot at rest at a known pose; forgets every past sample. */
    virtual void reset(float x_mm, float y_mm, float theta_rad) = 0;

    /** @brief One control-loop tick. Called from the ISR: no I/O, no allocation. */
    virtual void step(const EstimatorInput &input) = 0;

    /** @return The current estimate. */
    virtual PoseEstimate pose() const = 0;

    /** @return HealthFlag bits. */
    virtual uint8_t health_flags() const = 0;

    /** @return A short name for logs. */
    virtual const char *name() const = 0;
};

/** @brief Wraps an angle into [-pi, pi). */
double normalize_angle(double theta);

}  // namespace pose

#endif  // POSE_ESTIMATOR_H
