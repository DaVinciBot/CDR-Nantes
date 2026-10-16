/*
 * Complementary filter: wheel encoders + optical sensor + IMU heading.
 *
 * This is the 2026 estimator of holonomic_basis, moved here WITHOUT changing its
 * behaviour, so that it can be tested on the host and swapped with the EKF. The
 * only difference is where the data comes from: the optical pose arrives in the
 * EstimatorInput instead of an I2C read inside the ISR. It is rebuilt by
 * OpticalWorldIntegrator from the sensor's robot-frame steps, rotated by the
 * estimator's heading, since the sensor no longer rotates them itself.
 *
 * What it does, per tick:
 *   - encoder deltas under encoder_noise_counts are zeroed;
 *   - optical: the sensor's absolute pose is differenced; a step above
 *     optical_outlier_mm is rejected and counted, one under optical_noise_mm is
 *     zeroed. The optical step is in the table frame (see above);
 *   - pure rotation (three wheels same sign, speeds within 20 %) discards the
 *     optical step;
 *   - fusion: alpha_encoders of the encoders and the rest from the optical,
 *     or 100 % optical if the last encoder read is older than encoder_max_age_us;
 *   - heading: the IMU heading when valid, the encoder rotation otherwise;
 *   - without optical, the encoder displacement is integrated at the mean
 *     heading of the tick, if use_encoders is set.
 *
 * Known weaknesses kept on purpose (behaviour preserved, to be fixed in its own
 * change, with these tests as the safety net):
 *   - the 2 mm optical noise floor blinds the optical below 2 mm per tick, and
 *     the 15 mm outlier bound sits right at full speed at 1 m/s and 15 ms;
 *   - the mean heading of a tick is a plain average, wrong across +-pi.
 *
 * Arduino-free: builds and is tested in the native environment.
 */

#ifndef COMPLEMENTARY_ESTIMATOR_H
#define COMPLEMENTARY_ESTIMATOR_H

#include <stdint.h>

#include "pose_estimator.h"

namespace pose {

/** @brief Every constant of the complementary filter, at its 2026 value. */
struct ComplementaryParams {
    /** Centre of the robot to the wheel contact, mm. No default: board geometry. */
    double robot_radius_mm = 0.0;
    /** Wheel travel per encoder count, mm. No default: board geometry. */
    double counts_to_mm = 0.0;

    /** Encoder deltas below this are zeroed, counts. */
    double encoder_noise_counts = 16.0;

    /** An optical step above this is an outlier, mm per tick. */
    double optical_outlier_mm = 15.0;
    /** An optical step below this is noise, zeroed, mm per tick. */
    double optical_noise_mm = 2.0;
    /** Smallest accumulated optical motion that counts as movement, mm. */
    double optical_motion_mm = 0.01;

    /** Pure-rotation detection: smallest encoder rotation looked at, rad. */
    double pure_rotation_min_rad = 0.005;
    /** Pure-rotation detection: smallest mean wheel delta, counts. */
    double pure_rotation_min_counts = 50.0;
    /** Pure-rotation detection: largest relative spread of wheel deltas. */
    double pure_rotation_tolerance = 0.2;

    /** Share of the encoders in the fused displacement. */
    double alpha_encoders = 0.20;
    /** An encoder read older than this gets no share, us. */
    uint32_t encoder_max_age_us = 50000;

    /** Integrate the encoders alone when the optical sensor is inactive. */
    bool use_encoders = true;
};

class ComplementaryEstimator : public PoseEstimator {
public:
    explicit ComplementaryEstimator(const ComplementaryParams &params);

    void reset(float x_mm, float y_mm, float theta_rad) override;
    void step(const EstimatorInput &input) override;
    PoseEstimate pose() const override;
    uint8_t health_flags() const override;
    const char *name() const override { return "complementary"; }

    // === DIAGNOSTICS ===

    /** Accumulated accepted optical motion since boot, mm. */
    double optical_x_acc() const { return optical_x_acc_; }
    double optical_y_acc() const { return optical_y_acc_; }
    /** Optical samples that contributed motion. */
    uint32_t optical_valid_count() const { return optical_valid_count_; }
    /** Optical samples rejected as outliers. */
    uint32_t optical_outlier_count() const { return optical_outlier_count_; }
    /** Accumulated encoder motion in the world frame since boot, mm. */
    double encoder_x_acc() const { return encoder_x_acc_; }
    double encoder_y_acc() const { return encoder_y_acc_; }

private:
    /** Differences one optical pose sample into the accumulators. */
    void integrate_optical(double x_mm, double y_mm);

    ComplementaryParams params_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    /** Heading at the end of the previous tick, for the mean heading. */
    double last_theta_ = 0.0;
    bool ready_ = false;

    bool optical_seen_ = false;
    double last_optical_x_ = 0.0;
    double last_optical_y_ = 0.0;
    double optical_x_acc_ = 0.0;
    double optical_y_acc_ = 0.0;
    uint32_t optical_valid_count_ = 0;
    uint32_t optical_outlier_count_ = 0;
    bool optical_active_ = false;
    bool optical_rejected_ = false;

    double encoder_x_acc_ = 0.0;
    double encoder_y_acc_ = 0.0;
};

}  // namespace pose

#endif  // COMPLEMENTARY_ESTIMATOR_H
