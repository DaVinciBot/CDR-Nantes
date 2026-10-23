/*
 * Complementary filter: wheel encoders + optical sensor + IMU heading.
 *
 * The 2026 filter of holonomic_basis (20 % encoders, 80 % optical), rewritten
 * on the common contract: it reads the robot-frame optical step the sensor
 * layer already computes, and its thresholds are speeds, so they keep their
 * meaning whatever the tick period and when an optical read is missed.
 *
 * What it does, per tick (dt = the tick duration):
 *   - encoder deltas under encoder_noise_counts are zeroed, then turned into a
 *     robot-frame displacement by the forward kinematics (lib/kinematics);
 *   - heading: the IMU heading when valid, the encoder rotation otherwise;
 *   - optical: the step of a new sample is divided by the time it covers, which
 *     gives the optical speed. Above optical_outlier_mm_s it is rejected and
 *     counted; below optical_noise_mm_s it is noise and not used. Otherwise its
 *     displacement over the tick is that speed times dt (equal to the step
 *     itself when the sensor is read once per tick);
 *   - pure rotation (three wheels same sign, speeds within 20 %) discards the
 *     optical step: the sensor is off the centre of rotation, its offset is
 *     compensated in the EKF only, so here it would read a fake translation;
 *   - fusion, robot frame: alpha_encoders of the encoders and the rest from the
 *     optical, or 100 % optical if the encoder deltas do not span exactly this
 *     tick (missed read); without a usable optical step, the encoders alone if
 *     use_encoders is set;
 *   - the displacement is turned into the table frame at the mean heading of
 *     the tick, computed across +-pi correctly;
 *   - velocity: the table-frame displacement and the heading change over dt.
 *
 * Arduino-free: builds and is tested in the native environment.
 */

#ifndef COMPLEMENTARY_ESTIMATOR_H
#define COMPLEMENTARY_ESTIMATOR_H

#include <stdint.h>

#include "kinematics.h"
#include "pose_estimator.h"

namespace pose {

/** @brief Every constant of the complementary filter. */
struct ComplementaryParams {
    /** Centre of the robot to the wheel contact, mm. No default: board geometry. */
    double robot_radius_mm = 0.0;
    /** Wheel travel per encoder count, mm. No default: board geometry. */
    double counts_to_mm = 0.0;

    /** Encoder deltas below this are zeroed, counts. */
    double encoder_noise_counts = 16.0;

    /** An optical speed above this is an outlier, mm/s: 1.5 times the top
     *  speed of the base (1 m/s). */
    double optical_outlier_mm_s = 1500.0;
    /** An optical speed below this is noise, mm/s. Provisional: to set from
     *  the sensor at rest. (The 2026 floor was 2 mm per tick, i.e. 200 mm/s at
     *  10 ms: the optical was blind below 0.2 m/s.) */
    double optical_noise_mm_s = 5.0;

    /** Pure-rotation detection: smallest encoder rotation looked at, rad. */
    double pure_rotation_min_rad = 0.005;
    /** Pure-rotation detection: smallest mean wheel delta, counts. */
    double pure_rotation_min_counts = 50.0;
    /** Pure-rotation detection: largest relative spread of wheel deltas. */
    double pure_rotation_tolerance = 0.2;

    /** Share of the encoders in the fused displacement. */
    double alpha_encoders = 0.20;

    /** Integrate the encoders alone when there is no usable optical step. */
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

    /** Accumulated optical motion used since boot, table frame, mm. */
    double optical_x_acc() const { return optical_x_acc_; }
    double optical_y_acc() const { return optical_y_acc_; }
    /** Optical samples that contributed motion. */
    uint32_t optical_valid_count() const { return optical_valid_count_; }
    /** Optical samples rejected as outliers. */
    uint32_t optical_outlier_count() const { return optical_outlier_count_; }
    /** Accumulated encoder motion since boot, table frame, mm. */
    double encoder_x_acc() const { return encoder_x_acc_; }
    double encoder_y_acc() const { return encoder_y_acc_; }

private:
    /** True if the wheel deltas describe a rotation in place. */
    bool is_pure_rotation(double d1, double d2, double d3) const;

    ComplementaryParams params_;
    kinematics::Geometry geometry_;

    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
    double vx_ = 0.0;
    double vy_ = 0.0;
    double omega_ = 0.0;
    bool ready_ = false;

    double optical_x_acc_ = 0.0;
    double optical_y_acc_ = 0.0;
    uint32_t optical_valid_count_ = 0;
    uint32_t optical_outlier_count_ = 0;
    bool optical_used_ = false;
    bool optical_rejected_ = false;

    double encoder_x_acc_ = 0.0;
    double encoder_y_acc_ = 0.0;
};

}  // namespace pose

#endif  // COMPLEMENTARY_ESTIMATOR_H
