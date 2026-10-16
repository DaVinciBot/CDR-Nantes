/*
 * The 6-state EKF of lib/kalman behind the PoseEstimator contract.
 *
 * Per tick, in this order:
 *   1. prediction and the three wheel-speed updates (Ekf6::process_tick); the
 *      wheel updates are skipped when the encoders are not fresh, the velocity
 *      model carrying the pose through a missed read;
 *   2. the IMU heading, when valid (the filter drops a sample whose timestamp it
 *      has already seen, so a buffer that was not refreshed is harmless);
 *   3. the optical velocity, when valid. The optical POSE of the input is never
 *      used: an integrated pose is not a measurement.
 *
 * The parameters are lib/kalman's provisional ones until R and Q are measured.
 *
 * Arduino-free: builds and is tested in the native environment.
 */

#ifndef EKF_ESTIMATOR_H
#define EKF_ESTIMATOR_H

#include <stdint.h>

#include "ekf6.h"
#include "pose_estimator.h"

namespace pose {

/** @brief Uncertainty assumed at reset(): robot at rest at a known pose. */
struct EkfInitialSigmas {
    float xy_mm = 10.0f;
    float theta_rad = 0.02f;
    float v_mm_s = 1.0f;
    float omega_rad_s = 0.01f;
};

class EkfEstimator : public PoseEstimator {
public:
    EkfEstimator(const kalman::Geometry &geometry, const kalman::Ekf6Params &params,
                 const EkfInitialSigmas &sigmas = EkfInitialSigmas());

    void reset(float x_mm, float y_mm, float theta_rad) override;
    void step(const EstimatorInput &input) override;
    PoseEstimate pose() const override;
    uint8_t health_flags() const override;
    const char *name() const override { return "ekf6"; }

    /** @return The filter itself, for its diagnostics (NIS, counters). */
    const kalman::Ekf6 &filter() const { return filter_; }

private:
    kalman::Ekf6 filter_;
    EkfInitialSigmas sigmas_;
    /** A measurement was rejected during the last tick. */
    bool rejected_ = false;
    /** An optical sample was fed during the last tick. Ekf6::optical_ok() is
     *  true when no optical sample ever came, which is not "optical OK". */
    bool optical_fed_ = false;
};

}  // namespace pose

#endif  // EKF_ESTIMATOR_H
