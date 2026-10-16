/*
 * The 6-state EKF behind the PoseEstimator contract. See
 * include/ekf_estimator.h.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "ekf_estimator.h"

namespace pose {

EkfEstimator::EkfEstimator(const kalman::Geometry &geometry,
                           const kalman::Ekf6Params &params,
                           const EkfInitialSigmas &sigmas)
    : filter_(geometry, params), sigmas_(sigmas) {
    reset(0.0f, 0.0f, 0.0f);
}

void EkfEstimator::reset(float x_mm, float y_mm, float theta_rad) {
    filter_.reset(x_mm, y_mm, theta_rad, sigmas_.xy_mm, sigmas_.theta_rad,
                  sigmas_.v_mm_s, sigmas_.omega_rad_s);
    rejected_ = false;
    optical_fed_ = false;
}

void EkfEstimator::step(const EstimatorInput &in) {
    // process_tick() does not report its three wheel updates one by one: the
    // rejection counter tells whether one of them was gated.
    const uint32_t wheel_rejected_before = filter_.wheel_stats().rejected;
    rejected_ = false;
    optical_fed_ = false;

    kalman::OdometryTick tick;
    tick.d1 = in.d1;
    tick.d2 = in.d2;
    tick.d3 = in.d3;
    tick.dt_us = in.dt_us;
    tick.first_frame = in.first_frame;
    tick.encoders_valid = in.encoders_fresh;
    filter_.process_tick(tick);
    if (filter_.wheel_stats().rejected != wheel_rejected_before) {
        rejected_ = true;
    }

    if (in.imu_valid) {
        const uint32_t stale_before = filter_.stale_samples();
        const kalman::UpdateResult r =
            filter_.update_heading(in.heading_rad, in.imu_timestamp_us);
        // A stale sample is a buffer not refreshed yet, not a rejection.
        if (!r.accepted && filter_.stale_samples() == stale_before) {
            rejected_ = true;
        }
    }

    if (in.optical_velocity_valid) {
        optical_fed_ = true;
        const kalman::OpticalUpdate r = filter_.update_optical(
            in.optical_vx_mm_s, in.optical_vy_mm_s, in.optical_timestamp_us);
        if (!r.stale && (!r.x.accepted || !r.y.accepted)) {
            rejected_ = true;
        }
    }
}

PoseEstimate EkfEstimator::pose() const {
    const kalman::Vec<kalman::Ekf6::N> &x = filter_.state();
    PoseEstimate p;
    p.x_mm = x[kalman::Ekf6::X];
    p.y_mm = x[kalman::Ekf6::Y];
    p.theta_rad = x[kalman::Ekf6::THETA];
    return p;
}

uint8_t EkfEstimator::health_flags() const {
    uint8_t flags = 0;
    if (filter_.covariance_healthy()) flags |= FILTER_READY;
    if (optical_fed_ && filter_.optical_ok()) flags |= OPTICAL_OK;
    if (rejected_) flags |= MEAS_REJECTED;
    return flags;
}

}  // namespace pose
