/*
 * Complementary filter. See include/complementary_estimator.h.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "complementary_estimator.h"

#include <math.h>

namespace pose {

ComplementaryEstimator::ComplementaryEstimator(const ComplementaryParams &params)
    : params_(params) {
    geometry_.robot_radius_mm = static_cast<float>(params.robot_radius_mm);
    geometry_.counts_to_mm = static_cast<float>(params.counts_to_mm);
}

void ComplementaryEstimator::reset(float x_mm, float y_mm, float theta_rad) {
    x_ = x_mm;
    y_ = y_mm;
    theta_ = normalize_angle(theta_rad);
    vx_ = 0.0;
    vy_ = 0.0;
    omega_ = 0.0;
    optical_used_ = false;
    optical_rejected_ = false;
}

bool ComplementaryEstimator::is_pure_rotation(double d1, double d2, double d3) const {
    const double rotation =
        (d1 + d2 + d3) * params_.counts_to_mm / (3.0 * params_.robot_radius_mm);
    if (fabs(rotation) <= params_.pure_rotation_min_rad) return false;

    const bool same_sign = (d1 * d2 > 0) && (d2 * d3 > 0);
    const double d_avg = (fabs(d1) + fabs(d2) + fabs(d3)) / 3.0;
    if (!same_sign || d_avg <= params_.pure_rotation_min_counts) return false;

    const double tol = params_.pure_rotation_tolerance;
    return fabs(fabs(d1) - d_avg) / d_avg < tol && fabs(fabs(d2) - d_avg) / d_avg < tol &&
           fabs(fabs(d3) - d_avg) / d_avg < tol;
}

void ComplementaryEstimator::step(const EstimatorInput &in) {
    optical_used_ = false;
    optical_rejected_ = false;

    // The first tick only sets the encoder reference, which the caller holds.
    if (in.first_frame) {
        return;
    }
    ready_ = true;
    const double dt_s = static_cast<double>(in.dt_us) * 1e-6;

    // --- Encoders: noise floor, then forward kinematics (robot frame) ---
    int32_t d[3] = {in.d1, in.d2, in.d3};
    for (int i = 0; i < 3; ++i) {
        if (fabs(static_cast<double>(d[i])) < params_.encoder_noise_counts) d[i] = 0;
    }
    const kinematics::BodyDelta enc = kinematics::forward_kinematics(geometry_, d[0], d[1], d[2]);

    // --- Heading: IMU when available, encoder rotation otherwise ---
    const double theta_new = in.imu_valid ? normalize_angle(in.heading_rad)
                                          : normalize_angle(theta_ + enc.dtheta);
    // Mean heading of the tick, across +-pi: half the wrapped change.
    const double dtheta = normalize_angle(theta_new - theta_);
    const double theta_mean = normalize_angle(theta_ + 0.5 * dtheta);
    const double c = cos(theta_mean);
    const double s = sin(theta_mean);

    // --- Optical: speed of the new sample, displacement over this tick ---
    double optical_dx = 0.0;
    double optical_dy = 0.0;
    if (in.optical_step_valid && in.optical_step_dt_us > 0 && dt_s > 0.0) {
        const double step_s = static_cast<double>(in.optical_step_dt_us) * 1e-6;
        const double vx = in.optical_dx_mm / step_s;
        const double vy = in.optical_dy_mm / step_s;
        const double speed = sqrt(vx * vx + vy * vy);
        if (speed > params_.optical_outlier_mm_s) {
            ++optical_outlier_count_;
            optical_rejected_ = true;
        } else if (speed >= params_.optical_noise_mm_s &&
                   !is_pure_rotation(d[0], d[1], d[2])) {
            optical_dx = vx * dt_s;
            optical_dy = vy * dt_s;
            optical_used_ = true;
            ++optical_valid_count_;
            optical_x_acc_ += optical_dx * c - optical_dy * s;
            optical_y_acc_ += optical_dx * s + optical_dy * c;
        }
    }

    encoder_x_acc_ += enc.dx * c - enc.dy * s;
    encoder_y_acc_ += enc.dx * s + enc.dy * c;

    // --- Fusion, robot frame ---
    bool moves = false;
    double bx = 0.0;
    double by = 0.0;
    if (optical_used_) {
        // Encoder deltas that do not span exactly this tick get no share.
        const double alpha = in.encoders_fresh ? params_.alpha_encoders : 0.0;
        bx = alpha * enc.dx + (1.0 - alpha) * optical_dx;
        by = alpha * enc.dy + (1.0 - alpha) * optical_dy;
        moves = true;
    } else if (params_.use_encoders) {
        bx = enc.dx;
        by = enc.dy;
        moves = true;
    }

    // --- Pose and velocity, table frame ---
    const double dx_table = moves ? bx * c - by * s : 0.0;
    const double dy_table = moves ? bx * s + by * c : 0.0;
    x_ += dx_table;
    y_ += dy_table;
    theta_ = theta_new;
    if (dt_s > 0.0) {
        vx_ = dx_table / dt_s;
        vy_ = dy_table / dt_s;
        omega_ = dtheta / dt_s;
    }
}

PoseEstimate ComplementaryEstimator::pose() const {
    PoseEstimate p;
    p.x_mm = static_cast<float>(x_);
    p.y_mm = static_cast<float>(y_);
    p.theta_rad = static_cast<float>(theta_);
    p.vx_mm_s = static_cast<float>(vx_);
    p.vy_mm_s = static_cast<float>(vy_);
    p.omega_rad_s = static_cast<float>(omega_);
    return p;
}

uint8_t ComplementaryEstimator::health_flags() const {
    uint8_t flags = 0;
    if (ready_) flags |= FILTER_READY;
    if (optical_used_) flags |= OPTICAL_OK;
    if (optical_rejected_) flags |= MEAS_REJECTED;
    return flags;
}

}  // namespace pose
