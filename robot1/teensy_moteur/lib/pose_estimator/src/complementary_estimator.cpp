/*
 * Complementary filter. See include/complementary_estimator.h: the behaviour is
 * the 2026 one of holonomic_basis, step for step.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "complementary_estimator.h"

#include <math.h>

namespace pose {

ComplementaryEstimator::ComplementaryEstimator(const ComplementaryParams &params)
    : params_(params) {}

void ComplementaryEstimator::reset(float x_mm, float y_mm, float theta_rad) {
    x_ = x_mm;
    y_ = y_mm;
    theta_ = normalize_angle(theta_rad);
    last_theta_ = theta_;
    // The next optical sample is the new reference: the sensor's tracking is
    // reset along with the pose.
    optical_seen_ = false;
    optical_active_ = false;
    optical_rejected_ = false;
}

void ComplementaryEstimator::integrate_optical(double x_mm, double y_mm) {
    const double dx_mm = x_mm - last_optical_x_;
    const double dy_mm = y_mm - last_optical_y_;
    last_optical_x_ = x_mm;
    last_optical_y_ = y_mm;

    // The first sample only sets the reference.
    if (!optical_seen_) {
        optical_seen_ = true;
        return;
    }

    // Already in the table frame (OpticalWorldIntegrator): no rotation here.
    const double magnitude = sqrt(dx_mm * dx_mm + dy_mm * dy_mm);
    if (magnitude > params_.optical_outlier_mm) {
        ++optical_outlier_count_;
        optical_rejected_ = true;
    } else {
        const double dx_world = (magnitude >= params_.optical_noise_mm) ? dx_mm : 0.0;
        const double dy_world = (magnitude >= params_.optical_noise_mm) ? dy_mm : 0.0;
        optical_x_acc_ += dx_world;
        optical_y_acc_ += dy_world;
        if (dx_world != 0.0 || dy_world != 0.0) {
            ++optical_valid_count_;
        }
    }
}

void ComplementaryEstimator::step(const EstimatorInput &in) {
    optical_rejected_ = false;
    optical_active_ = false;

    // The first tick only sets the encoder reference, which the caller holds.
    if (in.first_frame) {
        return;
    }
    ready_ = true;

    double d1 = static_cast<double>(in.d1);
    double d2 = static_cast<double>(in.d2);
    double d3 = static_cast<double>(in.d3);
    if (fabs(d1) < params_.encoder_noise_counts) d1 = 0.0;
    if (fabs(d2) < params_.encoder_noise_counts) d2 = 0.0;
    if (fabs(d3) < params_.encoder_noise_counts) d3 = 0.0;

    const double ctm = params_.counts_to_mm;
    const double radius = params_.robot_radius_mm;

    // --- Optical, before the encoder check, so that it is logged at rest too ---
    double dx_optical_world = 0.0;
    double dy_optical_world = 0.0;
    if (in.optical_pose_valid) {
        const double prev_acc_x = optical_x_acc_;
        const double prev_acc_y = optical_y_acc_;

        // Provisional rotation, 0 while the encoders are still.
        double omega_temp = 0.0;
        if (d1 != 0.0 || d2 != 0.0 || d3 != 0.0) {
            omega_temp = (d1 * ctm + d2 * ctm + d3 * ctm) / (3.0 * radius);
        }

        integrate_optical(in.optical_x_mm, in.optical_y_mm);

        const double diff_opt_x = optical_x_acc_ - prev_acc_x;
        const double diff_opt_y = optical_y_acc_ - prev_acc_y;

        // Pure rotation: the three wheels turn the same way at similar speeds.
        bool is_pure_rotation = false;
        if (fabs(omega_temp) > params_.pure_rotation_min_rad) {
            const double d1_abs = fabs(d1);
            const double d2_abs = fabs(d2);
            const double d3_abs = fabs(d3);
            const bool same_sign = (d1 * d2 > 0) && (d2 * d3 > 0);
            const double d_avg = (d1_abs + d2_abs + d3_abs) / 3.0;
            if (d_avg > params_.pure_rotation_min_counts) {
                const double d1_diff = fabs(d1_abs - d_avg) / d_avg;
                const double d2_diff = fabs(d2_abs - d_avg) / d_avg;
                const double d3_diff = fabs(d3_abs - d_avg) / d_avg;
                if (same_sign && d1_diff < params_.pure_rotation_tolerance &&
                    d2_diff < params_.pure_rotation_tolerance &&
                    d3_diff < params_.pure_rotation_tolerance) {
                    is_pure_rotation = true;
                }
            }
        }

        // Pure rotation: the optical X/Y is parasitic motion, drop it.
        if (!is_pure_rotation &&
            (fabs(diff_opt_x) > params_.optical_motion_mm ||
             fabs(diff_opt_y) > params_.optical_motion_mm)) {
            dx_optical_world = diff_opt_x;
            dy_optical_world = diff_opt_y;
            optical_active_ = true;
        }
    }

    // --- Encoders: forward kinematics, wheels at 120, 240 and 0 deg ---
    const double w1_mm = d1 * ctm;
    const double w2_mm = d2 * ctm;
    const double w3_mm = d3 * ctm;
    const double dx_enc = (-w1_mm - w2_mm + 2.0 * w3_mm) / 3.0;
    const double dy_enc = (w1_mm - w2_mm) / sqrt(3.0);
    const double omega_enc = -(w1_mm + w2_mm + w3_mm) / (3.0 * radius);

    // To the world frame, at the heading of the previous tick.
    const double dx_enc_world = dx_enc * cos(theta_) - dy_enc * sin(theta_);
    const double dy_enc_world = dx_enc * sin(theta_) + dy_enc * cos(theta_);
    encoder_x_acc_ += dx_enc_world;
    encoder_y_acc_ += dy_enc_world;

    // --- Fusion ---
    double dx_final_world = dx_enc_world;
    double dy_final_world = dy_enc_world;
    if (optical_active_ && optical_valid_count_ > 0) {
        // A stale encoder buffer gets no share: 100 % optical.
        const bool encoder_fresh = in.encoder_age_us < params_.encoder_max_age_us;
        const double alpha = encoder_fresh ? params_.alpha_encoders : 0.0;
        dx_final_world = alpha * dx_enc_world + (1.0 - alpha) * dx_optical_world;
        dy_final_world = alpha * dy_enc_world + (1.0 - alpha) * dy_optical_world;
    }

    // --- Heading: IMU when available, encoder rotation otherwise ---
    if (in.imu_valid) {
        theta_ = normalize_angle(in.heading_rad);
    } else {
        theta_ = normalize_angle(theta_ + omega_enc);
    }

    // --- Position ---
    const double theta_mean = (last_theta_ + theta_) / 2.0;
    if (optical_active_) {
        x_ += dx_final_world;
        y_ += dy_final_world;
    } else if (params_.use_encoders) {
        const double c = cos(theta_mean);
        const double s = sin(theta_mean);
        x_ += dx_enc * c - dy_enc * s;
        y_ += dx_enc * s + dy_enc * c;
    }
    last_theta_ = theta_;
}

PoseEstimate ComplementaryEstimator::pose() const {
    PoseEstimate p;
    p.x_mm = static_cast<float>(x_);
    p.y_mm = static_cast<float>(y_);
    p.theta_rad = static_cast<float>(theta_);
    return p;
}

uint8_t ComplementaryEstimator::health_flags() const {
    uint8_t flags = 0;
    if (ready_) flags |= FILTER_READY;
    if (optical_active_) flags |= OPTICAL_OK;
    if (optical_rejected_) flags |= MEAS_REJECTED;
    return flags;
}

}  // namespace pose
