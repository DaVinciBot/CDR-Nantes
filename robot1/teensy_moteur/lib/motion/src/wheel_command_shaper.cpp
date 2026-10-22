/*
 * Wheel command shaping. See include/wheel_command_shaper.h for the order of the
 * steps and why.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "wheel_command_shaper.h"

#include <math.h>

namespace motion {

namespace {

constexpr double PI_D = 3.14159265358979323846;

double max_abs3(const double v[3]) {
    return fmax(fmax(fabs(v[0]), fabs(v[1])), fabs(v[2]));
}

}  // namespace

double braking_speed(double distance, double accel) {
    if (distance <= 0.0 || accel <= 0.0) return 0.0;
    return sqrt(2.0 * accel * distance);
}

double braking_speed(double distance, double accel, double dt_s) {
    if (distance <= 0.0 || accel <= 0.0) return 0.0;
    if (!(dt_s > 0.0)) return braking_speed(distance, accel);
    const double a_dt = accel * dt_s;
    return sqrt(a_dt * a_dt + 2.0 * accel * distance) - a_dt;
}

WheelCommandShaper::WheelCommandShaper(const MotionParams &params)
    : params_(params) {
    geometry_.robot_radius_mm = static_cast<float>(params.robot_radius_mm);
    geometry_.counts_to_mm = 0.0f;  // not used by the inverse kinematics
    mm_s_to_rpm_ = 60.0 / (PI_D * params.wheel_diameter_mm);
    reset();
}

void WheelCommandShaper::reset() {
    for (int i = 0; i < 3; ++i) {
        command_.wheel_rpm[i] = 0.0;
        command_.requested_rpm[i] = 0.0;
    }
    command_.braking = false;
    command_.speed_saturated = false;
    command_.accel_limited = false;
}

const ShapedCommand &WheelCommandShaper::step(const VelocityCommand &position_loop,
                                              const VelocityCommand &feed_forward,
                                              double heading_rad, double distance_mm,
                                              double angle_rad, double dt_s) {
    if (!(dt_s > 0.0)) return command_;

    // 1. Braking cap on the position-loop part, then the feed-forward.
    double vx = position_loop.vx_mm_s;
    double vy = position_loop.vy_mm_s;
    double omega = position_loop.omega_rad_s;
    bool braking = false;

    const double v_cap = braking_speed(distance_mm, params_.max_accel_mm_s2, dt_s);
    const double v_norm = sqrt(vx * vx + vy * vy);
    if (v_norm > v_cap) {
        const double scale = (v_norm > 0.0) ? v_cap / v_norm : 0.0;
        vx *= scale;
        vy *= scale;
        braking = true;
    }
    const double omega_cap = braking_speed(fabs(angle_rad), params_.max_alpha_rad_s2, dt_s);
    if (fabs(omega) > omega_cap) {
        omega = (omega > 0.0) ? omega_cap : -omega_cap;
        braking = true;
    }

    vx += feed_forward.vx_mm_s;
    vy += feed_forward.vy_mm_s;
    omega += feed_forward.omega_rad_s;

    // 2. Table frame to robot frame.
    const double c = cos(heading_rad);
    const double s = sin(heading_rad);
    kinematics::BodyDelta body;
    body.dx = static_cast<float>(c * vx + s * vy);
    body.dy = static_cast<float>(-s * vx + c * vy);
    body.dtheta = static_cast<float>(omega);

    // 3. Inverse kinematics, wheel surface speed to rpm.
    float surface_mm_s[3];
    kinematics::inverse_kinematics(geometry_, body, surface_mm_s);
    double requested[3];
    for (int i = 0; i < 3; ++i) {
        requested[i] = static_cast<double>(surface_mm_s[i]) * mm_s_to_rpm_;
    }

    // 4. Proportional normalisation.
    bool speed_saturated = false;
    const double fastest = max_abs3(requested);
    if (fastest > params_.max_wheel_rpm) {
        const double scale = params_.max_wheel_rpm / fastest;
        for (int i = 0; i < 3; ++i) requested[i] *= scale;
        speed_saturated = true;
    }

    // 5. Synchronised acceleration limit.
    double change[3];
    for (int i = 0; i < 3; ++i) change[i] = requested[i] - command_.wheel_rpm[i];
    bool accel_limited = false;
    const double max_change = params_.max_wheel_accel_rpm_s * dt_s;
    const double largest = max_abs3(change);
    if (largest > max_change) {
        const double scale = max_change / largest;
        for (int i = 0; i < 3; ++i) change[i] *= scale;
        accel_limited = true;
    }

    for (int i = 0; i < 3; ++i) {
        command_.requested_rpm[i] = requested[i];
        command_.wheel_rpm[i] += change[i];
    }
    command_.braking = braking;
    command_.speed_saturated = speed_saturated;
    command_.accel_limited = accel_limited;
    return command_;
}

}  // namespace motion
