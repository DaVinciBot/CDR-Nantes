/*
 * Shared helpers of the pose estimators.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "pose_estimator.h"

#include <math.h>

namespace pose {

namespace {

// Not M_PI: -std=c++17 is strict ANSI, and MinGW hides M_PI in that mode.
constexpr double PI_D = 3.14159265358979323846;

}  // namespace

EstimatorInput empty_input() {
    EstimatorInput in;
    in.d1 = 0;
    in.d2 = 0;
    in.d3 = 0;
    in.dt_us = 0;
    in.first_frame = false;
    in.encoders_fresh = false;
    in.encoder_age_us = 0;
    in.imu_valid = false;
    in.heading_rad = 0.0f;
    in.imu_timestamp_us = 0;
    in.optical_timestamp_us = 0;
    in.optical_step_valid = false;
    in.optical_dx_mm = 0.0f;
    in.optical_dy_mm = 0.0f;
    in.optical_step_dt_us = 0;
    in.optical_velocity_valid = false;
    in.optical_vx_mm_s = 0.0f;
    in.optical_vy_mm_s = 0.0f;
    return in;
}

double normalize_angle(double theta) {
    // Shift by +pi, take the modulo 2 pi, remap to [0, 2 pi), shift back.
    theta = fmod(theta + PI_D, 2.0 * PI_D);
    if (theta < 0.0) {
        theta += 2.0 * PI_D;
    }
    return theta - PI_D;
}

}  // namespace pose
