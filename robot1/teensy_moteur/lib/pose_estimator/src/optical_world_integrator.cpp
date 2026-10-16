/*
 * Table-frame optical pose from robot-frame steps. See
 * include/optical_world_integrator.h.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "optical_world_integrator.h"

#include <math.h>

namespace pose {

void OpticalWorldIntegrator::reset(double x_mm, double y_mm) {
    x_mm_ = x_mm;
    y_mm_ = y_mm;
}

void OpticalWorldIntegrator::add(double dx_robot_mm, double dy_robot_mm, double theta_rad) {
    const double c = cos(theta_rad);
    const double s = sin(theta_rad);
    x_mm_ += c * dx_robot_mm - s * dy_robot_mm;
    y_mm_ += s * dx_robot_mm + c * dy_robot_mm;
}

}  // namespace pose
