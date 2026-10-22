/*
 * Covariance of a forward-kinematics displacement. See include/ekf_kinematics.h.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "ekf_kinematics.h"

namespace kalman {

namespace {

/**
 * Forward kinematics matrix J, without the 1/R on the rotation row:
 * (dx, dy, R dtheta) = J_UNIT (w1, w2, w3). Same equations as
 * kinematics::forward_kinematics.
 */
constexpr float J_UNIT[3][3] = {
    {-1.0f / 3.0f, -1.0f / 3.0f, 2.0f / 3.0f},
    {1.0f / SQRT3_F, -1.0f / SQRT3_F, 0.0f},
    {-1.0f / 3.0f, -1.0f / 3.0f, -1.0f / 3.0f},
};

}  // namespace

Mat<3> body_delta_covariance(const Geometry &geometry,
                             const float wheel_variance_mm2[3]) {
    // Row scale: 1 for dx and dy, 1/R for dtheta.
    const float row_scale[3] = {1.0f, 1.0f, 1.0f / geometry.robot_radius_mm};

    Mat<3> sigma;
    for (size_t a = 0; a < 3; ++a) {
        for (size_t b = a; b < 3; ++b) {
            float sum = 0.0f;
            for (size_t k = 0; k < 3; ++k) {
                sum += J_UNIT[a][k] * J_UNIT[b][k] * wheel_variance_mm2[k];
            }
            sum *= row_scale[a] * row_scale[b];
            sigma(a, b) = sum;
            sigma(b, a) = sum;
        }
    }
    return sigma;
}

}  // namespace kalman
