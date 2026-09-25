/*
 * Kinematics of the 3-wheel holonomic base. See include/kinematics.h for the
 * convention, which mirrors the motor board.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "kinematics.h"

namespace kalman {

namespace {

/**
 * Forward kinematics matrix J, without the 1/R on the rotation row:
 * (dx, dy, R dtheta) = J_UNIT (w1, w2, w3).
 */
constexpr float J_UNIT[3][3] = {
    {-1.0f / 3.0f, -1.0f / 3.0f, 2.0f / 3.0f},
    {1.0f / SQRT3_F, -1.0f / SQRT3_F, 0.0f},
    {-1.0f / 3.0f, -1.0f / 3.0f, -1.0f / 3.0f},
};

}  // namespace

BodyDelta forward_kinematics(const Geometry &geometry, int32_t d1, int32_t d2,
                             int32_t d3) {
    const float w1 = static_cast<float>(d1) * geometry.counts_to_mm;
    const float w2 = static_cast<float>(d2) * geometry.counts_to_mm;
    const float w3 = static_cast<float>(d3) * geometry.counts_to_mm;

    BodyDelta delta;
    delta.dx = (2.0f * w3 - w1 - w2) / 3.0f;
    delta.dy = (w1 - w2) / SQRT3_F;
    delta.dtheta = -(w1 + w2 + w3) / (3.0f * geometry.robot_radius_mm);
    return delta;
}

void inverse_kinematics(const Geometry &geometry, const BodyDelta &delta,
                        float travel_mm[3]) {
    const float spin = geometry.robot_radius_mm * delta.dtheta;
    travel_mm[0] = -0.5f * delta.dx + 0.5f * SQRT3_F * delta.dy - spin;
    travel_mm[1] = -0.5f * delta.dx - 0.5f * SQRT3_F * delta.dy - spin;
    travel_mm[2] = delta.dx - spin;
}

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
