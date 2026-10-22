/*
 * Kinematics of the 3-wheel holonomic base. See include/kinematics.h for the
 * convention.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "kinematics.h"

namespace kinematics {

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

}  // namespace kinematics
