/*
 * Kinematics of the 3-wheel holonomic base: the ONLY place where the wheel
 * equations are written. The motor commands (lib/motion), the EKF (lib/kalman)
 * and the complementary filter (lib/pose_estimator) all go through it, so that
 * the wheels are driven and read back with equations that are exact inverses of
 * each other. A change to the equations is made here, once.
 *
 *   inverse (commands), wheel axes at 120, 240 and 0 deg:
 *     w1 = -0.5 vx + (sqrt(3)/2) vy - R omega
 *     w2 = -0.5 vx - (sqrt(3)/2) vy - R omega
 *     w3 =      vx                  - R omega
 *
 *   forward (odometry), its exact inverse:
 *     vx    = (2 w3 - w1 - w2) / 3
 *     vy    = (w1 - w2) / sqrt(3)
 *     omega = -(w1 + w2 + w3) / (3 R)
 *
 * Note the minus sign on R omega: a positive omega turns every wheel backwards.
 * Both maps are linear, so they apply to displacements over a tick (mm, rad)
 * and to speeds (mm/s, rad/s) alike.
 *
 * Frame: the robot frame, x forward. Robot to world is the standard rotation by
 * theta. Units: mm, rad.
 *
 * The geometry is passed in, never read from config.h: this lib stays free of
 * board configuration and of Arduino, so it builds and runs in the native test
 * environment.
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

namespace kinematics {

constexpr float SQRT3_F = 1.73205080756888f;

/** @brief The two constants the wheel equations need from the mechanics. */
struct Geometry {
    /** Centre of the robot to the wheel contact, mm. */
    float robot_radius_mm;
    /** Wheel travel per encoder count, mm. */
    float counts_to_mm;
};

/** @brief A displacement (or a speed) of the robot, in its own frame. */
struct BodyDelta {
    float dx;      // mm, robot x (forward)
    float dy;      // mm, robot y
    float dtheta;  // rad
};

/**
 * @brief Wheel count deltas to robot-frame displacement.
 *
 * @param geometry Robot geometry.
 * @param d1 Raw count delta of wheel 1 (axis at 120 deg).
 * @param d2 Raw count delta of wheel 2 (axis at 240 deg).
 * @param d3 Raw count delta of wheel 3 (axis at 0 deg).
 * @return The displacement over the tick.
 */
BodyDelta forward_kinematics(const Geometry &geometry, int32_t d1, int32_t d2,
                             int32_t d3);

/**
 * @brief Robot-frame displacement to wheel travel, in mm (or robot-frame speed
 * to wheel surface speed, in mm/s: the map is linear).
 *
 * @param geometry Robot geometry (counts_to_mm is not used).
 * @param delta The displacement.
 * @param travel_mm Out: travel of wheels 1, 2 and 3, mm.
 */
void inverse_kinematics(const Geometry &geometry, const BodyDelta &delta,
                        float travel_mm[3]);

}  // namespace kinematics

#endif  // KINEMATICS_H
