/*
 * Kinematics of the 3-wheel holonomic base, as seen by the filter.
 *
 * SAME CONVENTION AS THE MOTOR BOARD, and it has to stay that way: the counts
 * arriving in the EncoderFrame are produced by the Teensy Moteur, and the only
 * way to read them back correctly is with the exact inverse of the kinematics
 * that commanded them. The reference is
 * robot1/teensy_moteur/lib/holonomic_basis/src/holonomic_basis.cpp:
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
 * The motor code writes 0.866 for sqrt(3)/2; the exact value is used here so
 * that forward and inverse are exact inverses of each other. The difference,
 * 3e-5 relative, is far below any wheel noise.
 *
 * Frame: the motor board's robot frame, x forward. Robot to world is the
 * standard rotation by theta, as on the motor board. Units: mm, rad.
 *
 * The geometry is passed in, never read from config.h: lib/kalman stays free of
 * board configuration so it builds and runs in the native test environment.
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

#include "matrix.h"

namespace kalman {

constexpr float SQRT3_F = 1.73205080756888f;

/** @brief The two constants the filter needs from the mechanics. */
struct Geometry {
    /** Centre of the robot to the wheel contact, mm. */
    float robot_radius_mm;
    /** Wheel travel per encoder count, mm. */
    float counts_to_mm;
};

/** @brief A displacement of the robot, expressed in its own frame. */
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
 * @brief Robot-frame displacement to wheel travel, in mm.
 *
 * Used to build the wheel measurement rows of the 6-state filter, and by the
 * tests to generate encoder counts from a known motion.
 *
 * @param geometry Robot geometry.
 * @param delta The displacement.
 * @param travel_mm Out: travel of wheels 1, 2 and 3, mm.
 */
void inverse_kinematics(const Geometry &geometry, const BodyDelta &delta,
                        float travel_mm[3]);

/**
 * @brief Covariance of a forward-kinematics displacement, from per-wheel noise.
 *
 * Sigma_body = J diag(wheel_variance) J^T, with J the (constant) forward
 * kinematics matrix. The three wheels are independent, so this is where their
 * noise becomes correlated in the robot frame: a noisy wheel 3 moves dx and
 * dtheta together, never dy.
 *
 * @param geometry Robot geometry.
 * @param wheel_variance_mm2 Variance of each wheel's travel, mm^2.
 * @return The 3 x 3 covariance of (dx, dy, dtheta).
 */
Mat<3> body_delta_covariance(const Geometry &geometry,
                             const float wheel_variance_mm2[3]);

}  // namespace kalman

#endif  // KINEMATICS_H
