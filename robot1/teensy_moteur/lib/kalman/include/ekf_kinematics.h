/*
 * The base kinematics as seen by the filters.
 *
 * The wheel equations live in lib/kinematics, shared with the motor commands:
 * the encoder counts are produced by wheels driven through the inverse
 * kinematics, and the only way to read them back correctly is with its exact
 * inverse. This header brings them into the kalman namespace and adds what only
 * a filter needs: the covariance of a displacement computed from noisy wheels.
 */

#ifndef EKF_KINEMATICS_H
#define EKF_KINEMATICS_H

#include "kinematics.h"
#include "matrix.h"

namespace kalman {

using kinematics::BodyDelta;
using kinematics::Geometry;
using kinematics::SQRT3_F;
using kinematics::forward_kinematics;
using kinematics::inverse_kinematics;

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

#endif  // EKF_KINEMATICS_H
