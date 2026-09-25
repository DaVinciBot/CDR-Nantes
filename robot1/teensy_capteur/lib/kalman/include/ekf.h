/*
 * Extended Kalman filter for the holonomic base. PLACEHOLDER — filled by the
 * 3-state then the 6-state filter.
 *
 * Planned sequence:
 *   first : 3 states [x, y, theta], wheel odometry as the control input u.
 *           Assumed fallback if the 6-state version is not tuned in time, and
 *           a trivial model on which to debug the whole plumbing.
 *   then  : 6 states [x, y, theta, vx, vy, omega], scalar measurement updates,
 *           NIS gate.
 *   later : possible 7th state b_omega (gyro bias), not before.
 *
 * Units: mm and rad, in float. Angles normalized to [-pi, pi).
 *
 * THREE TRAPS THIS FILTER MUST AVOID, all of them already sprung once:
 *
 * 1. Odometry is the INPUT u, not a measurement. The 2026 prototype fed the
 *    Teensy pose in as z[0..2], which makes the filter correct itself with its
 *    own prediction.
 *
 * 2. Do not double-count correlated measurements. The prototype also fed
 *    z[3..4] = sqrt(px^2+py^2), atan2(py, px), calling it a LiDAR reading: the
 *    same x and y restated in polar form, with an independent R. The result is
 *    guaranteed overconfidence. The live version of this trap is the OTOS, which
 *    fuses its own IMU internally: the retained approach is to take its
 *    (dx, dy) as robot-frame displacement and IGNORE its theta, so the
 *    three sources (OTOS translation, BNO085 heading, encoders) stay
 *    decorrelated.
 *
 * 3. The P_MAX guard must NOT cover x and y. With the LiDAR out of the filter
 *    (decided 05/09/2026) there is no absolute position measurement at all, so X
 *    and Y are unobservable and their variance grows for the whole match. That is
 *    correct behaviour; a guard on it would trip every single match.
 *
 * No sigma is published on the link yet: it only means something once Q is
 * tuned, which is planned for January. An unvalidated sigma is worse than
 * none, because it gets trusted.
 *
 * This header must stay free of any Arduino include, so lib/kalman builds in the
 * "native" environment.
 */

#ifndef EKF_H
#define EKF_H

#include "matrix.h"

namespace kalman {

// Filter goes here.

}  // namespace kalman

#endif  // EKF_H
