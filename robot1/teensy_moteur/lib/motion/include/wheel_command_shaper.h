/*
 * From a robot velocity command to the three wheel speeds sent to the drivers.
 *
 * Once per control tick, in this order:
 *
 *   1. braking: the position-loop part of the command is capped at the speed
 *      from which the robot can still stop on the remaining distance, in
 *      translation and in rotation (sqrt(2 a d) in its per-tick form, see
 *      braking_speed). The velocity feed-forward is added after the cap: a
 *      moving target brakes through its own profile, not through the distance
 *      to it;
 *   2. table frame to robot frame, by the heading;
 *   3. inverse kinematics (lib/kinematics), mm/s to rpm;
 *   4. proportional normalisation: if one wheel is above max_wheel_rpm, the
 *      three are scaled down by the same factor, so the direction holds;
 *   5. synchronised acceleration limit: if one wheel would change by more than
 *      max_wheel_accel_rpm_s * dt since the previous tick, the three changes are
 *      scaled down by the same factor. The kinematics being linear, this scales
 *      the change of the robot velocity without turning it: a wheel with half
 *      the target speed no longer reaches it twice as early, and the robot keeps
 *      its heading during a ramp. The limit is per wheel (it protects each
 *      motor's torque): the robot itself accelerates at 1 to 1.15 times the
 *      wheel-level rate, depending on the direction.
 *
 * The limiter only sees the current tick: it cannot anticipate a stop. That is
 * what the braking cap is for.
 *
 * Units: mm, rad, s, rpm. Arduino-free: builds and is tested in the native
 * environment.
 */

#ifndef WHEEL_COMMAND_SHAPER_H
#define WHEEL_COMMAND_SHAPER_H

#include "kinematics.h"

namespace motion {

/** @brief Geometry and limits of the base. No defaults: board configuration. */
struct MotionParams {
    /** Centre of the robot to the wheel contact, mm. */
    double robot_radius_mm;
    /** Wheel diameter, mm. */
    double wheel_diameter_mm;
    /** Wheel speed ceiling, rpm. */
    double max_wheel_rpm;
    /** Wheel acceleration limit, rpm/s. */
    double max_wheel_accel_rpm_s;
    /** Deceleration assumed by the braking cap, translation, mm/s^2. */
    double max_accel_mm_s2;
    /** Deceleration assumed by the braking cap, rotation, rad/s^2. */
    double max_alpha_rad_s2;
};

/** @brief A robot velocity in the table frame. */
struct VelocityCommand {
    double vx_mm_s;
    double vy_mm_s;
    double omega_rad_s;
};

/** @brief What one tick produced, for the drivers and for the trace. */
struct ShapedCommand {
    /** Speeds to send to wheels 1, 2 and 3, rpm. */
    double wheel_rpm[3];
    /** The same before the acceleration limit, rpm. */
    double requested_rpm[3];
    /** The braking cap reduced the position-loop command. */
    bool braking;
    /** The proportional normalisation scaled the wheels down. */
    bool speed_saturated;
    /** The acceleration limit scaled the change of the wheels down. */
    bool accel_limited;
};

/**
 * @return The largest speed from which a stop within `distance` is possible at
 *     deceleration `accel`: sqrt(2 accel distance). Zero for a non-positive
 *     distance or deceleration.
 */
double braking_speed(double distance, double accel);

/**
 * @return The same for a command held over a tick of dt_s: the speed v such
 *     that, after travelling v dt_s during the tick, the robot can still stop on
 *     what is left, v dt + v^2 / (2 accel) = distance. The continuous
 *     sqrt(2 a d), applied tick by tick, ends a few mm past the target.
 */
double braking_speed(double distance, double accel, double dt_s);

class WheelCommandShaper {
public:
    explicit WheelCommandShaper(const MotionParams &params);

    /** @brief The wheels are known to be at rest (boot, after a stop). */
    void reset();

    /**
     * @brief One control tick.
     *
     * @param position_loop Output of the position PIDs, table frame.
     * @param feed_forward Velocity of the trajectory, table frame; zero for a
     *     plain pose target.
     * @param heading_rad Current heading of the robot.
     * @param distance_mm Remaining distance to the target position.
     * @param angle_rad Remaining heading error (sign ignored).
     * @param dt_s Tick duration. A non-positive dt changes nothing.
     * @return The command of this tick (also kept, see last()).
     */
    const ShapedCommand &step(const VelocityCommand &position_loop,
                              const VelocityCommand &feed_forward,
                              double heading_rad, double distance_mm,
                              double angle_rad, double dt_s);

    /** @return The command of the last tick (all zero after reset()). */
    const ShapedCommand &last() const { return command_; }

private:
    MotionParams params_;
    kinematics::Geometry geometry_;
    /** Wheel surface speed (mm/s) to wheel speed (rpm). */
    double mm_s_to_rpm_;
    ShapedCommand command_;
};

}  // namespace motion

#endif  // WHEEL_COMMAND_SHAPER_H
