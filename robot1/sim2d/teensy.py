"""The control law of the Teensy, from SET_TARGET to the three wheel speeds.

A Python copy of two firmware pieces, kept line for line so the match loop is
tested against the base it will drive in 2027:

    Holonomic_Basis::handle()     robot1/teensy_moteur/lib/holonomic_basis
        target extrapolated at the velocity of SET_TARGET, velocity that
        expires after FEEDFORWARD_TTL_S, dead zone, position loop;
    WheelCommandShaper::step()    robot1/teensy_moteur/lib/motion
        braking cap sqrt(2 a d) in its per-tick form, feed-forward added after
        the cap, inverse kinematics, proportional normalisation, synchronised
        per-wheel acceleration limit.

The position loop is a P (config.h: KP = 1, KI = KD = 0), with its output
bounded in physical units: +/-ROBOT_MAX_SPEED_MM_S in X/Y and
+/-ROBOT_MAX_RAD_S in theta. That is the match firmware as specified, NOT the
test firmware of src/main.cpp, which bounds the PIDs at +/-MAX_SPEED_RPM (350,
read as mm/s and rad/s) and gives the theta PID an output dead band of
2.0 rad/s -- with KP_THETA = 1, no rotation at all below 2 rad of error. The
X/Y output dead band of main.cpp (5 mm/s) is kept, it is harmless.

What stays out: the drivers (a wheel is taken to turn at its command, the MKS
closed loop and its internal ramp are not modelled), the RS485 latency, the
pose estimator. The loop runs on the ODOMETRY pose, as the firmware does.

Any change to the firmware files above must be carried over here, or the
simulator tests a robot that no longer exists.
"""

import math

from .kinematics import (
    ACCEL_MM_S2,
    ALPHA_RAD_S2,
    MAX_OMEGA_RAD_S,
    MAX_SPEED_MM_S,
    MAX_WHEEL_ACCEL_RPM_S,
    MAX_WHEEL_RPM,
    MM_S_TO_RPM,
    inverse_kinematics,
)

# config.h: ASSERVISSEMENT_FREQUENCY, 15 ms (66.7 Hz).
CONTROL_PERIOD_S = 0.015

# holonomic_basis.cpp: closer than this, the position loop stops correcting.
ARRIVED_DISTANCE_MM = 10.0
ARRIVED_ANGLE_RAD = 0.15

# holonomic_basis.cpp: lifetime of the SET_TARGET velocity, three periods of
# the 20 Hz high-level loop.
FEEDFORWARD_TTL_S = 0.150

# config.h: KP_X, KP_Y, KP_THETA (1/s). KI and KD are 0.
KP_XY = 1.0
KP_THETA = 1.0

# main.cpp: output dead band of the X/Y PIDs, mm/s.
OUTPUT_DEADBAND_XY_MM_S = 5.0


def braking_speed(distance: float, accel: float, dt: float) -> float:
    """Largest speed held over one tick from which a stop is still possible.

    Solves v dt + v^2 / (2 a) = d, the per-tick form of sqrt(2 a d): applied
    tick by tick, the continuous formula ends a few mm past the target.

    Args:
        distance: remaining distance (mm or rad).
        accel: deceleration (mm/s^2 or rad/s^2).
        dt: tick duration, s.

    Returns:
        float: the speed cap, zero for a non-positive distance or accel.
    """
    if distance <= 0.0 or accel <= 0.0:
        return 0.0
    a_dt = accel * dt
    return math.sqrt(a_dt * a_dt + 2.0 * accel * distance) - a_dt


def _p_output(error: float, kp: float, limit: float,
              deadband: float = 0.0) -> float:
    """Output of a P-only PID: gain, smooth dead band, clamp (lib/pid)."""
    value = kp * error
    magnitude = abs(value)
    if magnitude <= deadband:
        return 0.0
    value = math.copysign(magnitude - deadband, value)
    return max(-limit, min(limit, value))


class TeensyController:
    """Holonomic_Basis::handle() and WheelCommandShaper::step(), in Python.

    Time is the caller's: set_target() and tick() both take the simulated time,
    in seconds, where the firmware reads micros().
    """

    def __init__(self, x: float, y: float, theta: float) -> None:
        """Build the controller at rest, holding the given pose.

        Args:
            x: target X, in mm (table frame).
            y: target Y, in mm.
            theta: target heading, in rad.
        """
        self.target = (x, y, theta)
        self.feed_forward = (0.0, 0.0, 0.0)
        self.feed_forward_time = -FEEDFORWARD_TTL_S
        self.wheel_rpm = [0.0, 0.0, 0.0]

        # Flags of the last tick, as in ShapedCommand.
        self.braking = False
        self.speed_saturated = False
        self.accel_limited = False
        self.arrived = True

    def set_target(self, now: float, x: float, y: float, theta: float,
                   vx: float = 0.0, vy: float = 0.0,
                   omega: float = 0.0) -> None:
        """Take one SET_TARGET: the pose, and the velocity it moves at.

        Args:
            now: simulated time of reception, s.
            x, y, theta: pose of the reference, mm and rad (table frame).
            vx, vy, omega: velocity of the reference, mm/s and rad/s.
        """
        self.target = (x, y, theta)
        self.feed_forward = (vx, vy, omega)
        self.feed_forward_time = now

    def reset(self) -> None:
        """Wheels known to be at rest, no velocity (emergency_stop())."""
        self.wheel_rpm = [0.0, 0.0, 0.0]
        self.feed_forward = (0.0, 0.0, 0.0)
        self.braking = self.speed_saturated = self.accel_limited = False

    def reference(self, now: float) -> tuple:
        """Return the reference pose and the live feed-forward at time now.

        The pose of the last SET_TARGET moved on at its velocity for the time
        since. Past FEEDFORWARD_TTL_S the reference stops at the last
        extrapolated point and the feed-forward drops to zero: a silent sender
        never leaves the robot running on a stale velocity.

        Returns:
            tuple: ((x, y, theta), (vx, vy, omega)).
        """
        age = now - self.feed_forward_time
        alive = age < FEEDFORWARD_TTL_S
        if not alive:
            age = FEEDFORWARD_TTL_S
        vx, vy, omega = self.feed_forward
        x, y, theta = self.target
        pose = (x + vx * age, y + vy * age, theta + omega * age)
        ff = self.feed_forward if alive else (0.0, 0.0, 0.0)
        return pose, ff

    def tick(self, now: float, pose: tuple) -> list:
        """One control tick: the wheel speeds to hold until the next one.

        Args:
            now: simulated time of the tick, s.
            pose: (x, y, theta) the controller believes the robot is at, i.e.
                the odometry, never the true pose.

        Returns:
            list: speeds of wheels 1, 2 and 3, rpm.
        """
        dt = CONTROL_PERIOD_S
        (tx, ty, ttheta), (ff_vx, ff_vy, ff_omega) = self.reference(now)
        x, y, theta = pose

        xerr = tx - x
        yerr = ty - y
        theta_error = _wrap_angle(ttheta - theta)
        distance = math.hypot(xerr, yerr)

        # Dead zone: the position loop stops correcting and the robot comes to
        # rest through the acceleration limit. The feed-forward stays: close to
        # a moving target means the tracking is good, not that it is over.
        self.arrived = (distance < ARRIVED_DISTANCE_MM
                        and abs(theta_error) < ARRIVED_ANGLE_RAD)
        if self.arrived:
            vx = vy = omega = 0.0
        else:
            vx = _p_output(xerr, KP_XY, MAX_SPEED_MM_S,
                           OUTPUT_DEADBAND_XY_MM_S)
            vy = _p_output(yerr, KP_XY, MAX_SPEED_MM_S,
                           OUTPUT_DEADBAND_XY_MM_S)
            omega = _p_output(theta_error, KP_THETA, MAX_OMEGA_RAD_S)

        # 1. Braking cap on the position-loop part, then the feed-forward:
        # a moving target brakes through its own profile.
        braking = False
        v_cap = braking_speed(distance, ACCEL_MM_S2, dt)
        v_norm = math.hypot(vx, vy)
        if v_norm > v_cap:
            scale = v_cap / v_norm if v_norm > 0.0 else 0.0
            vx *= scale
            vy *= scale
            braking = True
        omega_cap = braking_speed(abs(theta_error), ALPHA_RAD_S2, dt)
        if abs(omega) > omega_cap:
            omega = math.copysign(omega_cap, omega)
            braking = True

        vx += ff_vx
        vy += ff_vy
        omega += ff_omega

        # 2. Table frame to robot frame, by the heading the robot believes in.
        c, s = math.cos(theta), math.sin(theta)
        rx = c * vx + s * vy
        ry = -s * vx + c * vy

        # 3. Inverse kinematics, wheel surface speed to rpm.
        requested = [v * MM_S_TO_RPM for v in inverse_kinematics(rx, ry, omega)]

        # 4. Proportional normalisation: the direction holds.
        fastest = max(abs(v) for v in requested)
        self.speed_saturated = fastest > MAX_WHEEL_RPM
        if self.speed_saturated:
            requested = [v * MAX_WHEEL_RPM / fastest for v in requested]

        # 5. Synchronised acceleration limit, per wheel: the three changes are
        # scaled by the same factor, so the robot keeps its heading in a ramp.
        change = [r - w for r, w in zip(requested, self.wheel_rpm)]
        max_change = MAX_WHEEL_ACCEL_RPM_S * dt
        largest = max(abs(c_) for c_ in change)
        self.accel_limited = largest > max_change
        if self.accel_limited:
            change = [c_ * max_change / largest for c_ in change]

        self.wheel_rpm = [w + c_ for w, c_ in zip(self.wheel_rpm, change)]
        self.braking = braking
        return list(self.wheel_rpm)


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
