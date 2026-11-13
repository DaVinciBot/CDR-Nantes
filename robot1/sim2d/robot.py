"""The simulated robot: true pose, motion, and drifting odometry.

Two poses live here and must never be confused. The TRUE pose is where the
robot really is, and only the simulator knows it. The ODOMETRY pose is what the
Teensy would report, drift included, and it is the only one the match loop ever
sees. Comparing the two is the whole point of the simulator.

The motion is the control law of the Teensy (teensy.py: extrapolated target,
P + feed-forward, braking cap, per-wheel limiter), run every 15 ms like the
firmware ISR, on the odometry pose. The wheels are then taken to turn exactly
at their command: the drivers, wheel slip and the physics of the base are not
modelled. The sim tells whether the match loop drives the 2027 control law
sensibly; settling, overshoot on the real base and gains are validated on the
bench.
"""

import collections
import math
import random
from dataclasses import dataclass

from .kinematics import (
    MM_S_TO_RPM,
    forward_kinematics,
)
from .teensy import CONTROL_PERIOD_S, TeensyController

# The firmware sends ODOMETRY every second control tick (30 ms, 33.3 Hz).
ODOMETRY_EVERY_TICKS = 2

# Samples kept for the transport. Bounded so a run that never reads them
# cannot grow without limit; a reader that keeps up never sees more than two.
ODOMETRY_BACKLOG = 64


@dataclass
class OdometrySample:
    """What the firmware would put in one ODOMETRY frame."""

    t: float          # simulated time of the sample, s
    x: float          # odometry pose, mm and rad (table frame)
    y: float
    theta: float
    vx: float         # velocity, table frame, mm/s and rad/s
    vy: float
    omega: float


@dataclass
class DriftModel:
    """How the odometry of the simulated robot goes wrong.

    Two mechanisms, because they are told apart by very different tests: a
    systematic scale error, which grows with distance and is what a wrong wheel
    diameter produces, and a random walk, which is sensor noise.

    Defaults are plausible placeholders, NOT measured values. They are to be
    fitted against real robot logs, once those can be replayed.
    """

    translation_scale: float = 0.002        # 0.2 %, i.e. 2 mm per metre
    heading_scale: float = 0.002            # 0.2 % of the angle travelled
    translation_noise_mm_per_sqrt_m: float = 0.5
    heading_noise_rad_per_sqrt_rad: float = 0.002


class SimulatedRobot:
    """Holonomic base with a true pose and a drifting odometry."""

    def __init__(
        self,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        drift: DriftModel = None,
        seed: int = 0,
    ) -> None:
        """Build the robot at a given pose.

        Args:
            x: initial true X, in mm (field frame).
            y: initial true Y, in mm.
            theta: initial true heading, in rad.
            drift: odometry error model. Default DriftModel().
            seed: seed of the noise generator, so a run is reproducible.
        """
        self.true_x = x
        self.true_y = y
        self.true_theta = theta

        self.odom_x = x
        self.odom_y = y
        self.odom_theta = theta

        # Simulated time seen by the firmware, s. Advanced by step() only.
        self.time = 0.0
        self.controller = TeensyController(x, y, theta)
        self.wheel_rpm = [0.0, 0.0, 0.0]
        self._next_tick = 0.0
        self._ticks = 0
        self.odometry_samples = collections.deque(maxlen=ODOMETRY_BACKLOG)

        self.vx = 0.0            # true velocity, field frame, mm/s
        self.vy = 0.0
        self.omega = 0.0
        self.max_speed_mm_s = 0.0

        self.distance_travelled_mm = 0.0
        self.angle_travelled_rad = 0.0

        # Distance the odometry BELIEVES it travelled, and the worst position
        # error seen during the run. Both exist because the final position
        # error is a bad measure of a scale error: on a closed path the error
        # of the outward legs cancels the error of the return legs. That is why
        # the classic square test detects rotational errors, not scale ones.
        self.odom_distance_mm = 0.0
        self.max_position_error_mm = 0.0

        self.drift = drift if drift is not None else DriftModel()
        self._rng = random.Random(seed)

    # ── Commands, as they arrive from the match loop ──────────────────────

    def set_target(self, x: float, y: float, theta: float,
                   vx: float = 0.0, vy: float = 0.0,
                   omega: float = 0.0) -> None:
        """Take one SET_TARGET: reference pose and its velocity, field frame.

        Received at the current simulated time: the velocity lives 150 ms
        from now, as on the Teensy.
        """
        self.controller.set_target(self.time, x, y, theta, vx, vy, omega)

    @property
    def target_x(self) -> float:
        """X of the last SET_TARGET, in mm (not extrapolated)."""
        return self.controller.target[0]

    @property
    def target_y(self) -> float:
        """Y of the last SET_TARGET, in mm (not extrapolated)."""
        return self.controller.target[1]

    @property
    def target_theta(self) -> float:
        """Heading of the last SET_TARGET, in rad (not extrapolated)."""
        return self.controller.target[2]

    def reset_odometry(self, x: float, y: float, theta: float) -> None:
        """Force both poses to a known value (SET_ODOMETRIE).

        The true pose follows, because that is what the real command means on
        the table: the robot is placed there and told where it is.
        """
        self.true_x = self.odom_x = x
        self.true_y = self.odom_y = y
        self.true_theta = self.odom_theta = theta
        self.vx = self.vy = self.omega = 0.0
        # The robot was placed: the wheels are at rest and the velocity of the
        # trajectory it was on no longer applies. The target pose is kept.
        self.controller.reset()
        self.wheel_rpm = [0.0, 0.0, 0.0]

    # ── Integration ───────────────────────────────────────────────────────

    def step(self, dt: float) -> None:
        """Advance the simulation by dt seconds.

        The firmware ticks every CONTROL_PERIOD_S whatever dt is: the motion
        is integrated piecewise, each wheel command held from its tick to the
        next (zero-order hold), as the drivers hold the last speed sent.

        Args:
            dt: time step, in seconds. Must be > 0.
        """
        if dt <= 0.0:
            return
        end = self.time + dt
        while True:
            if self._next_tick <= self.time + 1e-12:
                self._control_tick()
            segment_end = min(end, self._next_tick)
            self._move(segment_end - self.time)
            self.time = segment_end
            if self.time >= end - 1e-12:
                self.time = end
                break

    def _control_tick(self) -> None:
        """One firmware tick: telemetry first, then the new wheel command."""
        if self._ticks % ODOMETRY_EVERY_TICKS == 0:
            self.odometry_samples.append(OdometrySample(
                self.time, self.odom_x, self.odom_y, self.odom_theta,
                self.vx, self.vy, self.omega,
            ))
        self._ticks += 1
        self._next_tick += CONTROL_PERIOD_S
        # The controller servoes on the ODOMETRY, never on the true pose: no
        # real controller can see where it truly is, and pretending otherwise
        # hides the only failure mode that matters. Servoing on the true pose
        # made the robot chase its own drift forever -- app.py stops by
        # commanding "go to your current estimated pose", which leaves a
        # permanent error when the estimate is not the truth, and the robot
        # crept 1.2 m out of the table over a match (measured 22/09/2026).
        self.wheel_rpm = self.controller.tick(self.time, self.odometry_pose())
        self._update_velocity()

    def _update_velocity(self) -> None:
        """True field-frame velocity produced by the current wheel speeds.

        The controller turned a field-frame velocity into wheel speeds using
        the heading it BELIEVES it has; the base executes them in its TRUE
        frame. This is how a heading error becomes a position error: the robot
        moves confidently in the wrong direction.
        """
        surface = [rpm / MM_S_TO_RPM for rpm in self.wheel_rpm]
        rx, ry, omega = forward_kinematics(*surface)
        cos_t, sin_t = math.cos(self.true_theta), math.sin(self.true_theta)
        self.vx = rx * cos_t - ry * sin_t
        self.vy = rx * sin_t + ry * cos_t
        self.omega = omega
        self.max_speed_mm_s = max(self.max_speed_mm_s,
                                  math.hypot(self.vx, self.vy))

    def _move(self, dt: float) -> None:
        """Integrate the held wheel speeds over dt seconds."""
        if dt <= 0.0:
            return
        # Integrate at the midpoint heading: over a long segment at speed the
        # end point heading would already be off.
        mid_theta = self.true_theta + 0.5 * self.omega * dt
        surface = [rpm / MM_S_TO_RPM for rpm in self.wheel_rpm]
        rx, ry, _ = forward_kinematics(*surface)
        cos_m, sin_m = math.cos(mid_theta), math.sin(mid_theta)
        dx = (rx * cos_m - ry * sin_m) * dt
        dy = (rx * sin_m + ry * cos_m) * dt
        dtheta = self.omega * dt

        self.true_x += dx
        self.true_y += dy
        self.true_theta = _wrap_angle(self.true_theta + dtheta)
        self._update_velocity()

        self.distance_travelled_mm += math.hypot(dx, dy)
        self.angle_travelled_rad += abs(dtheta)

        self._integrate_odometry(dx, dy, dtheta, mid_theta)

        self.max_position_error_mm = max(
            self.max_position_error_mm, self.position_error_mm()
        )

    def _integrate_odometry(
        self, dx: float, dy: float, dtheta: float, mid_theta: float,
    ) -> None:
        """Accumulate the measured displacement, wrong on purpose.

        The error is applied to the DISPLACEMENT, never to the pose: that is
        what makes it accumulate the way a real odometry does, and what makes
        the total error grow with the distance travelled rather than with time.
        """
        step_mm = math.hypot(dx, dy)
        d = self.drift

        scale = 1.0 + d.translation_scale
        noise = 0.0
        if d.translation_noise_mm_per_sqrt_m > 0.0 and step_mm > 0.0:
            sigma = d.translation_noise_mm_per_sqrt_m * math.sqrt(
                step_mm / 1000.0
            )
            noise = self._rng.gauss(0.0, sigma)

        measured_mm = step_mm * scale + noise
        self.odom_distance_mm += abs(measured_mm)
        if step_mm > 0.0:
            ux, uy = dx / step_mm, dy / step_mm
        else:
            ux, uy = 0.0, 0.0
        self.odom_x += ux * measured_mm
        self.odom_y += uy * measured_mm

        theta_noise = 0.0
        if d.heading_noise_rad_per_sqrt_rad > 0.0 and abs(dtheta) > 0.0:
            sigma_t = d.heading_noise_rad_per_sqrt_rad * math.sqrt(
                abs(dtheta)
            )
            theta_noise = self._rng.gauss(0.0, sigma_t)
        self.odom_theta = _wrap_angle(
            self.odom_theta + dtheta * (1.0 + d.heading_scale) + theta_noise
        )

    # ── Read-only views ───────────────────────────────────────────────────

    def true_pose(self) -> tuple:
        """Return the true pose (x_mm, y_mm, theta_rad). Simulator only."""
        return self.true_x, self.true_y, self.true_theta

    def odometry_pose(self) -> tuple:
        """Return the pose the Teensy would report (x_mm, y_mm, theta_rad)."""
        return self.odom_x, self.odom_y, self.odom_theta

    def position_error_mm(self) -> float:
        """Return the distance between the odometry and the true position."""
        return math.hypot(
            self.odom_x - self.true_x, self.odom_y - self.true_y
        )

    def heading_error_rad(self) -> float:
        """Return the signed heading error of the odometry."""
        return _wrap_angle(self.odom_theta - self.true_theta)

    def wheel_speeds(self) -> tuple:
        """Return the three wheel surface speeds being commanded, mm/s."""
        return tuple(rpm / MM_S_TO_RPM for rpm in self.wheel_rpm)

    def body_velocity(self) -> tuple:
        """Return (vx, vy, omega) in the robot frame, from the wheel speeds."""
        return forward_kinematics(*self.wheel_speeds())


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
