"""The simulated robot: true pose, motion, and drifting odometry.

Two poses live here and must never be confused. The TRUE pose is where the
robot really is, and only the simulator knows it. The ODOMETRY pose is what the
Teensy would report, drift included, and it is the only one the match loop ever
sees. Comparing the two is the whole point of the simulator.

What this is NOT: a model of the firmware. The motion below is a bounded
proportional controller, not the PID of holonomic_basis.cpp, and it is not
meant to predict how the real base settles on a target. PLAN_REFONTE section 14,
principle 1: simulate the world and the sensors, never the firmware. Anything
that depends on the real control loop is validated on the bench, not here.
"""

import math
import random
from dataclasses import dataclass

from .kinematics import (
    MAX_OMEGA_RAD_S,
    MAX_SPEED_MM_S,
    clamp_to_wheel_limits,
    forward_kinematics,
    inverse_kinematics,
)

# Approach gains of the simulated base. Deliberately not the firmware PID.
APPROACH_GAIN = 2.0          # 1/s, on the distance to the target
APPROACH_GAIN_THETA = 3.0    # 1/s, on the heading error
ACCEL_MM_S2 = 800.0          # acceleration ceiling, translation
ACCEL_RAD_S2 = 8.0           # acceleration ceiling, rotation

# Below these the base is considered arrived and stops asking for speed.
DEADZONE_MM = 5.0
DEADZONE_RAD = 0.02


@dataclass
class DriftModel:
    """How the odometry of the simulated robot goes wrong.

    Two mechanisms, because they are told apart by very different tests: a
    systematic scale error, which grows with distance and is what a wrong wheel
    diameter produces, and a random walk, which is sensor noise.

    Defaults are plausible placeholders, NOT measured values. They will be
    fitted against real logs in lot 8 (replay), see PLAN_REFONTE section 14.
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

        self.target_x = x
        self.target_y = y
        self.target_theta = theta

        self.vx = 0.0            # field frame, mm/s
        self.vy = 0.0
        self.omega = 0.0

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

    def set_target(self, x: float, y: float, theta: float) -> None:
        """Set the target pose, in field coordinates (SET_TARGET_POSITION)."""
        self.target_x = x
        self.target_y = y
        self.target_theta = theta

    def reset_odometry(self, x: float, y: float, theta: float) -> None:
        """Force both poses to a known value (SET_ODOMETRIE).

        The true pose follows, because that is what the real command means on
        the table: the robot is placed there and told where it is.
        """
        self.true_x = self.odom_x = x
        self.true_y = self.odom_y = y
        self.true_theta = self.odom_theta = theta
        self.vx = self.vy = self.omega = 0.0

    # ── Integration ───────────────────────────────────────────────────────

    def step(self, dt: float) -> None:
        """Advance the simulation by dt seconds.

        Args:
            dt: time step, in seconds. Must be > 0.
        """
        if dt <= 0.0:
            return

        wanted_vx, wanted_vy, wanted_omega = self._wanted_velocity()
        self._apply_acceleration(wanted_vx, wanted_vy, wanted_omega, dt)

        # Integrate at the midpoint heading: at 20 Hz and 2 rad/s the end point
        # heading would already be off by a degree per step.
        mid_theta = self.true_theta + 0.5 * self.omega * dt
        dx = self.vx * dt
        dy = self.vy * dt
        dtheta = self.omega * dt

        self.true_x += dx
        self.true_y += dy
        self.true_theta = _wrap_angle(self.true_theta + dtheta)

        self.distance_travelled_mm += math.hypot(dx, dy)
        self.angle_travelled_rad += abs(dtheta)

        self._integrate_odometry(dx, dy, dtheta, mid_theta)

        self.max_position_error_mm = max(
            self.max_position_error_mm, self.position_error_mm()
        )

    def _wanted_velocity(self) -> tuple:
        """Proportional approach of the target, within the base limits.

        The error is measured against the ODOMETRY, never against the true
        pose: no real controller can see where it truly is, and pretending
        otherwise hides the only failure mode that matters. Servoing on the
        true pose made the robot chase its own drift forever -- app.py stops by
        commanding "go to your current estimated pose", which leaves a
        permanent error when the estimate is not the truth, and the robot crept
        1.2 m out of the table over a match (measured 22/09/2026).
        """
        ex = self.target_x - self.odom_x
        ey = self.target_y - self.odom_y
        etheta = _wrap_angle(self.target_theta - self.odom_theta)

        distance = math.hypot(ex, ey)
        if distance < DEADZONE_MM:
            vx = vy = 0.0
        else:
            speed = min(APPROACH_GAIN * distance, MAX_SPEED_MM_S)
            vx = speed * ex / distance
            vy = speed * ey / distance

        if abs(etheta) < DEADZONE_RAD:
            omega = 0.0
        else:
            omega = max(
                -MAX_OMEGA_RAD_S,
                min(MAX_OMEGA_RAD_S, APPROACH_GAIN_THETA * etheta),
            )

        # The controller turns a field-frame velocity into wheel speeds using
        # the heading it BELIEVES it has.
        cos_e, sin_e = math.cos(self.odom_theta), math.sin(self.odom_theta)
        rx = vx * cos_e + vy * sin_e
        ry = -vx * sin_e + vy * cos_e

        # The wheel ceiling applies in the robot frame. Scaling the whole twist
        # is what the firmware does; this is where a diagonal move plus a fast
        # rotation ends up slower than either alone.
        rx, ry, omega = clamp_to_wheel_limits(rx, ry, omega)

        # The physical base then executes those wheel speeds in its TRUE frame.
        # This is how a heading error becomes a position error: the robot moves
        # confidently in the wrong direction.
        cos_t, sin_t = math.cos(self.true_theta), math.sin(self.true_theta)
        vx = rx * cos_t - ry * sin_t
        vy = rx * sin_t + ry * cos_t
        return vx, vy, omega

    def _apply_acceleration(
        self, wanted_vx: float, wanted_vy: float, wanted_omega: float,
        dt: float,
    ) -> None:
        """Move the current velocity toward the wanted one, within limits."""
        max_dv = ACCEL_MM_S2 * dt
        dvx = wanted_vx - self.vx
        dvy = wanted_vy - self.vy
        norm = math.hypot(dvx, dvy)
        if norm > max_dv and norm > 0.0:
            dvx *= max_dv / norm
            dvy *= max_dv / norm
        self.vx += dvx
        self.vy += dvy

        max_domega = ACCEL_RAD_S2 * dt
        domega = wanted_omega - self.omega
        self.omega += max(-max_domega, min(max_domega, domega))

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
        """Return the three wheel speeds matching the current motion, mm/s.

        Lot 4 will log these as one of the synthetic sensor streams; they are
        exposed now because they cost nothing and they prove the kinematics is
        actually in the loop.
        """
        cos_t, sin_t = math.cos(self.true_theta), math.sin(self.true_theta)
        rx = self.vx * cos_t + self.vy * sin_t
        ry = -self.vx * sin_t + self.vy * cos_t
        return inverse_kinematics(rx, ry, self.omega)

    def body_velocity(self) -> tuple:
        """Return (vx, vy, omega) in the robot frame, from the wheel speeds."""
        return forward_kinematics(*self.wheel_speeds())


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
