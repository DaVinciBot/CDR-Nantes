"""Holonomic kinematics of the three-wheel base.

Same geometry as robot1/teensy_moteur/lib/holonomic_basis: three omni wheels at
120 degrees, W1 at 120, W2 at 240, W3 at 0. A wheel placed at angle a rolls
along the tangent (-sin a, cos a), so its linear speed is

    v_i = -sin(a_i) * vx + cos(a_i) * vy + R * omega

with vx, vy expressed in the ROBOT frame. The three wheels being evenly spaced,
the matrix is invertible and the inverse has a closed form, used below.

Constants come from robot1/teensy_moteur/include/config.h. ROBOT_RADIUS is the
kinematic radius, 160 mm; it has nothing to do with the ROBOT_RADIUS_MM of
rasp/world.py (120 mm, obstacle inflation for A*). Never align the two.
"""

import math

# Wheel positions on the chassis, in the robot frame (degrees).
WHEEL_ANGLES_DEG = (120.0, 240.0, 0.0)
WHEEL_ANGLES_RAD = tuple(math.radians(a) for a in WHEEL_ANGLES_DEG)

# config.h: ROBOT_RADIUS, WHEEL_DIAMETER, MKS_MAX_RPM.
ROBOT_RADIUS_MM = 160.0
WHEEL_DIAMETER_MM = 60.0
MAX_WHEEL_RPM = 100.0

# A wheel turning at MAX_WHEEL_RPM covers this many mm per second.
MAX_WHEEL_SPEED_MM_S = MAX_WHEEL_RPM / 60.0 * math.pi * WHEEL_DIAMETER_MM

# config.h: measured ceiling of the whole base, well under the wheel ceiling.
MAX_SPEED_MM_S = 200.0
MAX_OMEGA_RAD_S = 2.0


def inverse_kinematics(vx: float, vy: float, omega: float) -> tuple:
    """Turn a robot-frame twist into the three wheel speeds.

    Args:
        vx: forward speed along the robot X axis, in mm/s.
        vy: lateral speed along the robot Y axis, in mm/s.
        omega: rotation speed, in rad/s, counterclockwise.

    Returns:
        tuple: (v1, v2, v3) wheel linear speeds, in mm/s.
    """
    return tuple(
        -math.sin(a) * vx + math.cos(a) * vy + ROBOT_RADIUS_MM * omega
        for a in WHEEL_ANGLES_RAD
    )


def forward_kinematics(v1: float, v2: float, v3: float) -> tuple:
    """Turn the three wheel speeds back into a robot-frame twist.

    Exact inverse of inverse_kinematics() for evenly spaced wheels.

    Args:
        v1: speed of wheel 1, in mm/s.
        v2: speed of wheel 2, in mm/s.
        v3: speed of wheel 3, in mm/s.

    Returns:
        tuple: (vx, vy, omega) in mm/s and rad/s.
    """
    speeds = (v1, v2, v3)
    vx = (2.0 / 3.0) * sum(
        -math.sin(a) * v for a, v in zip(WHEEL_ANGLES_RAD, speeds)
    )
    vy = (2.0 / 3.0) * sum(
        math.cos(a) * v for a, v in zip(WHEEL_ANGLES_RAD, speeds)
    )
    omega = sum(speeds) / (3.0 * ROBOT_RADIUS_MM)
    return vx, vy, omega


def clamp_to_wheel_limits(vx: float, vy: float, omega: float) -> tuple:
    """Scale a twist down until every wheel stays within its speed limit.

    Scaling the whole twist, instead of clipping each wheel on its own, is what
    the firmware does (proportional normalisation): clipping wheel by wheel
    changes the direction of travel, scaling only makes the robot slower.

    Args:
        vx: forward speed, in mm/s.
        vy: lateral speed, in mm/s.
        omega: rotation speed, in rad/s.

    Returns:
        tuple: (vx, vy, omega), scaled down if needed.
    """
    speeds = inverse_kinematics(vx, vy, omega)
    worst = max(abs(v) for v in speeds)
    if worst <= MAX_WHEEL_SPEED_MM_S or worst == 0.0:
        return vx, vy, omega
    factor = MAX_WHEEL_SPEED_MM_S / worst
    return vx * factor, vy * factor, omega * factor
