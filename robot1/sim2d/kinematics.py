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

# config.h: ROBOT_RADIUS, WHEEL_DIAMETER, MAX_SPEED_RPM.
ROBOT_RADIUS_MM = 160.0
WHEEL_DIAMETER_MM = 60.0
MAX_WHEEL_RPM = 350.0

# Wheel surface speed (mm/s) to wheel speed (rpm), as in lib/motion.
MM_S_TO_RPM = 60.0 / (math.pi * WHEEL_DIAMETER_MM)

# A wheel turning at MAX_WHEEL_RPM covers this many mm per second (1099.6).
MAX_WHEEL_SPEED_MM_S = MAX_WHEEL_RPM / MM_S_TO_RPM

# config.h: MAX_WHEEL_ACCEL_RPM_S, 1 m/s^2 at the wheel (318.3 rpm/s) rounded
# up. Limit per wheel: the robot itself accelerates 1 to 1.15 times faster,
# depending on the direction.
MAX_WHEEL_ACCEL_RPM_S = 320.0

# config.h: ROBOT_MAX_SPEED_MM_S, ROBOT_MAX_RAD_S, ROBOT_MAX_ACCEL_MM_S2.
# Commanded targets of the base, below the wheel ceiling: 1 m/s needs 318.3 rpm,
# the remaining ~30 rpm leave room for rotation during a full-speed translation.
MAX_SPEED_MM_S = 1000.0
MAX_OMEGA_RAD_S = 3.0
ACCEL_MM_S2 = 1000.0
# The same wheel-level budget spent on rotation: alpha = a / R (6.25 rad/s^2).
ALPHA_RAD_S2 = ACCEL_MM_S2 / ROBOT_RADIUS_MM


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

