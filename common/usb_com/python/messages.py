"""Message identifiers, payload layouts and constants for Teensy USB communication.

Mirror of ``common/usb_com/cpp/include/messages.h``: an ID, a layout or a flag
changed on one side must be changed on the other in the same commit.
"""

from __future__ import annotations

import struct
from enum import Enum, IntFlag

# Every frame on the wire ends with this byte. COBS keeps it out of the encoded
# frame, so it always marks a frame boundary (see com/framing.py).
FRAME_DELIMITER: bytes = b"\x00"

# ID + data of one message, at most.
MAX_PAYLOAD_SIZE = 255


class Messages(Enum):
    r"""Enumeration for command types exchanged between the Raspberry Pi and Teensy.

    Commands from Raspberry Pi to Teensy are in the range ``0``-``127`` while
    those from Teensy to Raspberry Pi are in ``128``-``255``.

    Attributes:
        SET_TARGET: Target pose and trajectory velocity of the rolling basis.
        SET_PID: Update PID coefficients for the rolling basis.
        SET_ODOMETRIE: Reset the odometry.
        SET_SERVO_ANGLE_I2C: Set a servo angle over I\ :sub:``2``\ C.
        STEPPER_STEP: Move a stepper motor.
        SET_SERVO_ANGLE_DETACH: Move a servo then detach it.
        ATTACH_SWITCH: Attach a switch on the actuators board.
        SET_SERVO_ANGLE: Set a servo angle.
        SET_STEPPER_DRIVER_ACTIVATION_STATE: Enable or disable the stepper driver.
        STOP: Immediate stop of the rolling basis, back to idle.
        SET_ESTIMATOR: Select the pose estimator (refused while moving).
        SET_TRACE: Choose the per-tick messages to send (``TraceMask``).
        HEARTBEAT: Keep the Teensy watchdog fed.
        RESET_TEENSY: Reset the Teensy board.
        NACK: Notification of an invalid command.
        ODOMETRY: Pose and velocity estimated by the Teensy.
        SWITCH_STATE_RETURN: Report the state of a switch.
        STATUS: State of the rolling basis board.
        PID_TRACE: One line of PID trace per control tick.
        LOG_TICK: One line of sensor and estimator log per control tick.
        LOG_EVENT: Something happened on the board (boot, pose set, stop...).
        LOG_FW_INFO: Which firmware is running.
        LOG_PARAMS: The constants a replay or a calibration depends on.
        PRINT: Print a message from the Teensy.
        UNKNOWN_MSG_TYPE: Unknown command identifier.
    """

    # rasp -> teensy : 0-127 (Convention)

    # Rolling Basis
    SET_TARGET = 0
    """Target pose and trajectory velocity of the rolling basis."""
    SET_PID = 1
    """Update PID coefficients for the rolling basis."""
    SET_ODOMETRIE = 2
    """Reset the odometry."""

    # Actuators
    SET_SERVO_ANGLE_I2C = 3
    """Set a servo angle over I2C."""
    STEPPER_STEP = 4
    """Move a stepper motor."""
    SET_SERVO_ANGLE_DETACH = 5
    """Move a servo then detach it."""
    ATTACH_SWITCH = 6
    """Attach a switch on the actuators board."""
    SET_SERVO_ANGLE = 7
    """Set a servo angle."""
    SET_STEPPER_DRIVER_ACTIVATION_STATE = 8
    """Enable or disable the stepper driver."""

    # Rolling Basis
    STOP = 9
    """Immediate stop of the rolling basis, back to idle."""
    SET_ESTIMATOR = 10
    """Select the pose estimator (refused while moving)."""
    SET_TRACE = 11
    """Choose the per-tick messages to send (``TraceMask``)."""
    HEARTBEAT = 12
    """Keep the Teensy watchdog fed."""

    # Common (Rolling Basis + Actuators)
    RESET_TEENSY = 126
    """Reset the Teensy board."""

    # two ways : 127 (Convention)
    NACK = 127
    """Notification of an invalid command."""

    # teensy -> rasp : 128-255 (Convention)
    # Rolling Basis
    ODOMETRY = 128
    """Pose and velocity estimated by the Teensy."""

    # Actuators
    SWITCH_STATE_RETURN = 129
    """Report the state of a switch."""

    # Rolling Basis
    STATUS = 130
    """State of the rolling basis board."""
    PID_TRACE = 131
    """One line of PID trace per control tick."""

    # Rolling Basis, logs
    LOG_TICK = 200
    """One line of sensor and estimator log per control tick."""
    LOG_EVENT = 201
    """Something happened on the board (boot, pose set, stop...)."""
    LOG_FW_INFO = 202
    """Which firmware is running."""
    LOG_PARAMS = 203
    """The constants a replay or a calibration depends on."""

    # Common (Rolling Basis + Actuators)
    PRINT = 254
    """Print a message from the Teensy."""
    UNKNOWN_MSG_TYPE = 255
    """Unknown command identifier."""

    # To use for message creation
    def to_bytes(self) -> bytes:
        """Converts the command to its byte representation.

        Returns:
            bytes: Single-byte representation of the command.
        """
        return bytes([self.value])


class Estimator(Enum):
    """Pose estimators of the rolling basis (``SET_ESTIMATOR``, ``STATUS``)."""

    COMPLEMENTARY = 0
    EKF6 = 1


class StatusFlag(IntFlag):
    """``STATUS`` flags."""

    MOTORS_ENABLED = 1 << 0
    MOVING = 1 << 1
    WATCHDOG_TRIPPED = 1 << 2
    """No message for the watchdog timeout."""
    TIMEOUT_TRIPPED = 1 << 3
    """Movement took too long."""
    IMU_OK = 1 << 4
    OPTICAL_OK = 1 << 5
    ESTIMATOR_REFUSED = 1 << 6
    """Last ``SET_ESTIMATOR`` refused (the wheels were turning)."""
    TRACE_ON = 1 << 7
    """``PID_TRACE`` is being sent."""
    # Bits 8 and 9 are kept for the "base blocked" and "drivers unpowered" flags.
    LOG_TICK_ON = 1 << 10
    """``LOG_TICK`` is being sent."""


class TraceMask(IntFlag):
    """``SET_TRACE`` mask: the per-tick messages to send. Both are off at boot.

    The value 1 keeps the meaning it had when the field was a plain on / off.
    """

    NONE = 0
    PID_TRACE = 1 << 0
    LOG_TICK = 1 << 1


class TraceFlag(IntFlag):
    """``PID_TRACE`` flags."""

    BRAKING = 1 << 0
    SPEED_SATURATED = 1 << 1
    ACCEL_LIMITED = 1 << 2
    INTEGRAL_FROZEN = 1 << 3


# Layout version of the LOG_* messages, carried by each of them. A log file is
# decoded with the layout it was recorded with.
LOG_SCHEMA_VERSION = 1


class LogInputFlag(IntFlag):
    """``LOG_TICK`` ``input_flags``: the booleans of the estimator input."""

    FIRST_FRAME = 1 << 0
    ENCODERS_FRESH = 1 << 1
    IMU_VALID = 1 << 2
    OPTICAL_STEP_VALID = 1 << 3
    OPTICAL_VELOCITY_VALID = 1 << 4


class LogEstFlag(IntFlag):
    """``LOG_TICK`` ``est_flags``: what the estimator did with the optical sample."""

    OPTICAL_USED = 1 << 0
    OPTICAL_REJECTED = 1 << 1
    PURE_ROTATION = 1 << 2


class LogHealthFlag(IntFlag):
    """``LOG_TICK`` ``health_flags``."""

    OVERRUN = 1 << 0
    """The work of the tick overran the period."""
    OPTICAL_READ = 1 << 1
    """A new optical sample this tick."""
    IMU_READ = 1 << 2
    """A new IMU sample this tick."""
    WRITE_FAILED = 1 << 3
    """Wheel speeds not sent this tick."""


class Rs485Fail(Enum):
    """``LOG_TICK`` ``rs485_fail``: why the encoder read of a wheel failed."""

    OK = 0
    NO_ANSWER = 1
    BAD_CHECKSUM = 2
    SHORT_FRAME = 3
    BAD_ADDRESS = 4


class LogEvent(Enum):
    """``LOG_EVENT`` ``event``."""

    BOOT = 0
    ESTIMATOR_SELECTED = 1
    """``arg`` = ``Estimator`` value."""
    POSE_SET = 2
    """``value`` = x, y, theta imposed on the estimator."""
    WATCHDOG = 3
    STOP = 4
    BLOCKED = 5
    DRIVERS_UNPOWERED = 6
    """``arg`` = 1 when lost, 0 when back."""
    ERROR = 7
    """``arg`` = error code."""


# Layout of the data that follows the ID byte, little-endian and without
# padding, exactly as the packed C++ structs. Units: mm, rad, s, us, rpm.
# Rolling basis and common messages only; the actuator messages are packed by
# their callers.
PAYLOAD_FORMATS: dict[Messages, str] = {
    # x, y, theta, vx, vy, omega (table frame; velocity zero for a plain pose)
    Messages.SET_TARGET: "<6f",
    # pid_type, kp, ki, kd
    Messages.SET_PID: "<B3f",
    # x, y, theta
    Messages.SET_ODOMETRIE: "<3d",
    Messages.STOP: "<",
    # Estimator value
    Messages.SET_ESTIMATOR: "<B",
    # TraceMask
    Messages.SET_TRACE: "<B",
    Messages.HEARTBEAT: "<",
    Messages.RESET_TEENSY: "<",
    # x, y, theta, vx, vy, omega (table frame), t_us
    Messages.ODOMETRY: "<6fI",
    # t_us, flags, estimator, estimator_health, rs485_read_failures,
    # rs485_write_failures
    Messages.STATUS: "<IHBBII",
    # t_us, target[3], pose[3], velocity_cmd[3], wheel_requested[3],
    # wheel_cmd[3], wheel_measured[3], p_term[3], i_term[3], d_term[3], flags
    Messages.PID_TRACE: "<I27fB",
    # Field by field: PAYLOAD_FIELDS below. One group per block of the struct.
    Messages.LOG_TICK: (
        "<BHII"  # header
        "3iIIfIIIffIffB"  # estimator input
        "3qIB"  # raw encoders
        "9f"  # raw optical sensor
        "ffB"  # raw IMU
        "6fBB"  # estimator output
        "HI"  # commands
        "HB3BHH"  # cycle health
    ),
    Messages.LOG_EVENT: "<BHIIB3fI",
    Messages.LOG_FW_INFO: "<B24s20s",
    Messages.LOG_PARAMS: (
        "<BBI"  # schema version, estimator, period
        "f3f3ff"  # geometry
        "ffIff3fB"  # complementary filter
        "5f"  # optical sensor
        "9f"  # position PIDs
        "2f"  # command limits
    ),
}

# Head of LOG_TICK and LOG_EVENT.
_LOG_HEADER: tuple[tuple[str, int], ...] = (
    ("schema_version", 1),
    ("seq", 1),
    ("tick", 1),
    ("t_us", 1),
)

# Name of every field, in the order of PAYLOAD_FORMATS, with the number of
# values it holds: 1 for a scalar (or a string), n for an array of n. Same
# names as the C++ structs. unpack_fields() reads a message with it, so a
# decoder never has to copy the schema.
PAYLOAD_FIELDS: dict[Messages, tuple[tuple[str, int], ...]] = {
    Messages.SET_TARGET: (
        ("x", 1), ("y", 1), ("theta", 1), ("vx", 1), ("vy", 1), ("omega", 1),
    ),
    Messages.SET_PID: (("pid_type", 1), ("kp", 1), ("ki", 1), ("kd", 1)),
    Messages.SET_ODOMETRIE: (("x", 1), ("y", 1), ("theta", 1)),
    Messages.STOP: (),
    Messages.SET_ESTIMATOR: (("estimator", 1),),
    Messages.SET_TRACE: (("mask", 1),),
    Messages.HEARTBEAT: (),
    Messages.RESET_TEENSY: (),
    Messages.ODOMETRY: (
        ("x", 1), ("y", 1), ("theta", 1), ("vx", 1), ("vy", 1), ("omega", 1),
        ("t_us", 1),
    ),
    Messages.STATUS: (
        ("t_us", 1), ("flags", 1), ("estimator", 1), ("estimator_health", 1),
        ("rs485_read_failures", 1), ("rs485_write_failures", 1),
    ),
    Messages.PID_TRACE: (
        ("t_us", 1), ("target", 3), ("pose", 3), ("velocity_cmd", 3),
        ("wheel_requested", 3), ("wheel_cmd", 3), ("wheel_measured", 3),
        ("p_term", 3), ("i_term", 3), ("d_term", 3), ("flags", 1),
    ),
    Messages.LOG_TICK: (
        *_LOG_HEADER,
        # Estimator input of the tick, as handed to the estimator
        ("d", 3), ("dt_us", 1), ("encoder_age_us", 1), ("heading_rad", 1),
        ("imu_timestamp_us", 1), ("imu_age_us", 1), ("optical_timestamp_us", 1),
        ("optical_dx_mm", 1), ("optical_dy_mm", 1), ("optical_step_dt_us", 1),
        ("optical_vx_mm_s", 1), ("optical_vy_mm_s", 1), ("input_flags", 1),
        # Raw encoders
        ("enc_counts", 3), ("enc_t_us_mid", 1), ("enc_read_ok", 1),
        # Raw optical sensor, sensor axes: x, y, h
        ("otos_pos", 3), ("otos_vel", 3), ("otos_acc", 3),
        # Raw IMU
        ("imu_raw_yaw_rad", 1), ("imu_yaw_offset_rad", 1), ("imu_accuracy", 1),
        # Estimator output, table frame
        ("est_pose", 3), ("est_vel", 3), ("est_health", 1), ("est_flags", 1),
        # Commands
        ("cmd_count", 1), ("cmd_t_us", 1),
        # Cycle health
        ("work_us", 1), ("health_flags", 1), ("rs485_fail", 3),
        ("log_dropped", 1), ("otos_read_us", 1),
    ),
    Messages.LOG_EVENT: (*_LOG_HEADER, ("event", 1), ("value", 3), ("arg", 1)),
    Messages.LOG_FW_INFO: (
        ("schema_version", 1), ("git_describe", 1), ("build_date", 1),
    ),
    Messages.LOG_PARAMS: (
        ("schema_version", 1), ("estimator", 1), ("period_us", 1),
        # Geometry
        ("robot_radius_mm", 1), ("wheel_diameter_mm", 3), ("wheel_angle_rad", 3),
        ("counts_per_rev", 1),
        # Complementary filter
        ("alpha_encoders", 1), ("encoder_noise_counts", 1), ("imu_max_age_us", 1),
        ("optical_outlier_mm_s", 1), ("optical_noise_mm_s", 1),
        ("pure_rotation_min_rad", 1), ("pure_rotation_min_counts", 1),
        ("pure_rotation_tolerance", 1), ("use_encoders", 1),
        # Optical sensor
        ("otos_linear_scalar", 1), ("otos_angular_scalar", 1),
        ("otos_linear_scale_soft", 1), ("otos_angular_scale_soft", 1),
        ("otos_mount_angle_rad", 1),
        # Position PIDs: x, y, theta
        ("kp", 3), ("ki", 3), ("kd", 3),
        # Command limits
        ("max_wheel_rpm", 1), ("max_wheel_accel_rpm_s", 1),
    ),
}


def payload_size(message: Messages) -> int:
    """Size of the data of a message, ID byte excluded.

    Args:
        message (Messages): A message of ``PAYLOAD_FORMATS``.

    Returns:
        int: What a callback receives for this message.
    """
    return struct.calcsize(PAYLOAD_FORMATS[message])


def pack_message(message: Messages, *fields: float | int) -> bytes:
    """Builds a message: ID byte followed by its packed fields.

    Args:
        message (Messages): A message of ``PAYLOAD_FORMATS``.
        *fields: Its fields, in the order of the format.

    Returns:
        bytes: The payload to hand to ``Com.send_bytes``.
    """
    return message.to_bytes() + struct.pack(PAYLOAD_FORMATS[message], *fields)


def unpack_message(message: Messages, data: bytes) -> tuple:
    """Reads the fields of a received message.

    Args:
        message (Messages): A message of ``PAYLOAD_FORMATS``.
        data (bytes): Its data, ID byte excluded (what a callback receives).

    Returns:
        tuple: The fields, in the order of the format.

    Raises:
        struct.error: If ``data`` does not have the size of the format.
    """
    return struct.unpack(PAYLOAD_FORMATS[message], data)


def unpack_fields(message: Messages, data: bytes) -> dict[str, object]:
    """Reads a received message into a dict keyed by field name.

    Arrays come back as lists, fixed-size strings as ``str`` without their NUL
    padding, everything else as the number ``struct`` gives. The result holds
    only plain Python values, so it goes straight into ``json.dumps``.

    Args:
        message (Messages): A message of ``PAYLOAD_FIELDS``.
        data (bytes): Its data, ID byte excluded (what a callback receives).

    Returns:
        dict[str, object]: The fields, in the order of the struct.

    Raises:
        struct.error: If ``data`` does not have the size of the format.
    """
    values = struct.unpack(PAYLOAD_FORMATS[message], data)
    fields: dict[str, object] = {}
    index = 0
    for name, count in PAYLOAD_FIELDS[message]:
        if count == 1:
            value = values[index]
            if isinstance(value, bytes):
                value = value.split(b"\x00", 1)[0].decode("ascii", errors="replace")
            fields[name] = value
        else:
            fields[name] = list(values[index : index + count])
        index += count
    return fields
