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
        SET_TRACE: Turn the PID trace on or off.
        HEARTBEAT: Keep the Teensy watchdog fed.
        RESET_TEENSY: Reset the Teensy board.
        NACK: Notification of an invalid command.
        ODOMETRY: Pose and velocity estimated by the Teensy.
        SWITCH_STATE_RETURN: Report the state of a switch.
        STATUS: State of the rolling basis board.
        PID_TRACE: One line of PID trace per control tick.
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
    """Turn the PID trace on or off."""
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


class TraceFlag(IntFlag):
    """``PID_TRACE`` flags."""

    BRAKING = 1 << 0
    SPEED_SATURATED = 1 << 1
    ACCEL_LIMITED = 1 << 2
    INTEGRAL_FROZEN = 1 << 3


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
    # enabled (0 / 1)
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
