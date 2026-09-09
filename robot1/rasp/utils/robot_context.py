#!/usr/bin/env python3
"""Robot communication bootstrap.

Hardware only: the USB link to the Teensy is the single supported transport.
The mode is explicit and never guessed, it is read from ROBOT_MODE:

    ROBOT_MODE=hardware (default) : open the real Teensy USB port
    ROBOT_MODE=dummy              : local loopback, for development without a robot

Failing to open the port raises. There is no silent fallback to dummy, so a
robot that cannot reach its Teensy never looks like a working robot.
"""

import logging
import os
import sys
from pathlib import Path

HARDWARE_MODE = "hardware"
DUMMY_MODE = "dummy"

# sim2d hook: add its name here, then dispatch on it in get_com_class().
VALID_MODES = (HARDWARE_MODE, DUMMY_MODE)


def get_mode() -> str:
    """Return the execution mode requested through ROBOT_MODE.

    Returns:
        str: one of VALID_MODES, defaulting to "hardware".

    Raises:
        ValueError: if ROBOT_MODE holds an unknown value. A typo must not
            silently degrade into hardware mode.
    """
    mode = os.environ.get("ROBOT_MODE", HARDWARE_MODE).strip().lower()
    if mode not in VALID_MODES:
        raise ValueError(
            f"Unknown ROBOT_MODE '{mode}'. Valid modes: {', '.join(VALID_MODES)}.",
        )
    return mode


def _get_loader():
    """Return the shared ModuleLoader, making robot1/rasp importable first.

    Kept lazy on purpose: importing loader instantiates ModuleLoader, which
    reads config.json and mutates sys.path. A plain "import utils" must not
    trigger that.
    """
    rasp_dir = str(Path(__file__).resolve().parent.parent)
    if rasp_dir not in sys.path:
        sys.path.insert(0, rasp_dir)

    from loader import loader

    return loader


def _get_effective_logger(logger=None):
    """Return a usable logger even when None is provided."""
    return logger if logger is not None else logging.getLogger("ROBOT_CONTEXT")


def get_com_config() -> dict:
    """Return the serial settings for the current mode.

    config.json ("serial_config") is the single source of truth for the Teensy
    identifiers. enable_dummy is derived from the mode and never read from the
    file, so that editing config.json cannot bring back a silent dummy robot.

    Returns:
        dict: serial_config, plus "mode" and an authoritative "enable_dummy".
    """
    mode = get_mode()
    config = dict(_get_loader().get_config("serial_config") or {})
    config["mode"] = mode
    config["enable_dummy"] = mode == DUMMY_MODE
    return config


def get_com_class():
    """Return the Com class matching the current mode.

    Single dispatch point for a future sim2d transport: return its own class
    here instead of the USB one.
    """
    return _get_loader().load_class("usb_com", "Com")


def create_com(logger=None):
    """Build the Com instance for the current mode.

    Args:
        logger: optional logger forwarded to Com.

    Returns:
        Com: ready-to-use instance, backed by a local loopback in dummy mode.

    Raises:
        ComError: when the Teensy port cannot be opened in hardware mode.
        KeyError: when config.json lacks a required serial_config entry.
    """
    config = get_com_config()
    com_class = get_com_class()

    return com_class(
        _get_effective_logger(logger),
        config["serial_number"],
        config["vid"],
        config["pid"],
        config.get("baudrate", 115200),
        enable_crc=config.get("enable_crc", True),
        enable_dummy=config["enable_dummy"],
    )


def init_robot(logger=None):
    """Open the robot communication link.

    Args:
        logger: optional logger.

    Returns:
        tuple: (com, mode) where mode is "HARDWARE" or "DUMMY".

    Example:
        from utils import init_robot
        com, mode = init_robot(logger)
    """
    effective_logger = _get_effective_logger(logger)
    mode = get_mode()

    effective_logger.info(f"Robot mode: {mode.upper()}")
    com = create_com(logger=logger)
    effective_logger.info("Communication link established")

    return com, mode.upper()


if __name__ == "__main__":
    print(f"Mode: {get_mode().upper()}")
    print(f"Configuration: {get_com_config()}")
