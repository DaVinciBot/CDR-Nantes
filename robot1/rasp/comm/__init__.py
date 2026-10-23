"""Communication layer: execution mode, link to the Teensy, and GPIO.

The USB link itself lives in the usb_com package (common/), installed with
`pip install -e .` from the repository root. This module owns the robot-side
decisions: which mode is running, which transport class that implies, and where
the GPIO pins come from.
"""

from .context import (
    CONFIG_PATH,
    DUMMY_MODE,
    HARDWARE_MODE,
    SIM2D_MODE,
    VALID_MODES,
    create_com,
    get_com_class,
    get_com_config,
    get_mode,
    init_robot,
)
from .gpio import FakeButton, get_button, set_fake_button
from .link import ComLink

__all__ = [
    "CONFIG_PATH",
    "DUMMY_MODE",
    "HARDWARE_MODE",
    "SIM2D_MODE",
    "VALID_MODES",
    "ComLink",
    "FakeButton",
    "create_com",
    "get_button",
    "get_com_class",
    "get_com_config",
    "get_mode",
    "init_robot",
    "set_fake_button",
]
