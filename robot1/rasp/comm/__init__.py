"""Communication layer: execution mode and link to the Teensy.

The USB link itself lives in the usb_com package (common/), installed with
`pip install -e .` from the repository root. This module owns the robot-side
decisions: which mode is running, and which transport class that implies.
"""

from .context import (
    CONFIG_PATH,
    DUMMY_MODE,
    HARDWARE_MODE,
    VALID_MODES,
    create_com,
    get_com_class,
    get_com_config,
    get_mode,
    init_robot,
)

__all__ = [
    "CONFIG_PATH",
    "DUMMY_MODE",
    "HARDWARE_MODE",
    "VALID_MODES",
    "create_com",
    "get_com_class",
    "get_com_config",
    "get_mode",
    "init_robot",
]
