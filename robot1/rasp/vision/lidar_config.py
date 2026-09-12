"""Single source of truth for the RPLidar A2M12 serial link.

Both LiDAR stacks read their port here, so they can never disagree on the
hardware again:

    detection.py     opponent detection, used by the match loop.
    localization.py  beacon extraction and SVD pose correction (frozen).

Resolution order, first match wins:

    1. the LIDAR_PORT environment variable,
    2. the "lidar_config" section of config.json,
    3. the defaults below (Raspberry Pi values).

A missing or malformed config.json is not fatal: the defaults target the
production hardware, and the resolved port is logged at import time so a wrong
value shows up in the match log instead of being silently wrong.
"""

import json
import logging
import os
from pathlib import Path

logger = logging.getLogger("LIDAR_CONFIG")

CONFIG_PATH = Path(__file__).resolve().parent.parent / "config.json"

# Raspberry Pi defaults. Only used when config.json says nothing.
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 256000
DEFAULT_TIMEOUT = 3


def _load_section() -> dict:
    """Return the "lidar_config" section, or an empty dict when unavailable."""
    try:
        with open(CONFIG_PATH, encoding="utf-8") as handle:
            return dict(json.load(handle).get("lidar_config", {}))
    except (OSError, ValueError) as exc:
        logger.warning(
            "config.json unreadable (%s), falling back to built-in LiDAR defaults.",
            exc,
        )
        return {}


_section = _load_section()

PORT = os.environ.get("LIDAR_PORT") or _section.get("port", DEFAULT_PORT)
BAUDRATE = int(_section.get("baudrate", DEFAULT_BAUDRATE))
TIMEOUT = int(_section.get("timeout", DEFAULT_TIMEOUT))

logger.info("LiDAR port: %s (%d bauds)", PORT, BAUDRATE)
