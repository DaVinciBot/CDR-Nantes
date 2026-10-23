"""USB communication utilities."""

from usb_com.python.com import Com
from usb_com.python.messages import (
    FRAME_DELIMITER,
    Messages,
    pack_message,
    payload_size,
    unpack_message,
)

__all__ = [
    "FRAME_DELIMITER",
    "Com",
    "Messages",
    "get_all_serial_number",
    "pack_message",
    "payload_size",
    "unpack_message",
]


def __getattr__(name):
    """Resolve get_all_serial_number on first use only (see python/__init__.py)."""
    if name == "get_all_serial_number":
        from usb_com.python.tools.get_serial_number import get_all_serial_number

        return get_all_serial_number
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
