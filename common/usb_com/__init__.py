"""USB communication utilities."""

from usb_com.python.com import Com
from usb_com.python.com.com import TAP_RX, TAP_RX_REJECTED, TAP_TX
from usb_com.python.messages import (
    FRAME_DELIMITER,
    Messages,
    TraceMask,
    pack_message,
    payload_size,
    unpack_fields,
    unpack_message,
)

__all__ = [
    "FRAME_DELIMITER",
    "TAP_RX",
    "TAP_RX_REJECTED",
    "TAP_TX",
    "Com",
    "Messages",
    "TraceMask",
    "get_all_serial_number",
    "pack_message",
    "payload_size",
    "unpack_fields",
    "unpack_message",
]


def __getattr__(name):
    """Resolve get_all_serial_number on first use only (see python/__init__.py)."""
    if name == "get_all_serial_number":
        from usb_com.python.tools.get_serial_number import get_all_serial_number

        return get_all_serial_number
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
