"""High-level USB communication utilities."""

from usb_com.python.com import Com
from usb_com.python.messages import END_BYTES_SIGNATURE, Messages

__all__ = [
    "END_BYTES_SIGNATURE",
    "Com",
    "Messages",
    "get_all_serial_number",
]


def __getattr__(name):
    """Resolve get_all_serial_number on first use only.

    tools/get_serial_number.py pulls in loggerplusplus, which is a dependency of
    that one helper, not of the USB link. Importing it eagerly here would make
    `from usb_com.python.com import Com` fail on any machine where only the
    runtime dependencies are installed.
    """
    if name == "get_all_serial_number":
        from usb_com.python.tools.get_serial_number import get_all_serial_number

        return get_all_serial_number
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
