"""The transport contract the match loop actually needs.

app.py uses three methods of usb_com.Com and nothing else. Writing that down is
what makes a substitute possible without having to read app.py first: sim2d's
transport implements ComLink, and so does the real Com.

The constructor is deliberately NOT part of the protocol. create_com() builds
every transport with the USB signature (serial number, vid, pid, baudrate), so
a simulated transport accepts those arguments and ignores them. That is the
price of keeping a single build path, and it is cheaper than maintaining two.
"""

from typing import Callable, Protocol, runtime_checkable


@runtime_checkable
class ComLink(Protocol):
    """Minimal contract between the Raspberry Pi and the Teensy."""

    def add_callback(self, func: Callable[[bytes], None], iid: int) -> None:
        """Register func as the handler for incoming messages of id iid."""

    def send_bytes(self, data: bytes) -> None:
        """Send one already framed message."""

    def read_bytes(self) -> bytes:
        """Return the bytes received since the previous call."""
