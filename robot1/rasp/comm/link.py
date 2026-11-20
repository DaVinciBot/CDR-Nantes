"""The transport contract the match loop actually needs.

app.py uses three methods of usb_com.Com and nothing else. Writing that down is
what makes a substitute possible without having to read app.py first: sim2d's
transport implements ComLink, and so does the real Com.

set_tap() is the fourth: the run log listens to the link through it, the same
way on every transport, so a simulated match and a real one leave the same
files behind.

The constructor is deliberately NOT part of the protocol. create_com() builds
every transport with the USB signature (serial number, vid, pid, baudrate), so
a simulated transport accepts those arguments and ignores them. That is the
price of keeping a single build path, and it is cheaper than maintaining two.
"""

from typing import Callable, Optional, Protocol, runtime_checkable


@runtime_checkable
class ComLink(Protocol):
    """Minimal contract between the Raspberry Pi and the Teensy."""

    def add_callback(self, func: Callable[[bytes], None], iid: int) -> None:
        """Register func as the handler for incoming messages of id iid."""

    def send_bytes(self, data: bytes) -> None:
        """Send one already framed message."""

    def read_bytes(self) -> bytes:
        """Return the bytes received since the previous call."""

    def set_tap(self, tap: Optional[Callable[[int, bytes], None]]) -> None:
        """Register a listener on every message, both ways, or None to remove it.

        The tap receives a direction (usb_com.TAP_RX, TAP_TX or
        TAP_RX_REJECTED) and the message as [ID][data]. It must return at once.
        """
