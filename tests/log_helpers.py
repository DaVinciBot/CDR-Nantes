"""Synthetic runs for the tests of the run log and of the offline tools.

No robot and no serial port: the Teensy is played by messages built with the
protocol itself, pushed through the tap a real link uses, into a real
RunLogger. The folder that comes out is a run folder like any other.
"""

from usb_com import TAP_RX, TAP_TX, Messages, TraceMask, pack_message
from usb_com.python.messages import (
    LOG_SCHEMA_VERSION,
    PAYLOAD_FIELDS,
    PAYLOAD_FORMATS,
)

from robot1.rasp.telemetry.logger import RunLogger

TICK_US = 15_000


class FakeClock:
    """Time that moves only when told to, in nanoseconds."""

    def __init__(self):
        self.t = 0

    def __call__(self):
        return self.t


class FakeCom:
    """The least a transport is: a tap, and a record of what was sent."""

    def __init__(self):
        self.tap = None
        self.sent = []

    def add_callback(self, func, iid):
        pass

    def read_bytes(self):
        return b""

    def set_tap(self, tap):
        self.tap = tap

    def send_bytes(self, data):
        self.sent.append(bytes(data))
        if self.tap is not None:
            self.tap(TAP_TX, data)

    def receive(self, payload):
        """Deliver one message as if the Teensy had sent it."""
        self.tap(TAP_RX, payload)


# ── Messages, by field name ───────────────────────────────────────────────

def pack_named(message: Messages, **fields) -> bytes:
    """Pack a message from its field names; fields not given are zero.

    An array field takes a sequence of its length.
    """
    values = []
    for name, count in PAYLOAD_FIELDS[message]:
        value = fields.pop(name, None)
        if count == 1:
            values.append(0 if value is None else value)
        else:
            values.extend([0] * count if value is None else list(value))
    if fields:
        raise KeyError(f"unknown fields for {message.name}: {sorted(fields)}")
    # Integer fields must get integers, struct refuses a float.
    codes = [code for code in _codes(PAYLOAD_FORMATS[message])]
    values = [int(value) if code in "bBhHiIqQ" else value
              for code, value in zip(codes, values)]
    return pack_message(message, *values)


def _codes(fmt: str) -> list[str]:
    """One struct code per value of a format ("<3fI" -> f, f, f, I)."""
    codes = []
    count = ""
    for char in fmt.lstrip("<>=!@"):
        if char.isdigit():
            count += char
            continue
        if char == "s":
            codes.append("s")
        else:
            codes.extend(char * int(count or 1))
        count = ""
    return codes


def log_tick(seq: int, tick: int, t_us: int, **fields) -> bytes:
    """A LOG_TICK; fields not given are zero."""
    return pack_named(Messages.LOG_TICK, schema_version=LOG_SCHEMA_VERSION,
                      seq=seq & 0xFFFF, tick=tick, t_us=t_us & 0xFFFFFFFF,
                      **fields)


def log_event(seq: int, tick: int, t_us: int, event: int, **fields) -> bytes:
    """A LOG_EVENT."""
    return pack_named(Messages.LOG_EVENT, schema_version=LOG_SCHEMA_VERSION,
                      seq=seq & 0xFFFF, tick=tick, t_us=t_us & 0xFFFFFFFF,
                      event=event, **fields)


def pid_trace(t_us: int, **fields) -> bytes:
    """A PID_TRACE; fields not given are zero."""
    return pack_named(Messages.PID_TRACE, t_us=t_us & 0xFFFFFFFF, **fields)


def odometry(t_us: int, x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0,
             omega=0.0) -> bytes:
    """An ODOMETRY."""
    return pack_message(Messages.ODOMETRY, x, y, theta, vx, vy, omega,
                        t_us & 0xFFFFFFFF)


def set_target(x, y, theta, vx=0.0, vy=0.0, omega=0.0) -> bytes:
    """A SET_TARGET."""
    return pack_message(Messages.SET_TARGET, x, y, theta, vx, vy, omega)


# ── A whole run ───────────────────────────────────────────────────────────

class SyntheticRun:
    """Messages and lines dated on the clock of the Pi, then written.

    Everything is queued with its time of the Pi and played in that order at
    close(), so a test can add the streams one after the other.
    """

    def __init__(self, root, *, name="test", mode="dummy",
                 trace_mask=TraceMask.PID_TRACE | TraceMask.LOG_TICK,
                 meta=None):
        self.clock = FakeClock()
        self.com = FakeCom()
        self.log = RunLogger(root, name, mode=mode, clock_ns=self.clock,
                             clock_name="fake", lossless=True,
                             trace_mask=trace_mask, meta=meta)
        self.log.attach(self.com)
        self._events = []

    def receive(self, t_rasp_s: float, payload: bytes) -> None:
        """A message of the Teensy, received at t_rasp_s."""
        self._events.append((t_rasp_s, len(self._events), "rx", payload))

    def send(self, t_rasp_s: float, payload: bytes) -> None:
        """A message to the Teensy, sent at t_rasp_s."""
        self._events.append((t_rasp_s, len(self._events), "tx", payload))

    def emit(self, t_rasp_s: float, stream: str, **fields) -> None:
        """A line of the Pi, at t_rasp_s."""
        self._events.append((t_rasp_s, len(self._events), "emit",
                             (stream, fields)))

    def close(self):
        """Play everything in time order, close the log, return the folder."""
        for t_rasp_s, _, kind, item in sorted(self._events,
                                              key=lambda e: e[:2]):
            self.clock.t = int(round(t_rasp_s * 1e9))
            if kind == "rx":
                self.com.receive(item)
            elif kind == "tx":
                self.com.send_bytes(item)
            else:
                self.log.emit(item[0], **item[1])
        self.clock.t += 1
        self.log.close()
        return self.log.folder
