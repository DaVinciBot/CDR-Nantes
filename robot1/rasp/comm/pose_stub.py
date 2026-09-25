#!/usr/bin/env python3
"""Pose source stub: a moving pose without a robot, an EKF or a Teensy.

Why this exists: the high-level work is delegated, and the pose frame contract is
the interface between the two people. This stub lets the high level be developed
against that contract from day one. The 2026 lesson is that a brick which is
theoretically correct but never wired into the running chain does not count.

What it emits: ``UPDATE_ROLLING_BASIS`` (128) in the EXISTING wire format, three
doubles for x, y and theta in mm and rad. Nothing in ``common/usb_com/`` is
modified, and ``app.py`` needs no change either: it already consumes the pose
through a callback.

How it works: frames are built with the real ``encode_frame`` and pushed into a
minimal serial stand-in, so the actual receiver thread, the actual
``decode_frame`` and the actual callback dispatch all run. The stub exercises the
real decoder rather than imitating it, which is the only way it can catch a
framing regression.

There is no sigma field yet. It only means something once the EKF runs with a
tuned Q (planned for January), and an unvalidated sigma is worse than none
because it gets trusted. When it comes back, this stub is where the high level
will first see it.

Trajectory: generated, not replayed. Log replay is deliberately left out: the
JSONL log format will be defined once, by ``sim2d``, and shared between the log,
the control page and Rerun. Defining a second format here would guarantee a
mismatch. Replay plugs into ``pose_at()`` once that format exists.

Usage:
    ROBOT_MODE=pose_stub python -m telemetry.rerun_bridge
"""

from __future__ import annotations

import math
import struct
import threading
import time
from typing import TYPE_CHECKING

from usb_com.python.com import Com
from usb_com.python.com.framing import encode_frame
from usb_com.python.messages import Messages

if TYPE_CHECKING:
    from loggerplusplus import Logger

#: Value of ROBOT_MODE that selects this transport.
POSE_STUB_MODE = "pose_stub"

#: Bytes of an UPDATE_ROLLING_BASIS payload: three little-endian doubles.
_POSE_STRUCT = "<ddd"


class _StubSerial:
    """Minimal stand-in for a pyserial port, feeding the real receiver thread.

    ``DummySerial`` cannot be used here, and the reason is worth writing down:
    its ``read_until()`` returns ``input_buffer.split(signature)[0]``, which
    strips the end signature that ``decode_frame()`` requires and never consumes
    the buffer. Every read would return the same headless frame forever. It is
    usable to inspect what was written, not to feed a stream.

    ``read_until()`` here returns ``b""`` when no complete frame is buffered. The
    receiver loop already guards against an empty read with a short sleep, so
    that is a supported answer and not an error.
    """

    def __init__(self) -> None:
        """Initializes empty buffers and their lock."""
        self._input = bytearray()
        self._lock = threading.Lock()
        #: What the robot sent us, kept so outgoing messages do not crash.
        self.output_buffer = b""
        #: Always False: writes complete synchronously. Com busy-waits on this.
        self.out_waiting = False

    def feed(self, data: bytes) -> None:
        """Appends bytes as if they had arrived from the Teensy.

        Args:
            data (bytes): A complete, already framed message.
        """
        with self._lock:
            self._input += data

    def write(self, data: bytes) -> None:
        """Absorbs an outgoing frame.

        The robot really does send messages in this mode (SET_ODOMETRIE at
        startup, targets, PID updates). They are recorded rather than answered:
        this stub is a pose source, not a firmware simulator.

        Args:
            data (bytes): The frame written by Com.
        """
        with self._lock:
            self.output_buffer += data

    def read_until(self, signature: bytes) -> bytes:
        """Consumes and returns the next complete frame, signature included.

        Args:
            signature (bytes): The end-of-frame signature to look for.

        Returns:
            bytes: The frame up to and including the signature, or ``b""`` when
            no complete frame is buffered yet.
        """
        with self._lock:
            index = self._input.find(signature)
            if index < 0:
                return b""
            end = index + len(signature)
            frame = bytes(self._input[:end])
            del self._input[:end]
            return frame

    def reset_input_buffer(self) -> None:
        """Drops everything not yet read."""
        with self._lock:
            self._input.clear()

    def reset_output_buffer(self) -> None:
        """Drops everything the robot has written."""
        with self._lock:
            self.output_buffer = b""


class PoseStubCom(Com):
    """A Com whose Teensy is replaced by a generated pose.

    The constructor signature matches what ``context.create_com()`` passes, so
    this class is a drop-in replacement returned by ``get_com_class()``.

    The defaults describe a circle walked at constant speed, tangent heading, on
    the 3000 x 2000 mm table. They are class attributes so a caller can subclass
    or override them without touching this module.
    """

    #: Frames per second. The real firmware publishes its pose at 20 to 50 Hz.
    PUBLISH_HZ = 50.0

    #: Centre of the generated circle, in mm.
    CIRCLE_CENTER_MM = (1500.0, 1000.0)
    #: Radius of the generated circle, in mm.
    CIRCLE_RADIUS_MM = 400.0
    #: Time for one full turn, in seconds.
    CIRCLE_PERIOD_S = 20.0

    def __init__(
        self,
        logger: Logger,
        serial_number: int,
        vid: int,
        pid: int,
        baudrate: int,
        *,
        enable_crc: bool = True,
        enable_dummy: bool = False,
    ) -> None:
        """Builds the stub link and starts generating poses.

        Args:
            logger (Logger): Logger forwarded to Com.
            serial_number (int): Ignored, no port is opened.
            vid (int): Ignored.
            pid (int): Ignored.
            baudrate (int): Ignored.
            enable_crc (bool, optional): Frame the poses with a CRC8, exactly as
                the firmware does. Defaults to ``True``.
            enable_dummy (bool, optional): Ignored, see below. Defaults to
                ``False``.
        """
        # Must exist before Com.__init__ runs, because it calls _get_serial().
        self._stub_serial = _StubSerial()

        # enable_dummy is forced off on purpose. Com._start_receiver() returns
        # None in dummy mode, so the receiver thread never starts and no callback
        # is ever dispatched: a stub built that way would be silently dead. This
        # mode is not dummy mode, it is a transport of its own.
        super().__init__(
            logger,
            serial_number,
            vid,
            pid,
            baudrate,
            enable_crc=enable_crc,
            enable_dummy=False,
        )

        self._stop_event = threading.Event()
        self._producer_thread = threading.Thread(
            target=self._produce,
            name="PoseStubProducer",
            daemon=True,
        )
        self._producer_thread.start()

        self.logger.info(
            f"Pose stub: {self.PUBLISH_HZ:.0f} Hz, circle r="
            f"{self.CIRCLE_RADIUS_MM:.0f} mm, period {self.CIRCLE_PERIOD_S:.0f} s",
        )

    def _get_serial(self) -> _StubSerial:
        """Returns the stub port instead of scanning for a real device.

        Returns:
            _StubSerial: The in-memory port this instance feeds.
        """
        return self._stub_serial

    def _start_receiver(self) -> threading.Thread:
        """Starts Com's own receiver, but as a daemon thread.

        Same target as the parent, so the real decoding and dispatch path is the
        one that runs. The only change is ``daemon=True``.

        Com's receiver loops forever and has no stop flag, and the parent starts
        it as a non-daemon thread: a process using this stub would therefore hang
        on exit, Ctrl-C included. That gap is a known limitation of Com (no
        serial timeout, no stop flag on the receiver thread) and is not fixed
        here, because ``common/usb_com/`` is out of scope for this
        step and is shared with the hardware path. Making the copy a daemon
        keeps the stub usable without touching any of that.

        Returns:
            threading.Thread: The running receiver thread.
        """
        # Name-mangled private method of Com. Ugly, and deliberately so: it makes
        # the borrowing visible instead of duplicating the receive loop here,
        # which would defeat the point of exercising the real decoder.
        receiver = threading.Thread(
            target=self._Com__receiver,  # noqa: SLF001
            name="PoseStubReceiver",
            daemon=True,
        )
        receiver.start()
        return receiver

    def pose_at(self, elapsed_s: float) -> tuple[float, float, float]:
        """Returns the generated pose at a given time.

        Override this to replay a log once the sim2d log format exists.

        Args:
            elapsed_s (float): Seconds since the stub started.

        Returns:
            tuple[float, float, float]: x in mm, y in mm, theta in rad,
            normalized to ``[-pi, pi)``.
        """
        angle = 2.0 * math.pi * elapsed_s / self.CIRCLE_PERIOD_S
        center_x, center_y = self.CIRCLE_CENTER_MM

        x = center_x + self.CIRCLE_RADIUS_MM * math.cos(angle)
        y = center_y + self.CIRCLE_RADIUS_MM * math.sin(angle)

        # Heading tangent to the circle, so the pose reads like a robot actually
        # following it rather than a point sliding sideways.
        theta = angle + math.pi / 2.0

        # Normalize to [-pi, pi), the convention used everywhere in the system.
        theta = (theta + math.pi) % (2.0 * math.pi) - math.pi

        return x, y, theta

    def _produce(self) -> None:
        """Feeds one framed pose per period until stop() is called."""
        period = 1.0 / self.PUBLISH_HZ
        started_at = time.monotonic()
        next_at = started_at

        while not self._stop_event.is_set():
            now = time.monotonic()
            if now < next_at:
                # wait() rather than sleep(): stop() takes effect immediately.
                self._stop_event.wait(next_at - now)
                continue

            # Advance the schedule by whole periods so a late wake-up does not
            # make the pose drift in time; if we fall behind by more than one
            # period, skip ahead instead of bursting.
            next_at = max(next_at + period, now)

            x, y, theta = self.pose_at(now - started_at)
            payload = Messages.UPDATE_ROLLING_BASIS.to_bytes() + struct.pack(
                _POSE_STRUCT,
                x,
                y,
                theta,
            )
            self._stub_serial.feed(encode_frame(payload, enable_crc=self.enable_crc))

    def stop(self) -> None:
        """Stops generating poses.

        The receiver thread has no stop flag of its own (see _start_receiver),
        but it is a daemon here, so it does not hold the process open.
        """
        self._stop_event.set()
        self._producer_thread.join(timeout=1.0)
