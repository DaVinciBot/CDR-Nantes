"""Offline tests for the USB frame encoder/decoder and the message layouts.

No serial port and no Teensy required.

Run from the repository root::

    python -m unittest discover common/usb_com/python/tests
"""

from __future__ import annotations

import importlib.util
import random
import struct
import sys
import threading
import types
import unittest
from pathlib import Path

_PYTHON_DIR = Path(__file__).resolve().parents[1]


def _load_module(name: str, path: Path) -> types.ModuleType:
    """Loads a single module from its file, without importing its packages.

    Importing ``usb_com`` normally runs its ``__init__.py`` chain, which pulls
    in ``loggerplusplus`` and ``pyserial``. Those belong on the robot, not in a
    protocol test: framing.py and messages.py have no dependency beyond
    ``crc8``, so they are loaded directly here. Empty namespace modules are
    registered first so the absolute imports inside framing.py resolve.
    """
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


for _package in ("usb_com", "usb_com.python", "usb_com.python.com"):
    sys.modules.setdefault(_package, types.ModuleType(_package))

_messages = _load_module("usb_com.python.messages", _PYTHON_DIR / "messages.py")
_framing = _load_module(
    "usb_com.python.com.framing",
    _PYTHON_DIR / "com" / "framing.py",
)
_dummy = _load_module("usb_com.python.com.dummy", _PYTHON_DIR / "com" / "dummy.py")

FRAME_DELIMITER = _messages.FRAME_DELIMITER
MAX_PAYLOAD_SIZE = _messages.MAX_PAYLOAD_SIZE
Messages = _messages.Messages
PAYLOAD_FORMATS = _messages.PAYLOAD_FORMATS
pack_message = _messages.pack_message
unpack_message = _messages.unpack_message
compute_crc8 = _framing.compute_crc8
cobs_encode = _framing.cobs_encode
cobs_decode = _framing.cobs_decode
decode_frame = _framing.decode_frame
encode_frame = _framing.encode_frame
DummySerial = _dummy.DummySerial

SET_TARGET_PAYLOAD = pack_message(Messages.SET_TARGET, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0)
ODOMETRY_PAYLOAD = pack_message(
    Messages.ODOMETRY, 1234.5, -67.25, 1.5707964, 0.0, 200.0, 0.0, 123456789,
)


def split_stream(stream: bytes) -> list[bytes]:
    """Cuts a byte stream into frames the way read_until(FRAME_DELIMITER) does."""
    frames = []
    start = 0
    while True:
        end = stream.find(FRAME_DELIMITER, start)
        if end < 0:
            return frames
        frames.append(stream[start : end + 1])
        start = end + 1


class TestCobs(unittest.TestCase):
    """COBS encoding: reference vectors, and no 0x00 ever inside a frame."""

    def test_reference_vectors(self) -> None:
        vectors = {
            b"\x00": b"\x01\x01",
            b"\x00\x00": b"\x01\x01\x01",
            b"\x00\x11\x00": b"\x01\x02\x11\x01",
            b"\x11\x22\x00\x33": b"\x03\x11\x22\x02\x33",
            b"\x11\x22\x33\x44": b"\x05\x11\x22\x33\x44",
            b"\x11\x00\x00\x00": b"\x02\x11\x01\x01\x01",
        }
        for raw, encoded in vectors.items():
            self.assertEqual(cobs_encode(raw), encoded, raw.hex())
            self.assertEqual(cobs_decode(encoded), raw, raw.hex())

    def test_full_block(self) -> None:
        """254 non-zero bytes: one full block, then a trailing code byte (the
        C++ encoder does the same). The minimal form without it decodes too.
        """
        raw = bytes(range(1, 255))
        encoded = cobs_encode(raw)
        self.assertEqual(encoded, b"\xff" + raw + b"\x01")
        self.assertEqual(cobs_decode(encoded), raw)
        self.assertEqual(cobs_decode(b"\xff" + raw), raw)

    def test_no_zero_inside_and_round_trip(self) -> None:
        rng = random.Random(1)
        for _ in range(2000):
            raw = bytes(rng.choice((0, 0, rng.randrange(256))) for _ in range(rng.randrange(300)))
            encoded = cobs_encode(raw)
            self.assertNotIn(0, encoded)
            self.assertLessEqual(len(encoded), len(raw) + len(raw) // 254 + 1)
            self.assertEqual(cobs_decode(encoded), raw)

    def test_invalid_input(self) -> None:
        self.assertIsNone(cobs_decode(b"\x00"))  # zero code
        self.assertIsNone(cobs_decode(b"\x03\x11\x00"))  # zero inside a block
        self.assertIsNone(cobs_decode(b"\x05\x11\x22"))  # block past the end


class TestRoundTrip(unittest.TestCase):
    """encode_frame / decode_frame must be exact inverses."""

    def test_set_target(self) -> None:
        decoded = decode_frame(encode_frame(SET_TARGET_PAYLOAD))
        self.assertIsNotNone(decoded)
        message_id, data = decoded
        self.assertEqual(message_id, Messages.SET_TARGET.value)
        self.assertEqual(unpack_message(Messages.SET_TARGET, data), (100.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_without_crc(self) -> None:
        decoded = decode_frame(encode_frame(SET_TARGET_PAYLOAD, enable_crc=False), enable_crc=False)
        self.assertEqual(decoded, (SET_TARGET_PAYLOAD[0], SET_TARGET_PAYLOAD[1:]))

    def test_single_byte_payload(self) -> None:
        decoded = decode_frame(encode_frame(Messages.NACK.to_bytes()))
        self.assertEqual(decoded, (Messages.NACK.value, b""))

    def test_maximum_payload(self) -> None:
        payload = bytes(range(256))[:MAX_PAYLOAD_SIZE]
        self.assertEqual(decode_frame(encode_frame(payload)), (payload[0], payload[1:]))

    def test_frame_has_a_single_delimiter_at_its_end(self) -> None:
        frame = encode_frame(ODOMETRY_PAYLOAD)
        self.assertTrue(frame.endswith(FRAME_DELIMITER))
        self.assertEqual(frame.count(FRAME_DELIMITER), 1)

    def test_payload_containing_the_old_signature(self) -> None:
        """Was a known failure before COBS: the four bytes BA DD 1C C5 used to
        end the frame, and a payload holding them cut its own frame short.
        """
        payload = Messages.SET_ODOMETRIE.to_bytes() + b"\xba\xdd\x1c\xc5" + b"\x00" * 4
        frames = split_stream(encode_frame(payload))
        self.assertEqual(len(frames), 1)
        self.assertEqual(decode_frame(frames[0]), (payload[0], payload[1:]))


class TestWireCompatibility(unittest.TestCase):
    """Reference frames produced by the C++ side (cobs.cpp + crc.cpp, built on
    a PC with the packed structs of messages.h). Both sides must produce and
    accept exactly these bytes.
    """

    CPP_FRAMES = {
        "set_target": "01010103c8420101010101010101010101010101010101010102ab00",
        "odometry": "028004509a44088086c2db0fc93f01010101010348430101010615cd5b071000",
        "signature": "0602badd1cc5010101023e00",
    }

    def test_set_target_matches_cpp(self) -> None:
        self.assertEqual(encode_frame(SET_TARGET_PAYLOAD).hex(), self.CPP_FRAMES["set_target"])

    def test_odometry_matches_cpp(self) -> None:
        self.assertEqual(encode_frame(ODOMETRY_PAYLOAD).hex(), self.CPP_FRAMES["odometry"])
        message_id, data = decode_frame(bytes.fromhex(self.CPP_FRAMES["odometry"]))
        self.assertEqual(message_id, Messages.ODOMETRY.value)
        fields = unpack_message(Messages.ODOMETRY, data)
        self.assertEqual(fields[0], 1234.5)
        self.assertEqual(fields[6], 123456789)

    def test_signature_payload_matches_cpp(self) -> None:
        payload = Messages.SET_ODOMETRIE.to_bytes() + b"\xba\xdd\x1c\xc5" + b"\x00" * 4
        self.assertEqual(encode_frame(payload).hex(), self.CPP_FRAMES["signature"])

    def test_full_block_matches_cpp(self) -> None:
        payload = bytes(range(255))  # ID 0, then 1..254: a zero, then a full block
        expected = "01ff" + bytes(range(1, 255)).hex() + "02d200"
        self.assertEqual(encode_frame(payload).hex(), expected)

    def test_crc_matches_cpp_table(self) -> None:
        """Cross-check against the table-driven CRC8 of cpp/include/crc.h."""

        def cpp_digest(data: bytes) -> int:
            crc = 0xFF  # initial value
            for byte in data:
                value = crc ^ byte
                # Table entry = CRC-8 poly 0x07, init 0, no reflection
                for _ in range(8):
                    value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
                crc = value
            return crc

        for payload in (b"\x00", b"\x80\x01\x02", SET_TARGET_PAYLOAD, ODOMETRY_PAYLOAD):
            self.assertEqual(compute_crc8(payload), cpp_digest(payload), payload.hex())


class TestMessageLayouts(unittest.TestCase):
    """PAYLOAD_FORMATS against the packed structs of messages.h."""

    # sizeof() of each struct on the C++ side, ID byte included
    CPP_SIZES = {
        Messages.SET_TARGET: 25,
        Messages.SET_PID: 14,
        Messages.SET_ODOMETRIE: 25,
        Messages.STOP: 1,
        Messages.SET_ESTIMATOR: 2,
        Messages.SET_TRACE: 2,
        Messages.HEARTBEAT: 1,
        Messages.ODOMETRY: 29,
        Messages.STATUS: 17,
        Messages.PID_TRACE: 114,
    }

    def test_sizes_match_the_cpp_structs(self) -> None:
        for message, size in self.CPP_SIZES.items():
            self.assertEqual(1 + struct.calcsize(PAYLOAD_FORMATS[message]), size, message.name)

    def test_every_message_fits_in_a_frame(self) -> None:
        for message, fmt in PAYLOAD_FORMATS.items():
            self.assertLessEqual(1 + struct.calcsize(fmt), MAX_PAYLOAD_SIZE, message.name)

    def test_pack_unpack_round_trip(self) -> None:
        status = (42, 0b10110001, 1, 3, 7, 0)
        payload = pack_message(Messages.STATUS, *status)
        self.assertEqual(payload[0], Messages.STATUS.value)
        self.assertEqual(unpack_message(Messages.STATUS, payload[1:]), status)
        self.assertEqual(pack_message(Messages.HEARTBEAT), Messages.HEARTBEAT.to_bytes())


class TestMalformedFrames(unittest.TestCase):
    """decode_frame returns None instead of raising, so the receiver loop can
    simply skip a bad frame.
    """

    def test_empty(self) -> None:
        self.assertIsNone(decode_frame(b""))
        self.assertIsNone(decode_frame(FRAME_DELIMITER))

    def test_too_short(self) -> None:
        # One decoded byte: an ID without its CRC
        self.assertIsNone(decode_frame(cobs_encode(b"\x01") + FRAME_DELIMITER))

    def test_corrupted_crc(self) -> None:
        frame = bytearray(encode_frame(SET_TARGET_PAYLOAD))
        frame[-2] ^= 0x01  # last encoded byte = the CRC here
        self.assertIsNone(decode_frame(bytes(frame)))

    def test_corrupted_payload(self) -> None:
        frame = bytearray(encode_frame(ODOMETRY_PAYLOAD))
        frame[3] ^= 0x10
        self.assertIsNone(decode_frame(bytes(frame)))

    def test_truncated_frame(self) -> None:
        frame = encode_frame(ODOMETRY_PAYLOAD)
        self.assertIsNone(decode_frame(frame[:10] + FRAME_DELIMITER))

    def test_rejects_are_silent(self) -> None:
        """Fuzz-ish: no input may raise out of decode_frame."""
        rng = random.Random(2)
        for _ in range(3000):
            blob = bytes(rng.randrange(256) for _ in range(rng.randrange(40)))
            decode_frame(blob)
            decode_frame(blob, enable_crc=False)


class TestResynchronisation(unittest.TestCase):
    """The delimiter always marks a frame boundary: after noise or a cut frame,
    the stream is back in sync at the next 0x00.
    """

    def test_noise_costs_one_frame_at_most(self) -> None:
        good = encode_frame(SET_TARGET_PAYLOAD)
        for noise in (b"\x01", b"\xff\xfe", bytes(range(1, 30))):
            frames = split_stream(noise + good + good)
            decoded = [decode_frame(f) for f in frames]
            # The noise sticks to the first frame and spoils it; the next is fine.
            self.assertIsNone(decoded[0], noise.hex())
            self.assertEqual(decoded[1], (SET_TARGET_PAYLOAD[0], SET_TARGET_PAYLOAD[1:]))

    def test_leading_zero_is_seen_by_the_crc(self) -> None:
        """A stray 0x01 decodes as a leading 0x00. With a CRC initialised at 0
        it went unnoticed and the frame passed as ID 0; init 0xFF catches it.
        """
        body = SET_TARGET_PAYLOAD + bytes([compute_crc8(SET_TARGET_PAYLOAD)])
        self.assertIsNone(decode_frame(cobs_encode(b"\x00" + body) + FRAME_DELIMITER))

    def test_cut_frame_then_good_frame(self) -> None:
        good = encode_frame(ODOMETRY_PAYLOAD)
        frames = split_stream(good[:12] + FRAME_DELIMITER + good)
        self.assertIsNone(decode_frame(frames[0]))
        self.assertEqual(decode_frame(frames[1]), (ODOMETRY_PAYLOAD[0], ODOMETRY_PAYLOAD[1:]))


class TestDummySerial(unittest.TestCase):
    """read_until on the dummy port behaves like pyserial's: it consumes."""

    def test_read_until_consumes_up_to_the_delimiter(self) -> None:
        port = DummySerial()
        a = encode_frame(SET_TARGET_PAYLOAD)
        b = encode_frame(ODOMETRY_PAYLOAD)
        port.dummy_add_input(a + b[:5])
        self.assertEqual(port.read_until(FRAME_DELIMITER), a)
        self.assertEqual(port.read_until(FRAME_DELIMITER), b[:5])  # timeout, partial
        port.dummy_add_input(b[5:])
        self.assertEqual(port.read_until(FRAME_DELIMITER), b[5:])
        self.assertEqual(port.read_until(FRAME_DELIMITER), b"")


class TestEncodeGuards(unittest.TestCase):
    def test_empty_payload_rejected(self) -> None:
        with self.assertRaises(ValueError):
            encode_frame(b"")

    def test_oversized_payload_rejected(self) -> None:
        with self.assertRaises(ValueError):
            encode_frame(bytes(MAX_PAYLOAD_SIZE + 1))


class TestConcurrency(unittest.TestCase):
    """Non-regression for the shared crc8 instance.

    The previous code kept one mutable crc8 object on the Com instance, used by
    the receiver thread and by send_bytes() at the same time: a reset() landing
    inside another thread's update() produced a wrong CRC, hence NACKs on
    perfectly valid frames.
    """

    def test_parallel_encode_decode(self) -> None:
        errors: list[str] = []
        barrier = threading.Barrier(8)

        def worker(seed: int) -> None:
            payload = bytes([seed]) + struct.pack("<ddd", float(seed), 1.5, -2.5)
            barrier.wait()
            for _ in range(500):
                decoded = decode_frame(encode_frame(payload))
                if decoded != (payload[0], payload[1:]):
                    errors.append(f"thread {seed}: {decoded!r}")
                    return

        threads = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        self.assertEqual(errors, [])


if __name__ == "__main__":
    unittest.main()
