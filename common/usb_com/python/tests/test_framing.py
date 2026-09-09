"""Offline tests for the USB frame encoder/decoder.

No serial port and no Teensy required.

Run from the repository root::

    python -m unittest discover common/usb_com/python/tests
"""

from __future__ import annotations

import importlib.util
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

END_BYTES_SIGNATURE = _messages.END_BYTES_SIGNATURE
Messages = _messages.Messages
compute_crc8 = _framing.compute_crc8
decode_frame = _framing.decode_frame
encode_frame = _framing.encode_frame

SET_TARGET_POSITION_PAYLOAD = Messages.SET_TARGET_POSITION.to_bytes() + struct.pack(
    "<ddd",
    100.0,
    0.0,
    0.0,
)

# Same shape, but a non-zero message id (128) - see
# test_leading_zero_is_invisible_to_the_crc
UPDATE_ROLLING_BASIS_PAYLOAD = Messages.UPDATE_ROLLING_BASIS.to_bytes() + struct.pack(
    "<ddd",
    1234.5,
    -67.25,
    1.5707963267948966,
)


class TestRoundTrip(unittest.TestCase):
    """encode_frame / decode_frame must be exact inverses."""

    def test_set_target_position(self) -> None:
        frame = encode_frame(SET_TARGET_POSITION_PAYLOAD, enable_crc=True)
        decoded = decode_frame(frame, enable_crc=True)

        self.assertIsNotNone(decoded)
        message_id, data = decoded
        self.assertEqual(message_id, Messages.SET_TARGET_POSITION.value)
        self.assertEqual(struct.unpack("<ddd", data), (100.0, 0.0, 0.0))

    def test_without_crc(self) -> None:
        frame = encode_frame(SET_TARGET_POSITION_PAYLOAD, enable_crc=False)
        decoded = decode_frame(frame, enable_crc=False)

        self.assertIsNotNone(decoded)
        self.assertEqual(decoded[0], Messages.SET_TARGET_POSITION.value)

    def test_single_byte_payload(self) -> None:
        frame = encode_frame(Messages.NACK.to_bytes(), enable_crc=True)
        decoded = decode_frame(frame, enable_crc=True)

        self.assertEqual(decoded, (Messages.NACK.value, b""))

    def test_maximum_payload(self) -> None:
        payload = bytes(range(256))[:255]
        decoded = decode_frame(encode_frame(payload), enable_crc=True)

        self.assertEqual(decoded, (payload[0], payload[1:]))


class TestWireCompatibility(unittest.TestCase):
    """The wire format must NOT have changed: a patched Raspberry Pi has to
    keep talking to a Teensy that has not been reflashed.
    """

    def test_frame_layout(self) -> None:
        payload = SET_TARGET_POSITION_PAYLOAD
        frame = encode_frame(payload, enable_crc=True)

        # [ID][payload][size][CRC8][signature]
        self.assertEqual(frame[: len(payload)], payload)
        self.assertEqual(frame[len(payload)], len(payload))
        self.assertEqual(frame[len(payload) + 1], compute_crc8(payload + bytes([len(payload)])))
        self.assertEqual(frame[-4:], END_BYTES_SIGNATURE)
        self.assertEqual(len(frame), len(payload) + 6)

    def test_reference_vector(self) -> None:
        """Frozen byte-for-byte reference produced by the pre-fix code."""
        expected = bytes.fromhex(
            "00"                                      # SET_TARGET_POSITION
            "0000000000005940"                        # x = 100.0
            "0000000000000000"                        # y = 0.0
            "0000000000000000"                        # theta = 0.0
            "19"                                      # size = 25
            "c5"                                      # CRC8
            "badd1cc5",                               # end signature
        )
        self.assertEqual(encode_frame(SET_TARGET_POSITION_PAYLOAD), expected)

    def test_crc_matches_cpp_table(self) -> None:
        """Cross-check against the table-driven CRC8 of cpp/include/crc.h."""

        def cpp_digest(data: bytes) -> int:
            crc = 0
            for byte in data:
                index = crc ^ byte
                # Table entry = CRC-8 poly 0x07, init 0, no reflection
                value = index
                for _ in range(8):
                    value = ((value << 1) ^ 0x07) & 0xFF if value & 0x80 else (value << 1) & 0xFF
                crc = value
            return crc

        for payload in (b"\x00", b"\x80\x01\x02", SET_TARGET_POSITION_PAYLOAD):
            body = payload + bytes([len(payload)])
            self.assertEqual(compute_crc8(body), cpp_digest(body), payload.hex())


class TestMalformedFrames(unittest.TestCase):
    """decode_frame returns None instead of raising, so the receiver loop can
    simply skip a bad frame.
    """

    def test_empty(self) -> None:
        self.assertIsNone(decode_frame(b""))

    def test_too_short(self) -> None:
        self.assertIsNone(decode_frame(b"\xba\xdd\x1c\xc5"))
        self.assertIsNone(decode_frame(b"\x00\x01\xba\xdd\x1c\xc5"[:5]))

    def test_missing_signature(self) -> None:
        frame = encode_frame(SET_TARGET_POSITION_PAYLOAD)
        self.assertIsNone(decode_frame(frame[:-1]))

    def test_corrupted_crc(self) -> None:
        frame = bytearray(encode_frame(SET_TARGET_POSITION_PAYLOAD))
        frame[-5] ^= 0xFF  # flip the CRC byte
        self.assertIsNone(decode_frame(bytes(frame)))

    def test_corrupted_payload(self) -> None:
        frame = bytearray(encode_frame(SET_TARGET_POSITION_PAYLOAD))
        frame[3] ^= 0xFF
        self.assertIsNone(decode_frame(bytes(frame)))

    def test_declared_length_larger_than_received(self) -> None:
        """A size byte over-reporting the payload means a truncated frame."""
        payload = SET_TARGET_POSITION_PAYLOAD
        body = payload + bytes([len(payload) + 1])  # lie about the size
        frame = body + bytes([compute_crc8(body)]) + END_BYTES_SIGNATURE

        self.assertIsNone(decode_frame(frame))

    def test_zero_declared_length(self) -> None:
        """A zero length carries no message type byte."""
        body = b"\x00"
        frame = body + bytes([compute_crc8(body)]) + END_BYTES_SIGNATURE

        self.assertIsNone(decode_frame(frame))

    def test_wrong_length_is_caught_by_the_crc(self) -> None:
        """A size byte under-reporting the payload shifts the frame start, so
        the CRC no longer covers the same bytes and the frame is rejected.
        """
        payload = UPDATE_ROLLING_BASIS_PAYLOAD  # first byte 0x80, see below
        body = payload + bytes([len(payload) - 1])
        frame = body + bytes([compute_crc8(body)]) + END_BYTES_SIGNATURE

        self.assertIsNone(decode_frame(frame))

    def test_leading_zero_is_invisible_to_the_crc(self) -> None:
        """Documented blind spot, inherent to CRC-8 with init 0.

        Prefixing a message with 0x00 leaves the checksum unchanged, so a frame
        whose size byte under-reports by one CANNOT be detected when the byte
        being skipped is zero. Harmless in practice (a sender never lies about
        its own size), and it disappears with the COBS rework, but it must not
        be mistaken for a passing check.
        """
        payload = SET_TARGET_POSITION_PAYLOAD  # starts with 0x00
        body = payload + bytes([len(payload) - 1])
        frame = body + bytes([compute_crc8(body)]) + END_BYTES_SIGNATURE

        self.assertIsNotNone(decode_frame(frame))


class TestResynchronisation(unittest.TestCase):
    """read_until() returns everything up to the signature, leading noise
    included. The frame is anchored on its END, so a stray byte in front must
    not invalidate an otherwise valid frame.
    """

    def test_leading_noise_is_skipped(self) -> None:
        frame = encode_frame(SET_TARGET_POSITION_PAYLOAD)

        for noise in (b"\x00", b"\xff\xff", bytes(range(20))):
            decoded = decode_frame(noise + frame)
            self.assertEqual(
                decoded,
                (SET_TARGET_POSITION_PAYLOAD[0], SET_TARGET_POSITION_PAYLOAD[1:]),
                f"failed with {len(noise)} leading bytes",
            )

    def test_leading_noise_without_crc(self) -> None:
        frame = encode_frame(SET_TARGET_POSITION_PAYLOAD, enable_crc=False)
        decoded = decode_frame(b"\xde\xad" + frame, enable_crc=False)

        self.assertEqual(decoded[0], Messages.SET_TARGET_POSITION.value)

    def test_rejects_are_silent(self) -> None:
        """Fuzz-ish: no input may raise out of decode_frame."""
        for i in range(256):
            blob = bytes([i]) * i + END_BYTES_SIGNATURE
            decode_frame(blob)
            decode_frame(blob, enable_crc=False)


class TestEncodeGuards(unittest.TestCase):
    def test_empty_payload_rejected(self) -> None:
        with self.assertRaises(ValueError):
            encode_frame(b"")

    def test_oversized_payload_rejected(self) -> None:
        with self.assertRaises(ValueError):
            encode_frame(bytes(256))


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


class TestKnownLimitation(unittest.TestCase):
    """The end signature is still written raw, with no byte-stuffing."""

    @unittest.expectedFailure
    def test_payload_containing_the_signature(self) -> None:
        """KNOWN FAILURE - not fixed here, on purpose.

        A payload holding the four bytes BA DD 1C C5 truncates its own frame:
        read_until() on the Python side stops at the first occurrence. The fix
        is COBS / byte-stuffing, planned with the November firmware rework
        (TODO section 7) because it changes the wire format and requires
        reflashing both Teensy boards at the same time.
        """
        payload = Messages.SET_ODOMETRIE.to_bytes() + END_BYTES_SIGNATURE + b"\x00" * 4
        frame = encode_frame(payload)

        # What a reader splitting on the signature would actually hand us
        truncated = frame[: frame.index(END_BYTES_SIGNATURE) + 4]

        self.assertEqual(decode_frame(truncated), (payload[0], payload[1:]))


if __name__ == "__main__":
    unittest.main()
