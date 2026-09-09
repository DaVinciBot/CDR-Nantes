"""Pure frame encoding/decoding for the Teensy USB protocol.

Wire format (unchanged, must stay byte-for-byte identical to the C++ side in
``common/usb_com/cpp/src/com.cpp``)::

    msg_type | msg_data | msg_length | CRC8 | END_BYTES_SIGNATURE
       1     |    N     |     1      |  1   |         4

``msg_length`` counts the message type byte plus the data, i.e. ``1 + N``.
The CRC8 is computed over ``msg_type | msg_data | msg_length``.

These functions are deliberately free of any I/O, threading or shared state so
they can be unit-tested without a serial port. They are also the single place
where a future COBS / byte-stuffing layer would be added: today the end
signature is written raw, so a payload that happens to contain the four bytes
``BA DD 1C C5`` truncates its own frame. See the November firmware rework.
"""

from __future__ import annotations

import crc8

from usb_com.python.messages import END_BYTES_SIGNATURE

# Smallest possible frame: 1 type byte + 1 length byte + 1 CRC + 4 signature
MIN_FRAME_SIZE = 7


def compute_crc8(data: bytes) -> int:
    """Computes the CRC8 of ``data``.

    A fresh ``crc8`` object is created on every call: the previous code shared
    a single mutable instance between the receiver thread and the sender, so a
    ``reset()`` on one side could land in the middle of an ``update()`` on the
    other and produce a wrong checksum.

    Args:
        data (bytes): Bytes to checksum.

    Returns:
        int: The CRC8 value, as a single byte value.
    """
    hasher = crc8.crc8()
    hasher.update(data)
    return hasher.digest()[0]


def encode_frame(payload: bytes, *, enable_crc: bool = True) -> bytes:
    """Wraps a payload into a complete wire frame.

    Args:
        payload (bytes): Message type byte followed by the message data.
        enable_crc (bool, optional): Append a CRC8 byte. Defaults to ``True``.

    Returns:
        bytes: The frame, signature included, ready to be written.

    Raises:
        ValueError: If ``payload`` is empty or longer than 255 bytes (the
            length field is a single byte).
    """
    if not payload:
        msg = "Cannot encode an empty payload"
        raise ValueError(msg)
    if len(payload) > 255:  # noqa: PLR2004
        msg = f"Payload too long for a one-byte length field: {len(payload)}"
        raise ValueError(msg)

    frame = payload + bytes([len(payload)])
    if enable_crc:
        frame += bytes([compute_crc8(frame)])

    return frame + END_BYTES_SIGNATURE


def decode_frame(raw: bytes, *, enable_crc: bool = True) -> tuple[int, bytes] | None:
    """Parses a complete wire frame.

    Args:
        raw (bytes): A frame as returned by ``read_until``, i.e. everything up
            to and including the end signature.
        enable_crc (bool, optional): Verify the CRC8 byte. Defaults to ``True``.

    Returns:
        tuple[int, bytes] | None: ``(message_id, data)`` on success, ``None`` if
        the frame is truncated, mis-sized or fails its checksum. Never raises,
        so the receiver loop can simply skip a bad frame.
    """
    # 1 payload byte + length + [crc] + 4 signature bytes
    min_size = MIN_FRAME_SIZE if enable_crc else MIN_FRAME_SIZE - 1

    if len(raw) < min_size:
        return None

    if not raw.endswith(END_BYTES_SIGNATURE):
        return None

    received_crc = raw[-5] if enable_crc else None
    body = raw[:-5] if enable_crc else raw[:-4]  # [leading noise?] payload + length

    declared_length = body[-1]

    # Everything is anchored on the END of the frame, because that is where the
    # signature is. `raw` may start with leftovers: line noise, or the tail of
    # a frame that was cut short. Locate the real start from the declared
    # length instead of assuming the frame begins at index 0 - otherwise a
    # single stray byte invalidates every following frame.
    frame_start = len(body) - 1 - declared_length

    if declared_length < 1 or frame_start < 0:
        # A zero length carries no message type byte at all.
        # Declared length larger than what we actually received: truncated
        # frame, or a size byte read out of a corrupted stream.
        return None

    payload = body[frame_start:-1]

    # Note: CRC-8 with init 0 is transparent to leading 0x00 bytes, so a size
    # byte under-reporting by one is undetectable when the skipped byte is
    # zero. Harmless (a sender never lies about its own size) but real.
    if enable_crc and compute_crc8(body[frame_start:]) != received_crc:
        return None

    return payload[0], payload[1:]
