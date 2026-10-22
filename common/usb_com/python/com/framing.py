"""Pure frame encoding/decoding for the Teensy USB protocol.

Wire format (must stay byte-for-byte identical to the C++ side in
``common/usb_com/cpp/src/com.cpp`` and ``cobs.cpp``)::

    COBS( msg_type | msg_data | CRC8 ) | 0x00
               1   |    N     |  1

The CRC8 (polynomial 0x07, init 0xFF) covers ``msg_type | msg_data``. COBS
(Consistent Overhead Byte Stuffing) removes every 0x00 from the frame, at a
cost of one byte per 254, so that 0x00 can delimit frames: whatever the payload
holds, it cannot cut its own frame short, and a reader that lost track is back
in sync at the next 0x00.

These functions are deliberately free of any I/O, threading or shared state so
they can be unit-tested without a serial port.
"""

from __future__ import annotations

import crc8

from usb_com.python.messages import FRAME_DELIMITER, MAX_PAYLOAD_SIZE

# Initial value of the CRC8. Not 0: with 0, leading 0x00 bytes leave the CRC
# unchanged, so a stray COBS byte 0x01 in front of a frame (decoded as a leading
# zero) would pass as a valid message with ID 0.
CRC8_INIT = 0xFF


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
    hasher = crc8.crc8(initial_start=CRC8_INIT)
    hasher.update(data)
    return hasher.digest()[0]


def cobs_encode(data: bytes) -> bytes:
    """COBS-encodes ``data`` (delimiter not added).

    Same algorithm as ``cobs_encode`` in ``cpp/src/cobs.cpp``, including the
    trailing code byte after a full 254-byte block.

    Args:
        data (bytes): Bytes to encode.

    Returns:
        bytes: The encoded bytes, none of them 0x00.
    """
    out = bytearray([0])
    code_index = 0
    code = 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code = 1
            code_index = len(out)
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:  # noqa: PLR2004
                out[code_index] = code
                code = 1
                code_index = len(out)
                out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    """Decodes COBS bytes (delimiter excluded).

    Args:
        data (bytes): Encoded bytes.

    Returns:
        bytes | None: The decoded bytes, or ``None`` if ``data`` is not valid
        COBS (a 0x00 inside, a block running past the end).
    """
    out = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        index += 1
        if code == 0:
            return None
        block = data[index : index + code - 1]
        if len(block) != code - 1 or 0 in block:
            return None
        out += block
        index += code - 1
        # A block shorter than 254 stands for a zero, except the last one.
        if code != 0xFF and index < len(data):  # noqa: PLR2004
            out.append(0)
    return bytes(out)


def encode_frame(payload: bytes, *, enable_crc: bool = True) -> bytes:
    """Wraps a payload into a complete wire frame.

    Args:
        payload (bytes): Message type byte followed by the message data.
        enable_crc (bool, optional): Append a CRC8 byte. Defaults to ``True``.
            The Teensy always expects one: ``False`` is for tests only.

    Returns:
        bytes: The frame, delimiter included, ready to be written.

    Raises:
        ValueError: If ``payload`` is empty or longer than ``MAX_PAYLOAD_SIZE``.
    """
    if not payload:
        msg = "Cannot encode an empty payload"
        raise ValueError(msg)
    if len(payload) > MAX_PAYLOAD_SIZE:
        msg = f"Payload too long: {len(payload)} > {MAX_PAYLOAD_SIZE} bytes"
        raise ValueError(msg)

    body = payload + bytes([compute_crc8(payload)]) if enable_crc else payload
    return cobs_encode(body) + FRAME_DELIMITER


def decode_frame(raw: bytes, *, enable_crc: bool = True) -> tuple[int, bytes] | None:
    """Parses a complete wire frame.

    Args:
        raw (bytes): A frame as returned by ``read_until(FRAME_DELIMITER)``:
            the encoded bytes and, optionally, the delimiter that ends them.
        enable_crc (bool, optional): Verify the CRC8 byte. Defaults to ``True``.

    Returns:
        tuple[int, bytes] | None: ``(message_id, data)`` on success, ``None`` if
        the frame is empty, not valid COBS, too short or fails its checksum.
        Never raises, so the receiver loop can simply skip a bad frame.
    """
    if raw.endswith(FRAME_DELIMITER):
        raw = raw[: -len(FRAME_DELIMITER)]
    if not raw:
        return None

    body = cobs_decode(raw)
    if body is None:
        return None

    if enable_crc:
        # At least the ID and the CRC
        if len(body) < 2:  # noqa: PLR2004
            return None
        payload, received_crc = body[:-1], body[-1]
        if compute_crc8(payload) != received_crc:
            return None
    else:
        payload = body
        if not payload:
            return None

    return payload[0], payload[1:]
