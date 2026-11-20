"""On-disk format of a run log: the raw records and the JSONL lines.

A run leaves two views of the link behind:

    raw_usb.bin   every message as it went through, both ways, dated;
    log.jsonl     the same messages decoded, one JSON object per line, next
                  to what the Raspberry Pi itself decided.

The raw file is the reference: log.jsonl can always be rebuilt from it, so a
bug in the decoder never costs a run. Everything here is pure (no thread, no
file, no clock): the logger writes with it, the reader checks with it.

Raw record, little-endian, no padding:

    [u64 t_ns][u8 direction][u16 length][length bytes]

The bytes are the message as [ID][data], after COBS and CRC. That is what the
three transports have in common (the simulated one has no frame on a wire), so
one format serves them all. A rejected frame is kept as read on the port.

JSONL line: "t_rasp_ns", "src" ("teensy" or "rasp") and "stream" first, then
the fields of the stream. Units are those of the protocol: mm, rad, s, us, rpm.
"""

from __future__ import annotations

import json
import struct
from collections.abc import Iterator

from usb_com import TAP_RX, TAP_RX_REJECTED, TAP_TX, Messages
from usb_com.python.messages import PAYLOAD_FIELDS, payload_size, unpack_fields

RAW_HEADER = struct.Struct("<QBH")

SRC_TEENSY = "teensy"
SRC_RASP = "rasp"

# Stream of each message received from the Teensy. LOG_TICK is not here: it is
# split into one line per block (LOG_TICK_STREAMS).
RX_STREAMS: dict[Messages, str] = {
    Messages.ODOMETRY: "odometry",
    Messages.STATUS: "status",
    Messages.PID_TRACE: "ctrl",
    Messages.LOG_EVENT: "event",
    Messages.LOG_FW_INFO: "fw_info",
    Messages.LOG_PARAMS: "params",
}

# LOG_TICK carries one tick of everything; a table per topic is easier to load
# and to plot. Each line keeps the header (seq, tick, t_us), so the blocks of a
# tick are joined back on "tick".
LOG_TICK_HEADER = ("schema_version", "seq", "tick", "t_us")
LOG_TICK_STREAMS: dict[str, tuple[str, ...]] = {
    "est_in": (
        "d", "dt_us", "encoder_age_us", "heading_rad", "imu_timestamp_us",
        "imu_age_us", "optical_timestamp_us", "optical_dx_mm", "optical_dy_mm",
        "optical_step_dt_us", "optical_vx_mm_s", "optical_vy_mm_s",
        "input_flags",
    ),
    "enc": ("enc_counts", "enc_t_us_mid", "enc_read_ok"),
    "otos": ("otos_pos", "otos_vel", "otos_acc"),
    "imu": ("imu_raw_yaw_rad", "imu_yaw_offset_rad", "imu_accuracy"),
    "est": ("est_pose", "est_vel", "est_health", "est_flags"),
    "cmd_rx": ("cmd_count", "cmd_t_us"),
    "health": (
        "work_us", "health_flags", "rs485_fail", "log_dropped", "otos_read_us",
    ),
}

# Streams the decoder produces from the link. A line of any other stream comes
# from the Raspberry Pi itself (RunLogger.emit).
LINK_STREAMS = frozenset(
    {*RX_STREAMS.values(), *LOG_TICK_STREAMS, "cmd_tx", "print", "nack",
     "unknown_msg", "raw", "rejected"}
)

_MESSAGES_BY_ID = {message.value: message for message in Messages}


# ── Raw records ───────────────────────────────────────────────────────────

def pack_raw(t_ns: int, direction: int, payload: bytes) -> bytes:
    """Build one raw record.

    Args:
        t_ns: time of the message, in the time base of the run.
        direction: usb_com.TAP_RX, TAP_TX or TAP_RX_REJECTED.
        payload: the message, [ID][data].

    Returns:
        bytes: the record, header first.
    """
    return RAW_HEADER.pack(t_ns, direction, len(payload)) + payload


def iter_raw(data: bytes) -> Iterator[tuple[int, int, bytes]]:
    """Yield the records of a raw file, stopping at the first incomplete one.

    A run cut by a power loss ends in the middle of a record: everything
    before it is still good, and is returned.

    Args:
        data: content of raw_usb.bin.

    Yields:
        tuple: (t_ns, direction, payload).
    """
    offset = 0
    size = len(data)
    while offset + RAW_HEADER.size <= size:
        t_ns, direction, length = RAW_HEADER.unpack_from(data, offset)
        start = offset + RAW_HEADER.size
        if start + length > size:
            return
        yield t_ns, direction, data[start:start + length]
        offset = start + length


def raw_trailing_bytes(data: bytes) -> int:
    """Return how many bytes at the end of a raw file belong to no record."""
    consumed = sum(
        RAW_HEADER.size + len(payload) for _, _, payload in iter_raw(data)
    )
    return len(data) - consumed


# ── JSONL lines ───────────────────────────────────────────────────────────

def _json_default(value):
    """Turn what json does not know (numpy scalars, tuples in sets...) plain."""
    for convert in (float, list, str):
        try:
            return convert(value)
        except (TypeError, ValueError):
            continue
    return repr(value)


def dumps(line: dict) -> str:
    """Serialise one line, compact, floats with their exact representation."""
    return json.dumps(line, separators=(",", ":"), default=_json_default)


def make_line(t_ns: int, src: str, stream: str, fields: dict) -> dict:
    """Build a line: the three common keys first, then the fields."""
    return {"t_rasp_ns": t_ns, "src": src, "stream": stream, **fields}


def _split_log_tick(t_ns: int, fields: dict) -> list[dict]:
    """One line per block of a LOG_TICK, each with the header of the tick."""
    header = {name: fields[name] for name in LOG_TICK_HEADER}
    return [
        make_line(t_ns, SRC_TEENSY, stream,
                  {**header, **{name: fields[name] for name in names}})
        for stream, names in LOG_TICK_STREAMS.items()
    ]


def _undecoded(t_ns: int, src: str, payload: bytes, reason: str) -> dict:
    """A message kept as hex: unknown ID, or a size its layout does not have."""
    return make_line(t_ns, src, "raw", {
        "msg_id": payload[0] if payload else None,
        "hex": payload.hex(),
        "reason": reason,
    })


def decode_message(t_ns: int, direction: int, payload: bytes) -> list[dict]:
    """Decode one message of the link into its JSONL lines.

    Never raises on a message it cannot read: it is kept as hex, with the
    reason, so the count of lines always accounts for every message.

    Args:
        t_ns: time of the message.
        direction: usb_com.TAP_RX, TAP_TX or TAP_RX_REJECTED.
        payload: the message, [ID][data].

    Returns:
        list[dict]: one line, or one per block for a LOG_TICK.
    """
    if direction == TAP_RX_REJECTED:
        return [make_line(t_ns, SRC_TEENSY, "rejected", {"hex": payload.hex()})]

    src = SRC_RASP if direction == TAP_TX else SRC_TEENSY
    if not payload:
        return [_undecoded(t_ns, src, payload, "empty")]

    message = _MESSAGES_BY_ID.get(payload[0])
    data = payload[1:]

    if message is Messages.PRINT:
        text = data.decode("utf-8", errors="replace")
        return [make_line(t_ns, src, "print", {"text": text})]
    if message is Messages.NACK:
        return [make_line(t_ns, src, "nack", {})]
    if message is Messages.UNKNOWN_MSG_TYPE and len(data) == 1:
        return [make_line(t_ns, src, "unknown_msg", {"type_id": data[0]})]

    if message is None or message not in PAYLOAD_FIELDS:
        return [_undecoded(t_ns, src, payload, "no layout")]
    if len(data) != payload_size(message):
        return [_undecoded(t_ns, src, payload, "size")]

    fields = unpack_fields(message, data)

    if direction == TAP_TX:
        return [make_line(t_ns, SRC_RASP, "cmd_tx", {
            "msg": message.name, "msg_id": message.value, **fields,
        })]
    if message is Messages.LOG_TICK:
        return _split_log_tick(t_ns, fields)
    stream = RX_STREAMS.get(message)
    if stream is None:
        # A message of the Rasp -> Teensy range coming back: keep its name.
        return [make_line(t_ns, src, "raw", {
            "msg_id": message.value, "hex": payload.hex(),
            "reason": f"unexpected {message.name}",
        })]
    return [make_line(t_ns, SRC_TEENSY, stream, fields)]


def is_link_line(line: dict) -> bool:
    """Tell a line decoded from the link from one emitted by the Raspberry Pi."""
    return line.get("stream") in LINK_STREAMS
