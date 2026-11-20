"""Tests of the run log: on-disk format, logger, reader, and the link taps.

    pip install -e ".[dev]"
    python -m pytest tests

No robot and no serial port: the Teensy is played by messages built with the
protocol itself, pushed through the same tap a real link uses.
"""

import json
import logging
import threading
import time

import pytest

from usb_com import (
    TAP_RX,
    TAP_RX_REJECTED,
    TAP_TX,
    Messages,
    TraceMask,
    pack_message,
)
from usb_com.python.messages import PAYLOAD_FIELDS, PAYLOAD_FORMATS, StatusFlag

from robot1.rasp.comm import ComLink
from robot1.rasp.telemetry import log_format, log_reader
from robot1.rasp.telemetry.logger import (
    NULL_LOGGER,
    NullRunLogger,
    RunLogger,
    create_run_logger,
)

TICK_NS = 15_000_000  # control period of the Teensy


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


def log_tick(seq, tick):
    """A LOG_TICK whose every value is recognisable."""
    count = len(_zero_values(Messages.LOG_TICK))
    values = list(range(1, count + 1))
    values[0] = 1               # schema_version
    values[1] = seq & 0xFFFF
    values[2] = tick
    values[3] = (tick * 15_000) & 0xFFFFFFFF
    return pack_message(Messages.LOG_TICK, *values)


def pid_trace(tick):
    return pack_message(Messages.PID_TRACE, (tick * 15_000) & 0xFFFFFFFF,
                        *[float(i) for i in range(27)], 0)


def status(flags, t_us=0):
    return pack_message(Messages.STATUS, t_us, flags, 0, 1, 0, 0)


def _zero_values(message):
    import struct

    fmt = PAYLOAD_FORMATS[message]
    return struct.unpack(fmt, bytes(struct.calcsize(fmt)))


def make_logger(tmp_path, clock, **kwargs):
    kwargs.setdefault("mode", "dummy")
    kwargs.setdefault("lossless", True)
    return RunLogger(tmp_path, "test", clock_ns=clock, clock_name="fake",
                     **kwargs)


# ── On-disk format ────────────────────────────────────────────────────────

def test_raw_records_round_trip_and_survive_a_cut():
    records = [(0, TAP_TX, b"\x00abc"), (15, TAP_RX, b"\x80" + bytes(28)),
               (2**40, TAP_RX_REJECTED, b"")]
    data = b"".join(log_format.pack_raw(*record) for record in records)
    assert list(log_format.iter_raw(data)) == records
    assert log_format.raw_trailing_bytes(data) == 0

    # A power loss in the middle of the last record: the others are intact.
    cut = data[:-1]
    assert list(log_format.iter_raw(cut)) == records[:2]
    assert log_format.raw_trailing_bytes(cut) == log_format.RAW_HEADER.size - 1

    # Cut inside the bytes of a message rather than inside its header.
    cut = data + log_format.pack_raw(99, TAP_RX, b"\x80123")[:-2]
    assert list(log_format.iter_raw(cut)) == records
    assert log_format.raw_trailing_bytes(cut) == log_format.RAW_HEADER.size + 2


def test_log_tick_is_split_without_losing_or_repeating_a_field():
    named = [name for name, _ in PAYLOAD_FIELDS[Messages.LOG_TICK]]
    split = [name for names in log_format.LOG_TICK_STREAMS.values()
             for name in names]
    assert sorted(split + list(log_format.LOG_TICK_HEADER)) == sorted(named)
    assert len(split) == len(set(split))

    lines = log_format.decode_message(42, TAP_RX, log_tick(seq=7, tick=1000))
    assert [line["stream"] for line in lines] == list(log_format.LOG_TICK_STREAMS)
    for line in lines:
        assert line["t_rasp_ns"] == 42 and line["src"] == "teensy"
        assert (line["seq"], line["tick"], line["t_us"]) == (7, 1000, 15_000_000)
    by_stream = {line["stream"]: line for line in lines}
    assert by_stream["enc"]["enc_counts"] == [20, 21, 22]
    assert by_stream["est"]["est_pose"] == [37.0, 38.0, 39.0]
    assert by_stream["cmd_rx"]["cmd_count"] == 45


def test_decode_commands_text_and_what_cannot_be_read():
    target = pack_message(Messages.SET_TARGET, 100.0, 200.0, 0.5, 1.0, 2.0, 0.0)
    (line,) = log_format.decode_message(5, TAP_TX, target)
    assert line == {
        "t_rasp_ns": 5, "src": "rasp", "stream": "cmd_tx", "msg": "SET_TARGET",
        "msg_id": 0, "x": 100.0, "y": 200.0, "theta": 0.5, "vx": 1.0,
        "vy": 2.0, "omega": 0.0,
    }

    (line,) = log_format.decode_message(6, TAP_RX, bytes([254]) + b"hello")
    assert (line["stream"], line["text"]) == ("print", "hello")

    # Unknown ID, wrong size, rejected frame: kept as hex, never an exception.
    (line,) = log_format.decode_message(7, TAP_RX, bytes([99, 1, 2]))
    assert line["stream"] == "raw" and line["hex"] == "630102"
    (line,) = log_format.decode_message(8, TAP_RX, bytes([128, 1, 2]))
    assert line["stream"] == "raw" and line["reason"] == "size"
    (line,) = log_format.decode_message(9, TAP_RX_REJECTED, b"\x05\x06\x00")
    assert line["stream"] == "rejected" and line["hex"] == "050600"
    (line,) = log_format.decode_message(10, TAP_RX, b"")
    assert line["stream"] == "raw"

    # Every line goes through json, NaN included (a broken sensor must show).
    nan = pack_message(Messages.ODOMETRY, float("nan"), 0, 0, 0, 0, 0, 1)
    (line,) = log_format.decode_message(11, TAP_RX, nan)
    assert "NaN" in log_format.dumps(line)


# ── A Teensy session, played from synthetic messages ──────────────────────

def play_teensy_session(log, com, clock, ticks, skip=()):
    """Feed `ticks` control ticks of Teensy messages through the tap."""
    com.receive(pack_message(Messages.LOG_FW_INFO, 1, b"v1-test", b"2027-01-06"))
    seq = 0
    for tick in range(ticks):
        clock.t = tick * TICK_NS
        if tick in skip:
            seq += 1            # a frame the Teensy dropped: its seq is lost
            continue
        com.receive(log_tick(seq, tick))
        com.receive(pid_trace(tick))
        seq += 1
        if tick % 2 == 0:
            com.receive(pack_message(Messages.ODOMETRY, 1.0, 2.0, 0.0, 0, 0, 0,
                                     tick * 15_000))
        if tick % 13 == 0:
            com.receive(status(StatusFlag.TRACE_ON | StatusFlag.LOG_TICK_ON))
        if tick % 3 == 0:
            com.send_bytes(pack_message(Messages.SET_TARGET, 1.0 * tick, 0.0,
                                        0.0, 0.0, 0.0, 0.0))
            log.emit("state", odom_x=1.0 * tick)


def test_teensy_session_is_read_back_without_loss(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock)
    com = FakeCom()
    log.attach(com)
    ticks = 2000                                   # 30 s at 66.7 Hz
    play_teensy_session(log, com, clock, ticks)
    stats = log.close()

    check = log_reader.check_run(log.folder)
    assert check.ok, check.problems
    assert stats["dropped"] == 0 and stats["seq_gaps"] == 0

    for stream in log_format.LOG_TICK_STREAMS:
        assert check.streams[stream] == ticks
    assert check.streams["ctrl"] == ticks
    assert check.streams["odometry"] == ticks // 2
    assert check.streams["state"] == len(range(0, ticks, 3))
    # SET_TRACE at attach, then one SET_TARGET every third tick.
    assert check.streams["cmd_tx"] == 1 + len(range(0, ticks, 3))
    assert check.streams["link"] == 29             # one per second of run

    meta = check.meta
    assert meta["mode"] == "dummy" and meta["clock"] == "fake"
    assert meta["teensy_fw_info"]["git_describe"] == "v1-test"
    assert meta["t_end_rasp_ns"] == (ticks - 1) * TICK_NS
    assert meta["streams"] == {"pid_trace": True, "log_tick": True,
                               "lidar_raw": False}

    # The lines keep the order and the time of the loop that produced them.
    lines = log_reader.read_lines(log.folder)
    times = [line["t_rasp_ns"] for line in lines]
    assert times == sorted(times)
    enc = log_reader.stream_lines(lines, "enc")
    assert [line["tick"] for line in enc] == list(range(ticks))


def test_missing_teensy_frames_are_counted(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock)
    com = FakeCom()
    log.attach(com)
    play_teensy_session(log, com, clock, 300, skip={100, 101, 250})
    stats = log.close()

    assert stats["seq_gaps"] == 3
    check = log_reader.check_run(log.folder)
    assert not check.ok
    assert any("3 trames Teensy manquantes" in problem
               for problem in check.problems)
    # Nothing else is wrong: what did arrive is all there.
    assert len(check.problems) == 1


def test_rebuild_gives_the_same_log_back(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock)
    com = FakeCom()
    log.attach(com)
    play_teensy_session(log, com, clock, 200)
    log.close()

    before = log_reader.read_lines(log.folder)
    # A decoder bug wrote garbage: the raw file still holds the run.
    (log.folder / "log.jsonl").write_text(
        "\n".join(log_format.dumps(line) for line in before
                  if not log_format.is_link_line(line)) + "\n",
        encoding="utf-8",
    )
    assert not log_reader.check_run(log.folder).ok

    log_reader.rebuild_log(log.folder)
    after = log_reader.read_lines(log.folder)
    assert sorted(map(log_format.dumps, after)) == sorted(
        map(log_format.dumps, before))
    link = [line for line in after if log_format.is_link_line(line)]
    assert link == [line for line in before if log_format.is_link_line(line)]


def test_a_run_cut_before_its_end_is_reported_and_still_readable(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock)
    com = FakeCom()
    log.attach(com)
    play_teensy_session(log, com, clock, 100)
    time.sleep(1.2)                 # let the writer flush, as a live run does
    # Power loss: the files are copied as they are, the run is never closed.
    cut = tmp_path / "cut"
    cut.mkdir()
    for name in ("meta.json", "raw_usb.bin", "log.jsonl"):
        (cut / name).write_bytes((log.folder / name).read_bytes())
    raw = (cut / "raw_usb.bin").read_bytes()
    (cut / "raw_usb.bin").write_bytes(raw[:-5])
    log.close()

    check = log_reader.check_run(cut)
    assert not check.ok
    assert any("non clos" in problem for problem in check.problems)
    assert check.messages > 0 and check.lines > 0


# ── Per-tick messages asked from the Teensy ───────────────────────────────

def test_set_trace_is_sent_and_sent_again_when_the_teensy_forgets(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock)
    com = FakeCom()
    log.attach(com)
    wanted = pack_message(Messages.SET_TRACE,
                          TraceMask.PID_TRACE | TraceMask.LOG_TICK)
    assert com.sent == [wanted]

    both = StatusFlag.TRACE_ON | StatusFlag.LOG_TICK_ON
    clock.t = 2_000_000_000
    com.receive(status(both))
    assert len(com.sent) == 1                      # all is well: nothing sent

    # The Teensy restarted: STATUS no longer shows the traces.
    clock.t = 3_000_000_000
    com.receive(status(StatusFlag.MOTORS_ENABLED))
    assert com.sent == [wanted, wanted]
    clock.t = 3_200_000_000
    com.receive(status(StatusFlag.TRACE_ON))       # one of the two missing
    assert len(com.sent) == 2                      # asked less than 1 s ago
    clock.t = 4_100_000_000
    com.receive(status(StatusFlag.TRACE_ON))
    assert len(com.sent) == 3
    log.close()


def test_no_set_trace_when_the_run_asks_for_nothing(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock, trace_mask=TraceMask.NONE)
    com = FakeCom()
    log.attach(com)
    com.receive(status(0))
    assert com.sent == []
    log.close()
    assert log_reader.read_meta(log.folder)["streams"]["log_tick"] is False


# ── The log never gets in the way ─────────────────────────────────────────

def test_a_full_queue_drops_and_counts_but_never_blocks(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock, lossless=False, queue_size=10)
    gate = threading.Event()
    original = log._handle

    def stuck(item):
        gate.wait(5.0)          # a disk that stopped answering
        original(item)

    log._handle = stuck
    started = time.monotonic()
    for index in range(1000):
        log.emit("state", index=index)
    assert time.monotonic() - started < 1.0        # the loop never waited
    gate.set()
    stats = log.close()

    assert stats["dropped"] >= 980
    assert stats["lines"] + stats["dropped"] == 1000
    check = log_reader.check_run(log.folder)
    assert any("jetes par le logger" in problem for problem in check.problems)


def test_a_disk_failure_stops_the_log_not_the_caller(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock)

    def disk_full(*_args, **_kwargs):
        raise OSError(28, "No space left on device")

    log.emit("state", index=0)
    time.sleep(0.2)
    log._write_line = disk_full
    for index in range(50):
        log.emit("state", index=index)             # must not raise
    log.close()                                    # must not raise

    meta = log_reader.read_meta(log.folder)
    assert "No space left" in meta["error"]
    assert not log_reader.check_run(log.folder).ok


def test_emit_refuses_a_stream_of_the_link(tmp_path):
    log = make_logger(tmp_path, FakeClock())
    with pytest.raises(ValueError):
        log.emit("odometry", x=0.0)
    log.close()


def test_scans_go_to_their_own_file_only_when_asked(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock, lidar_raw=True)
    log.emit_scan(3, [0, 1000, 2000], [500, 510, 495], [15, 14, 15])
    log.emit("state", index=0)
    log.close()
    (scan,) = log_reader.read_lines(log.folder, "lidar.jsonl")
    assert scan["stream"] == "lidar_scan" and scan["number"] == 3
    assert scan["distances_mm"] == [500, 510, 495]
    assert log_reader.check_run(log.folder).ok

    quiet = make_logger(tmp_path, clock, lidar_raw=False)
    quiet.emit_scan(4, [0], [500])
    quiet.close()
    assert not (quiet.folder / "lidar.jsonl").exists()


def test_two_runs_in_the_same_minute_get_two_folders(tmp_path):
    first = make_logger(tmp_path, FakeClock())
    second = make_logger(tmp_path, FakeClock())
    first.close()
    second.close()
    assert first.folder != second.folder
    assert second.folder.name.endswith("_02")


# ── Switched off ──────────────────────────────────────────────────────────

def test_switched_off_nothing_is_written(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    log = create_run_logger("match", enabled=False)
    assert log is NULL_LOGGER and not log.enabled
    com = FakeCom()
    log.attach(com)
    log.emit("state", x=1)
    log.emit_scan(1, [0], [1])
    log.set_meta(note="x")
    assert log.close() == {}
    assert com.tap is None and com.sent == []      # the link is left alone
    assert list(tmp_path.iterdir()) == []


def test_environment_switches_the_log(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("ROBOT_MODE", "dummy")
    monkeypatch.setenv("ROBOT_LOG", "0")
    assert isinstance(create_run_logger("match"), NullRunLogger)
    assert list(tmp_path.iterdir()) == []

    monkeypatch.setenv("ROBOT_LOG", "1")
    log = create_run_logger("match")
    assert log.enabled and log.folder.parent == tmp_path / "logs"
    log.close()
    assert log_reader.read_meta(log.folder)["mode"] == "dummy"


def test_a_log_that_cannot_be_created_does_not_stop_the_robot(tmp_path,
                                                              monkeypatch):
    blocker = tmp_path / "logs"
    blocker.write_text("a file where the folder should be")
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("ROBOT_MODE", "dummy")
    assert create_run_logger("match", enabled=True) is NULL_LOGGER


# ── The taps of the transports ────────────────────────────────────────────

def test_sim2d_link_is_tapped_both_ways():
    from robot1.sim2d import transport
    from robot1.sim2d.robot import SimulatedRobot

    transport.set_robot(SimulatedRobot(x=500.0, y=500.0, theta=0.0, seed=1))
    com = transport.Sim2dCom()
    assert isinstance(com, ComLink)
    seen = []
    com.set_tap(lambda direction, payload: seen.append((direction, payload)))

    target = pack_message(Messages.SET_TARGET, 600.0, 500.0, 0.0, 0.0, 0.0, 0.0)
    com.send_bytes(target)
    com.robot.step(0.05)
    delivered = []
    com.add_callback(delivered.append, Messages.ODOMETRY.value)
    count = com.publish_odometry(0.05)

    assert seen[0] == (TAP_TX, target)
    received = [payload for direction, payload in seen if direction == TAP_RX]
    assert len(received) == count == len(delivered) > 0
    assert all(payload[0] == Messages.ODOMETRY.value for payload in received)
    assert received[0][1:] == delivered[0]         # same bytes as the callback

    com.set_tap(None)
    com.send_bytes(target)
    assert len(seen) == 1 + count


def test_usb_link_is_tapped_both_ways_and_sees_rejected_frames():
    from usb_com.python.com import Com
    from usb_com.python.com.framing import encode_frame

    com = Com(logging.getLogger("test"), 0, 0, 0, 115200, enable_dummy=True)
    assert isinstance(com, ComLink)
    seen = []
    com.set_tap(lambda direction, payload: seen.append((direction, payload)))

    target = pack_message(Messages.SET_TARGET, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0)
    com.send_bytes(target)
    assert seen == [(TAP_TX, target)]

    # The dummy port has no receiver thread: run the real one over it.
    odometry = pack_message(Messages.ODOMETRY, 1.0, 2.0, 0.5, 0.0, 0.0, 0.0, 42)
    delivered = []
    com.add_callback(delivered.append, Messages.ODOMETRY.value)
    com._device.dummy_add_input(encode_frame(odometry) + b"\x07\x07\x00")
    receiver = threading.Thread(target=com._Com__receiver, daemon=True)
    receiver.start()
    deadline = time.monotonic() + 2.0
    while len(seen) < 4 and time.monotonic() < deadline:
        time.sleep(0.01)
    com.close()
    receiver.join(timeout=2.0)

    assert (TAP_RX, odometry) in seen
    assert delivered == [odometry[1:]]
    assert (TAP_RX_REJECTED, b"\x07\x07\x00") in seen
    # The NACK answering the bad frame went through the tap too.
    assert (TAP_TX, Messages.NACK.to_bytes()) in seen


def test_a_tap_that_raises_does_not_break_the_link():
    from usb_com.python.com import Com

    com = Com(logging.getLogger("test"), 0, 0, 0, 115200, enable_dummy=True)

    def broken(_direction, _payload):
        raise RuntimeError("recorder bug")

    com.set_tap(broken)
    com.send_bytes(pack_message(Messages.STOP))    # must not raise
    assert com._device.output_buffer               # and the message left
    com.close()


def test_lines_are_plain_json(tmp_path):
    clock = FakeClock()
    log = make_logger(tmp_path, clock)
    log.emit("state", trajectory=[(1.0, 2.0), (3.0, 4.0)], ok=True, note=None)
    log.close()
    text = (log.folder / "log.jsonl").read_text(encoding="utf-8")
    (line,) = [json.loads(row) for row in text.splitlines()]
    assert line["trajectory"] == [[1.0, 2.0], [3.0, 4.0]]
    assert list(line)[:3] == ["t_rasp_ns", "src", "stream"]
