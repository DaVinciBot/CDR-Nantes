"""Run log: what went through the link and what the robot decided, on disk.

One run, one folder:

    meta.json     who, when, which settings, and the counters of the run
    raw_usb.bin   every message of the link, both ways, dated (the reference)
    log.jsonl     the same messages decoded, plus the streams of the Pi
    lidar.jsonl   raw LiDAR scans, kept apart: large, and not always wanted

The log can be on or off. Off, a NullRunLogger takes its place: same methods,
nothing written, so the callers never test for it.

Two rules shape everything here:

  * the match loop never waits for the log. tap() and emit() only date the
    item and push it to a queue; one writer thread does the decoding and the
    disk. A full queue costs log lines (counted), never a loop tick; a disk
    that fails stops the log, never the robot;
  * the time of a line is the time of the loop that produced it. The clock is
    handed in: time.monotonic_ns on the robot, the virtual clock in the
    simulator, where a 100 s match lasts under a second of wall time.

Nothing here opens a network connection, and nothing may: during a match the
robot talks to nobody outside the table. The folder is fetched afterwards.

This module stays free of the Rerun SDK, so it loads on a machine without it.
"""

from __future__ import annotations

import datetime
import json
import logging
import os
import queue
import struct
import subprocess
import threading
import time
from pathlib import Path

from usb_com import TAP_RX, TAP_RX_REJECTED, TAP_TX, Messages, TraceMask, pack_message
from usb_com.python.messages import LOG_SCHEMA_VERSION, StatusFlag

from . import log_format

logger = logging.getLogger("RUN_LOG")

RAW_FILE = "raw_usb.bin"
LOG_FILE = "log.jsonl"
LIDAR_FILE = "lidar.jsonl"
META_FILE = "meta.json"

FLUSH_PERIOD_S = 0.5
QUEUE_SIZE = 50_000
# One "link" line of counters per second of run time.
LINK_PERIOD_NS = 1_000_000_000
# SET_TRACE is sent again, at most this often, while STATUS says the Teensy
# does not send what was asked (it forgets the mask when it restarts).
TRACE_RETRY_NS = 1_000_000_000

ENV_ENABLE = "ROBOT_LOG"

_USB, _LINE, _SCAN = 0, 1, 2
_STOP = object()

# STATUS data: t_us (u32) then flags (u16), after the ID byte.
_STATUS_FLAGS = struct.Struct("<H")
_STATUS_FLAGS_OFFSET = 1 + 4

_TRACE_STATUS_BITS = (
    (TraceMask.PID_TRACE, StatusFlag.TRACE_ON),
    (TraceMask.LOG_TICK, StatusFlag.LOG_TICK_ON),
)


class NullRunLogger:
    """The log, switched off: same methods as RunLogger, nothing happens."""

    enabled = False
    folder = None

    def attach(self, com) -> None:
        """Do nothing: the link is not listened to."""

    def tap(self, direction: int, payload: bytes) -> None:
        """Do nothing."""

    def emit(self, stream: str, **fields) -> None:
        """Do nothing."""

    def emit_scan(self, number: int, angles, distances, qualities=None) -> None:
        """Do nothing."""

    def set_meta(self, **fields) -> None:
        """Do nothing."""

    def close(self) -> dict:
        """Do nothing.

        Returns:
            dict: empty, there are no counters.
        """
        return {}


NULL_LOGGER = NullRunLogger()


class RunLogger:
    """Records one run in its own folder. See the module docstring."""

    enabled = True

    def __init__(
        self,
        root,
        name: str,
        *,
        mode: str,
        clock_ns=time.monotonic_ns,
        clock_name: str = "monotonic",
        trace_mask: int = TraceMask.PID_TRACE | TraceMask.LOG_TICK,
        lidar_raw: bool = False,
        lossless: bool = False,
        meta: dict | None = None,
        queue_size: int = QUEUE_SIZE,
    ) -> None:
        """Create the folder, write meta.json and start the writer thread.

        Args:
            root: directory that holds the run folders.
            name: what the run is (e.g. "match", "otos_lin_x"); the folder is
                named after the date and this.
            mode: "hardware", "dummy" or "sim2d", written in meta.json.
            clock_ns: time source, in nanoseconds. Must be the clock of the
                loop being recorded.
            clock_name: which clock that is, written in meta.json
                ("monotonic", "sim").
            trace_mask: per-tick messages asked from the Teensy when a link is
                attached (TraceMask). 0 asks for nothing.
            lidar_raw: keep the raw scans handed to emit_scan().
            lossless: wait for room in the queue instead of dropping. Only
                for a simulated run, which has no real time to keep.
            meta: extra entries for meta.json (test type, note, seed...).
            queue_size: items waiting for the writer, at most.

        Raises:
            OSError: if the folder or its files cannot be created.
        """
        self._clock_ns = clock_ns
        self._lossless = lossless
        self._trace_mask = int(trace_mask)
        self._lidar_raw = lidar_raw
        self._com = None
        self._last_trace_request_ns = None

        # Absolute from the start: the folder must not move if the working
        # directory changes during the run.
        self.folder = _new_run_folder(Path(root).resolve(), name)
        self._raw = open(self.folder / RAW_FILE, "wb")
        self._log = open(self.folder / LOG_FILE, "w", encoding="utf-8", newline="\n")
        self._lidar = None
        if lidar_raw:
            self._lidar = open(self.folder / LIDAR_FILE, "w", encoding="utf-8",
                               newline="\n")

        self._stats = {
            "usb_rx": 0, "usb_tx": 0, "usb_rejected": 0, "lines": 0,
            "scans": 0, "dropped": 0, "seq_gaps": 0, "decode_errors": 0,
        }
        self._streams: dict[str, int] = {}
        self._last_seq: dict[str, int] = {}
        self._next_link_ns = None
        self._error = None
        self._closed = False

        self._meta = {
            "run_id": self.folder.name,
            "name": name,
            "started_at": datetime.datetime.now().isoformat(timespec="seconds"),
            "schema_version": LOG_SCHEMA_VERSION,
            "mode": mode,
            "clock": clock_name,
            "streams": {
                "pid_trace": bool(self._trace_mask & TraceMask.PID_TRACE),
                "log_tick": bool(self._trace_mask & TraceMask.LOG_TICK),
                "lidar_raw": lidar_raw,
            },
            "git": {"rasp": _git_describe()},
            "t_start_rasp_ns": clock_ns(),
            "t_end_rasp_ns": None,
        }
        if meta:
            self._meta.update(meta)
        self._meta_lock = threading.Lock()
        self._write_meta()

        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._writer = threading.Thread(target=self._run, name="RunLogWriter",
                                        daemon=True)
        self._writer.start()
        logger.info("Log du run dans %s", self.folder)

    # ── Producers: never block, never raise into the caller ───────────────

    def attach(self, com) -> None:
        """Listen to a link, and ask the Teensy for the per-tick messages.

        Args:
            com: the transport (comm.link.ComLink).
        """
        self._com = com
        com.set_tap(self.tap)
        if self._trace_mask:
            self._request_trace(self._clock_ns())

    def tap(self, direction: int, payload: bytes) -> None:
        """Take one message of the link. This is the tap given to the transport.

        Args:
            direction: usb_com.TAP_RX, TAP_TX or TAP_RX_REJECTED.
            payload: the message, [ID][data].
        """
        t_ns = self._clock_ns()
        self._put((_USB, t_ns, direction, bytes(payload)))
        if (
            direction == TAP_RX
            and self._trace_mask
            and payload
            and payload[0] == Messages.STATUS.value
        ):
            self._check_trace(payload, t_ns)

    def emit(self, stream: str, **fields) -> None:
        """Log one line of a stream of the Raspberry Pi.

        The values must be plain (numbers, strings, lists, dicts) and must
        not be changed afterwards: they are serialised later, by the writer.

        Args:
            stream: name of the stream ("state", "path", "strat", "match"...).
            **fields: content of the line.

        Raises:
            ValueError: if the stream is one the link decoder produces. A
                line that claims to come from the link but does not would
                make the log lie about what the Teensy sent.
        """
        if stream in log_format.LINK_STREAMS:
            raise ValueError(f"stream '{stream}' is reserved for the link")
        self._put((_LINE, self._clock_ns(), stream, fields))

    def emit_scan(self, number: int, angles, distances, qualities=None) -> None:
        """Log one raw LiDAR scan, if the run keeps them.

        The scan is written as three flat integer arrays, not as a list of
        points: about a thousand points per scan, ten times a second.

        Args:
            number: scan counter of the acquisition.
            angles: angles of the points, in millidegrees.
            distances: distances of the points, in mm.
            qualities: quality of the points, optional.
        """
        if not self._lidar_raw:
            return
        fields = {
            "number": number,
            "angles_mdeg": [int(value) for value in angles],
            "distances_mm": [int(value) for value in distances],
        }
        if qualities is not None:
            fields["qualities"] = [int(value) for value in qualities]
        self._put((_SCAN, self._clock_ns(), "lidar_scan", fields))

    def set_meta(self, **fields) -> None:
        """Add entries to meta.json (team colour, start pose, note...)."""
        with self._meta_lock:
            self._meta.update(fields)

    def _put(self, item) -> None:
        if self._closed:
            return
        if self._lossless:
            self._queue.put(item)
            return
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self._stats["dropped"] += 1

    # ── Per-tick messages of the Teensy ───────────────────────────────────

    def _request_trace(self, t_ns: int) -> None:
        """Send SET_TRACE with the mask of this run."""
        self._last_trace_request_ns = t_ns
        try:
            self._com.send_bytes(pack_message(Messages.SET_TRACE, self._trace_mask))
        except Exception as error:  # noqa: BLE001
            logger.error("SET_TRACE non envoye : %s", error)

    def _check_trace(self, payload: bytes, t_ns: int) -> None:
        """Ask again when STATUS shows the Teensy lost the mask (it restarted)."""
        end = _STATUS_FLAGS_OFFSET + _STATUS_FLAGS.size
        if len(payload) < end or self._com is None:
            return
        (flags,) = _STATUS_FLAGS.unpack_from(payload, _STATUS_FLAGS_OFFSET)
        missing = any(
            self._trace_mask & mask and not flags & status_bit
            for mask, status_bit in _TRACE_STATUS_BITS
        )
        if not missing:
            return
        last = self._last_trace_request_ns
        if last is not None and t_ns - last < TRACE_RETRY_NS:
            return
        self._request_trace(t_ns)

    # ── Writer thread ─────────────────────────────────────────────────────

    def _run(self) -> None:
        next_flush = time.monotonic() + FLUSH_PERIOD_S
        while True:
            try:
                item = self._queue.get(timeout=FLUSH_PERIOD_S)
            except queue.Empty:
                item = None
            if item is _STOP:
                break
            if item is not None and self._error is None:
                self._handle(item)
            if time.monotonic() >= next_flush:
                self._flush()
                next_flush = time.monotonic() + FLUSH_PERIOD_S

    def _handle(self, item) -> None:
        """Write one item. A disk error ends the log, the robot goes on."""
        try:
            if item[0] == _USB:
                self._write_usb(item[1], item[2], item[3])
            else:
                self._write_line(
                    self._lidar if item[0] == _SCAN else self._log,
                    log_format.make_line(item[1], log_format.SRC_RASP, item[2],
                                         item[3]),
                )
                if item[0] == _SCAN:
                    self._stats["scans"] += 1
            self._link_line(item[1])
        except OSError as error:
            self._fail(error)
        except Exception as error:  # noqa: BLE001
            # One line that cannot be written must not take the others down.
            self._stats["decode_errors"] += 1
            logger.error("Ligne de log perdue : %s", error)

    def _write_usb(self, t_ns: int, direction: int, payload: bytes) -> None:
        self._raw.write(log_format.pack_raw(t_ns, direction, payload))
        if direction == TAP_TX:
            self._stats["usb_tx"] += 1
        elif direction == TAP_RX_REJECTED:
            self._stats["usb_rejected"] += 1
        else:
            self._stats["usb_rx"] += 1

        lines = log_format.decode_message(t_ns, direction, payload)
        for line in lines:
            self._write_line(self._log, line)
        if direction != TAP_RX or not lines:
            return
        first = lines[0]
        if "seq" in first:
            self._track_seq(payload[0], first["seq"])
        if first["stream"] in ("fw_info", "params"):
            content = {key: value for key, value in first.items()
                       if key not in ("t_rasp_ns", "src", "stream")}
            self.set_meta(**{f"teensy_{first['stream']}": content})

    def _track_seq(self, message_id: int, seq: int) -> None:
        """Count the messages missing between two sequence numbers (u16)."""
        key = str(message_id)
        last = self._last_seq.get(key)
        self._last_seq[key] = seq
        if last is not None:
            self._stats["seq_gaps"] += (seq - last - 1) & 0xFFFF

    def _write_line(self, handle, line: dict) -> None:
        handle.write(log_format.dumps(line))
        handle.write("\n")
        self._stats["lines"] += 1
        stream = line["stream"]
        self._streams[stream] = self._streams.get(stream, 0) + 1

    def _link_line(self, t_ns: int) -> None:
        """Write the counters of the link once per second of run time."""
        if self._next_link_ns is None:
            self._next_link_ns = t_ns + LINK_PERIOD_NS
            return
        if t_ns < self._next_link_ns:
            return
        self._next_link_ns = t_ns + LINK_PERIOD_NS
        stats = self._stats
        self._write_line(self._log, log_format.make_line(
            t_ns, log_format.SRC_RASP, "link", {
                "rx": stats["usb_rx"], "tx": stats["usb_tx"],
                "rejected": stats["usb_rejected"], "dropped": stats["dropped"],
                "seq_gaps": stats["seq_gaps"],
            }))

    def _flush(self) -> None:
        if self._error is not None:
            return
        try:
            for handle in (self._raw, self._log, self._lidar):
                if handle is not None:
                    handle.flush()
        except OSError as error:
            self._fail(error)

    def _fail(self, error: OSError) -> None:
        """Stop logging for good: disk full, card removed. The robot goes on."""
        if self._error is None:
            self._error = str(error)
            logger.error("Log arrete, le robot continue : %s", error)

    # ── End of the run ────────────────────────────────────────────────────

    def close(self) -> dict:
        """Write what is still queued, close the files, complete meta.json.

        Returns:
            dict: the counters of the run (also in meta.json, under "stats").
        """
        if self._closed:
            return dict(self._stats)
        t_end = self._clock_ns()
        if self._com is not None:
            try:
                self._com.set_tap(None)
            except Exception as error:  # noqa: BLE001
                logger.error("Lien non detache du log : %s", error)
        self._closed = True
        self._queue.put(_STOP)
        self._writer.join(timeout=10.0)

        for handle in (self._raw, self._log, self._lidar):
            if handle is None:
                continue
            try:
                handle.flush()
                os.fsync(handle.fileno())
            except OSError as error:
                self._fail(error)
            try:
                handle.close()
            except OSError as error:
                self._fail(error)

        with self._meta_lock:
            self._meta["t_end_rasp_ns"] = t_end
            self._meta["stats"] = dict(self._stats)
            self._meta["stream_lines"] = dict(sorted(self._streams.items()))
            if self._error is not None:
                self._meta["error"] = self._error
        try:
            self._write_meta()
        except OSError as error:
            logger.error("meta.json non complete : %s", error)
        return dict(self._stats)

    def _write_meta(self) -> None:
        """Write meta.json through a temporary file, so it is never half there."""
        with self._meta_lock:
            text = json.dumps(self._meta, indent=2, ensure_ascii=False,
                              default=str)
        target = self.folder / META_FILE
        temporary = target.with_suffix(".tmp")
        temporary.write_text(text + "\n", encoding="utf-8")
        os.replace(temporary, target)


# ── Helpers ───────────────────────────────────────────────────────────────

def _new_run_folder(root: Path, name: str) -> Path:
    """Create logs/<date>_<time>_<name>, numbered if it already exists."""
    safe = "".join(char if char.isalnum() or char in "-_" else "_"
                   for char in name) or "run"
    stamp = datetime.datetime.now().strftime("%Y-%m-%d_%H%M")
    root.mkdir(parents=True, exist_ok=True)
    for index in range(1, 1000):
        suffix = "" if index == 1 else f"_{index:02d}"
        folder = root / f"{stamp}_{safe}{suffix}"
        try:
            folder.mkdir()
        except FileExistsError:
            continue
        return folder
    raise OSError(f"no free run folder under {root} for {stamp}_{safe}")


def _git_describe() -> str | None:
    """Version of the code that is running, or None when git cannot tell."""
    try:
        result = subprocess.run(
            ["git", "describe", "--always", "--dirty"],
            cwd=Path(__file__).resolve().parent,
            capture_output=True, text=True, timeout=2.0, check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    text = result.stdout.strip()
    return text if result.returncode == 0 and text else None


def load_log_config() -> dict:
    """Read "log_config" from config.json, with its defaults.

    Returns:
        dict: enabled, dir, pid_trace, log_tick, lidar_raw.
    """
    from robot1.rasp.comm.context import CONFIG_PATH

    config = {
        "enabled": True, "dir": "logs", "pid_trace": True, "log_tick": True,
        "lidar_raw": False,
    }
    try:
        with open(CONFIG_PATH, encoding="utf-8") as handle:
            config.update(json.load(handle).get("log_config", {}))
    except (OSError, ValueError) as error:
        logger.warning("log_config illisible, valeurs par defaut : %s", error)
    return config


def create_run_logger(
    name: str,
    *,
    mode: str | None = None,
    clock_ns=time.monotonic_ns,
    clock_name: str = "monotonic",
    enabled: bool | None = None,
    meta: dict | None = None,
):
    """Build the log of a run from the launch configuration.

    Whether the log is on comes from, in this order: the `enabled` argument,
    the ROBOT_LOG environment variable ("0" or "1"), then "log_config" in
    config.json. A log that cannot be created is replaced by a NullRunLogger:
    a robot never fails to start because of its log.

    Args:
        name: what the run is, used in the folder name.
        mode: execution mode; read from ROBOT_MODE when omitted.
        clock_ns: time source of the loop, in nanoseconds.
        clock_name: which clock that is.
        enabled: force the log on or off.
        meta: extra entries for meta.json.

    Returns:
        RunLogger or NullRunLogger.
    """
    config = load_log_config()
    if enabled is None:
        env = os.environ.get(ENV_ENABLE, "").strip()
        enabled = env != "0" if env else bool(config["enabled"])
    if not enabled:
        logger.info("Log du run coupe.")
        return NULL_LOGGER

    if mode is None:
        from robot1.rasp.comm.context import get_mode

        mode = get_mode()
    mask = TraceMask.NONE
    if config["pid_trace"]:
        mask |= TraceMask.PID_TRACE
    if config["log_tick"]:
        mask |= TraceMask.LOG_TICK
    try:
        return RunLogger(
            config["dir"], name, mode=mode, clock_ns=clock_ns,
            clock_name=clock_name, trace_mask=mask,
            lidar_raw=bool(config["lidar_raw"]), meta=meta,
        )
    except OSError as error:
        logger.error("Log du run impossible, le robot demarre sans : %s", error)
        return NULL_LOGGER
