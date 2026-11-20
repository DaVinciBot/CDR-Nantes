"""Load a run folder into one pandas table per stream, with its checks.

    from robot1.replay.load import load_run
    run = load_run("logs/2027-01-06_1432_otos_lin_x_01")
    run.stream("ctrl")[["t", "target_x", "pose_x"]]

Every table has the common time "t" (s, time base of the Teensy, see align.py),
"segment" (restarts of the Teensy), "t_rasp_ns" and "src", then the fields of
the stream. The array fields of the protocol are split into named columns:
pose and target as _x, _y, _theta, velocities as _vx, _vy, _omega, wheels as
_1, _2, _3, raw optical axes as _x, _y, _h. Field names come from the decoder
(usb_com PAYLOAD_FIELDS), never from a copy here.

A partial run loads: a stream that was not recorded is an empty table and a
line in run.warnings, never an exception.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import pandas as pd

from robot1.rasp.telemetry import log_format, log_reader, trial
from robot1.rasp.telemetry.logger import LIDAR_FILE

from . import align

COMMON_COLUMNS = ("t", "segment", "t_rasp_ns", "src")

_POSE = ("x", "y", "theta")
_VELOCITY = ("vx", "vy", "omega")
_WHEELS = ("1", "2", "3")
_SENSOR_AXES = ("x", "y", "h")

# Names of the parts of each array field. One place only: the report and the
# Rerun export read the columns by these names.
ARRAY_SUFFIXES: dict[str, tuple[str, ...]] = {
    "target": _POSE, "pose": _POSE, "est_pose": _POSE, "value": _POSE,
    "p_term": _POSE, "i_term": _POSE, "d_term": _POSE,
    "velocity_cmd": _VELOCITY, "est_vel": _VELOCITY,
    "wheel_requested": _WHEELS, "wheel_cmd": _WHEELS,
    "wheel_measured": _WHEELS, "d": _WHEELS, "enc_counts": _WHEELS,
    "rs485_fail": _WHEELS,
    "otos_pos": _SENSOR_AXES, "otos_vel": _SENSOR_AXES,
    "otos_acc": _SENSOR_AXES,
}

# Streams that carry a per-message sequence number, and the message they
# come from: one line per message is enough to follow it.
SEQ_STREAMS = {"est_in": "LOG_TICK", "event": "LOG_EVENT"}

TRACE_STREAM = "ctrl"
TICK_STREAMS = tuple(log_format.LOG_TICK_STREAMS)
LIDAR_STREAM = "lidar_scan"


@dataclass
class Run:
    """One run, loaded. See the module docstring."""

    folder: Path
    meta: dict
    truth: dict | None
    alignment: align.Alignment
    check: log_reader.RunCheck | None
    frames: dict = field(default_factory=dict)
    warnings: list = field(default_factory=list)
    counters: dict = field(default_factory=dict)

    def has(self, stream: str) -> bool:
        """True when the stream has at least one line."""
        frame = self.frames.get(stream)
        return frame is not None and not frame.empty

    def stream(self, stream: str) -> pd.DataFrame:
        """Table of one stream; empty, with the common columns, if absent."""
        frame = self.frames.get(stream)
        if frame is None:
            return pd.DataFrame(columns=list(COMMON_COLUMNS))
        return frame

    def tick_table(self) -> pd.DataFrame:
        """The blocks of each LOG_TICK joined back into one row per tick.

        Returns:
            DataFrame: one row per (segment, tick), the header columns once.
        """
        present = [name for name in TICK_STREAMS if self.has(name)]
        if not present:
            return pd.DataFrame(columns=list(COMMON_COLUMNS))
        keys = ["segment", "tick"]
        shared = set(COMMON_COLUMNS) | set(log_format.LOG_TICK_HEADER)
        table = self.frames[present[0]]
        for name in present[1:]:
            frame = self.frames[name]
            own = [column for column in frame.columns
                   if column not in shared or column in keys]
            table = table.merge(frame[own], on=keys, how="outer")
        return table.sort_values("t", kind="stable").reset_index(drop=True)


def flatten(line: dict) -> dict:
    """One line as a flat row: array fields split, "stream" dropped."""
    row = {}
    for key, value in line.items():
        if key == "stream":
            continue
        names = ARRAY_SUFFIXES.get(key)
        if (names and isinstance(value, list) and len(value) == len(names)
                and all(not isinstance(item, (list, dict)) for item in value)):
            for name, item in zip(names, value):
                row[f"{key}_{name}"] = item
        else:
            row[key] = value
    return row


def build_frames(lines: list[dict]) -> dict[str, pd.DataFrame]:
    """Group aligned lines by stream, one table each, in file order."""
    grouped: dict[str, list[dict]] = {}
    for line in lines:
        grouped.setdefault(line.get("stream"), []).append(flatten(line))
    return {
        stream: pd.DataFrame.from_records(rows)
        for stream, rows in grouped.items()
    }


def count_seq_gaps(frame: pd.DataFrame) -> int:
    """Messages missing between consecutive seq (u16), within each segment."""
    if frame.empty or "seq" not in frame:
        return 0
    missing = 0
    for _, group in frame.groupby("segment", sort=False):
        seq = group["seq"].to_numpy(dtype="int64")
        if len(seq) > 1:
            missing += int((((seq[1:] - seq[:-1]) - 1) & 0xFFFF).sum())
    return missing


def count_incomplete_ticks(trace: pd.DataFrame, ticks: pd.DataFrame) -> int:
    """Ticks with only one of PID_TRACE and LOG_TICK, while both were on.

    The two frames of a tick share its t_us. Only the time both streams
    cover is compared: they start and stop at slightly different ticks.
    """
    if trace.empty or ticks.empty:
        return 0
    incomplete = 0
    for segment, group in ticks.groupby("segment", sort=False):
        other = trace[trace["segment"] == segment]
        if other.empty:
            continue
        low = max(group["t_us"].min(), other["t_us"].min())
        high = min(group["t_us"].max(), other["t_us"].max())
        mine = set(group["t_us"][(group["t_us"] >= low) & (group["t_us"] <= high)])
        theirs = set(other["t_us"][(other["t_us"] >= low)
                                   & (other["t_us"] <= high)])
        incomplete += len(mine ^ theirs)
    return incomplete


def _checks(run: Run) -> None:
    """Fill run.warnings and run.counters."""
    if run.check is not None:
        run.warnings.extend(f"log : {problem}" for problem in run.check.problems)
    if run.alignment.restarts:
        run.warnings.append(
            f"{run.alignment.restarts} redemarrage(s) de la Teensy pendant le "
            f"run : temps aligne par segment")

    gaps = {}
    for stream, message in SEQ_STREAMS.items():
        gaps[message] = count_seq_gaps(run.stream(stream))
        if gaps[message]:
            run.warnings.append(f"{gaps[message]} {message} manquants "
                                f"(trous de seq)")
    run.counters["seq_gaps"] = gaps

    asked = run.meta.get("streams", {})
    if asked.get("pid_trace") and not run.has(TRACE_STREAM):
        run.warnings.append("PID_TRACE demandee, aucune ligne ctrl")
    if asked.get("log_tick") and not run.has(TICK_STREAMS[0]):
        run.warnings.append("LOG_TICK demande, aucune ligne recue")
    if asked.get("lidar_raw") and not run.has(LIDAR_STREAM):
        run.warnings.append("scans LiDAR demandes, aucun recu")

    incomplete = 0
    if asked.get("pid_trace") and asked.get("log_tick"):
        incomplete = count_incomplete_ticks(run.stream(TRACE_STREAM),
                                            run.stream(TICK_STREAMS[0]))
        if incomplete:
            run.warnings.append(f"{incomplete} ticks incomplets "
                                f"(PID_TRACE ou LOG_TICK seul)")
    run.counters["incomplete_ticks"] = incomplete

    stats = run.meta.get("stats") or {}
    run.counters["dropped"] = stats.get("dropped", 0)


def _load_scans(folder: Path, alignment: align.Alignment) -> list[dict]:
    """Raw LiDAR scans, dated on the common time, or [] when not kept."""
    if not (folder / LIDAR_FILE).exists():
        return []
    scans = log_reader.read_lines(folder, LIDAR_FILE)
    for scan in scans:
        scan["t"] = float(alignment.rasp_to_t(scan["t_rasp_ns"] * 1e-9))
        scan["segment"] = 0
    return scans


def load_run(folder, *, verify: bool = True) -> Run:
    """Load a run folder. See the module docstring.

    Args:
        folder: the run folder.
        verify: also run log_reader.check_run() (decodes raw_usb.bin again:
            a few seconds on a long match). Its problems go to run.warnings.

    Returns:
        Run: tables, alignment, checks.
    """
    folder = Path(folder)
    meta = log_reader.read_meta(folder)
    lines = log_reader.read_lines(folder)
    alignment = align.align_lines(lines, meta)
    lines.extend(_load_scans(folder, alignment))

    run = Run(
        folder=folder,
        meta=meta,
        truth=trial.read_truth(folder),
        alignment=alignment,
        check=log_reader.check_run(folder) if verify else None,
        frames=build_frames(lines),
    )
    _checks(run)
    return run
