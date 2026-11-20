"""Export a run to a Rerun recording (.rrd), on the common time of align.py.

    python -m robot1.replay.to_rerun logs/2027-01-06_1432_match
    python -m robot1.replay.to_rerun logs/... --out match.rrd --spawn

Replays a real match in the viewer without sim2d. The 3D view uses the entity
paths of telemetry/rerun_bridge.py, so a recording and the live view read the
same way:

    world/robot/odom              ODOMETRY pose
    world/robot/fused             best estimate of the match loop (state)
    world/robot/reference         reference pose of the controller (PID_TRACE)
    world/pathfinding/target      target of the match loop (state)
    world/pathfinding/trajectory  planned path (state)
    world/obstacles/adversaries   obstacles (state)
    world/truth/final             ground truth end pose (truth.json)

Curves are at the full rate of each stream, not decimated to the 20 Hz of the
bridge: data/<stream>/... for every numeric field. An array field of the
protocol becomes one entity per part, so a plot shows target against pose:
data/ctrl/x/target and data/ctrl/x/pose, data/ctrl/roue_1/wheel_cmd and
data/ctrl/roue_1/wheel_measured. Events, strategy and match lines go to
logs/<stream> as text.

Timelines: "teensy_time" (s, the common time) for everything, "tick" for the
LOG_TICK streams.

The recording is written next to the run (replay.rrd) unless --out says
otherwise. log.jsonl and raw_usb.bin are not touched.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import rerun as rr
import rerun.blueprint as rrb

from robot1.rasp.telemetry import rerun_bridge
from robot1.rasp.telemetry.table_map import log_static_map

from .load import ARRAY_SUFFIXES, COMMON_COLUMNS, TICK_STREAMS, Run, load_run

RRD_FILE = "replay.rrd"
TIMELINE = "teensy_time"
TICK_TIMELINE = "tick"

# Streams drawn as curves. Every numeric column is sent, header excepted.
CURVE_STREAMS = ("odometry", "status", "ctrl", *TICK_STREAMS)
# Streams written as text lines.
TEXT_STREAMS = ("event", "strat", "match", "path", "print", "opp")
_NOT_CURVES = set(COMMON_COLUMNS) | {"schema_version", "seq", "tick", "t_us"}

C_REFERENCE = [255, 160, 0, 255]
C_TRUTH = [0, 200, 255, 255]
WORLD_Z_MM = 60.0
# The 3D view is drawn one call per row: decimated to this rate, which the eye
# cannot tell from the full one. The curves are never decimated.
WORLD_RATE_HZ = 30.0


def _decimate(frame: pd.DataFrame, rate_hz: float = WORLD_RATE_HZ) -> pd.DataFrame:
    """First row of each 1 / rate_hz slice of the common time."""
    if frame.empty:
        return frame
    slices = np.floor(frame["t"].to_numpy(float) * rate_hz)
    keep = np.concatenate(([True], slices[1:] != slices[:-1]))
    return frame[keep]


def _column_entity(stream: str, column: str) -> str:
    """Entity path of one numeric column of a stream."""
    field, _, part = column.rpartition("_")
    if field in ARRAY_SUFFIXES and part in ARRAY_SUFFIXES[field]:
        group = f"roue_{part}" if part.isdigit() else part
        return f"data/{stream}/{group}/{field}"
    return f"data/{stream}/{column}"


def _send_curves(stream: str, frame: pd.DataFrame) -> list[str]:
    """Every numeric column of a stream as a scalar series."""
    indexes = [rr.TimeColumn(TIMELINE, duration=frame["t"].to_numpy(float))]
    if "tick" in frame:
        indexes.append(rr.TimeColumn(
            TICK_TIMELINE, sequence=frame["tick"].to_numpy(np.int64)))
    sent = []
    for column in frame.columns:
        if column in _NOT_CURVES or not pd.api.types.is_numeric_dtype(frame[column]):
            continue
        entity = _column_entity(stream, column)
        rr.send_columns(entity, indexes=indexes,
                        columns=rr.Scalars.columns(
                            scalars=frame[column].to_numpy(float)))
        sent.append(entity)
    return sent


def _send_odometry_view(frame: pd.DataFrame) -> list[str]:
    """The curves of the live bridge, under the same names."""
    times = [rr.TimeColumn(TIMELINE, duration=frame["t"].to_numpy(float))]
    series = {
        "data/teensy/x_mm": frame["x"],
        "data/teensy/y_mm": frame["y"],
        "data/teensy/theta_deg": np.degrees(frame["theta"]),
    }
    for entity, values in series.items():
        rr.send_columns(entity, indexes=times,
                        columns=rr.Scalars.columns(
                            scalars=np.asarray(values, float)))
    return list(series)


def _send_text(stream: str, frame: pd.DataFrame) -> str:
    """One text line per row: the fields of the stream, as JSON."""
    texts = []
    for row in frame.to_dict(orient="records"):
        fields = {key: value for key, value in row.items()
                  if key not in COMMON_COLUMNS and not _missing(value)}
        texts.append(json.dumps(fields, ensure_ascii=False, default=str))
    entity = f"logs/{stream}"
    rr.send_columns(
        entity,
        indexes=[rr.TimeColumn(TIMELINE, duration=frame["t"].to_numpy(float))],
        columns=rr.TextLog.columns(text=texts),
    )
    return entity


def _missing(value) -> bool:
    """A cell that pandas filled because the row has no such field."""
    return isinstance(value, float) and math.isnan(value)


def _draw_poses(frame: pd.DataFrame, path: str, columns, color,
                cylinder: bool = False) -> None:
    """A robot per row of a table, on the common time."""
    draw = (rerun_bridge._log_robot_cylinder if cylinder
            else rerun_bridge._log_robot_point)
    frame = _decimate(frame)
    for t, x, y, theta in frame[["t", *columns]].itertuples(index=False):
        if _missing(x) or _missing(y):
            continue
        rr.set_time(TIMELINE, duration=float(t))
        draw(path, float(x), float(y), float(0.0 if _missing(theta) else theta),
             color)


def _draw_state(frame: pd.DataFrame) -> list[str]:
    """Target, path, obstacles and best estimate of the match loop."""
    drawn = []
    columns = set(frame.columns)
    if {"fused_x", "fused_y", "fused_theta"} <= columns:
        _draw_poses(frame, "world/robot/fused",
                    ("fused_x", "fused_y", "fused_theta"),
                    rerun_bridge.C_FUSED, cylinder=True)
        drawn.append("world/robot/fused")
    for row in _decimate(frame).to_dict(orient="records"):
        rr.set_time(TIMELINE, duration=float(row["t"]))
        if "target_x" in row and not _missing(row["target_x"]):
            rr.log("world/pathfinding/target", rr.Points3D(
                positions=[[row["target_x"], row["target_y"], WORLD_Z_MM]],
                colors=[rerun_bridge.C_TARGET], radii=[40.0]))
        trajectory = row.get("trajectory")
        if isinstance(trajectory, list) and len(trajectory) >= 2:
            rr.log("world/pathfinding/trajectory", rr.LineStrips3D(
                [[[p[0], p[1], 10.0] for p in trajectory]],
                colors=[rerun_bridge.C_TRAJ]))
        elif isinstance(trajectory, list):
            rr.log("world/pathfinding/trajectory", rr.Clear(recursive=False))
        obstacles = row.get("obstacles")
        if isinstance(obstacles, list) and obstacles:
            rr.log("world/obstacles/adversaries", rr.Points3D(
                positions=[[o["x"], o["y"], 100.0] for o in obstacles],
                radii=[o.get("radius", 100.0) for o in obstacles],
                colors=[rerun_bridge.C_OBS]))
        elif isinstance(obstacles, list):
            rr.log("world/obstacles/adversaries", rr.Clear(recursive=False))
    drawn += ["world/pathfinding/target", "world/pathfinding/trajectory",
              "world/obstacles/adversaries"]
    return drawn


def _draw_truth(run: Run) -> list[str]:
    """Ground truth end pose, at the end of the run."""
    truth = run.truth or {}
    if truth.get("kind") != "pose" or not run.frames:
        return []
    end = max(frame["t"].max() for frame in run.frames.values()
              if not frame.empty)
    x, y, theta = truth["final_pose_table"]
    rr.set_time(TIMELINE, duration=float(end))
    rerun_bridge._log_robot_point("world/truth/final", x, y, theta, C_TRUTH)
    return ["world/truth/final"]


def blueprint() -> rrb.Blueprint:
    """Table and events on top, the curves of the controller in tabs."""
    curves = [
        rrb.TimeSeriesView(name="Suivi x", origin="data/ctrl/x"),
        rrb.TimeSeriesView(name="Suivi y", origin="data/ctrl/y"),
        rrb.TimeSeriesView(name="Suivi theta", origin="data/ctrl/theta"),
        rrb.TimeSeriesView(name="Roue 1", origin="data/ctrl/roue_1"),
        rrb.TimeSeriesView(name="Roue 2", origin="data/ctrl/roue_2"),
        rrb.TimeSeriesView(name="Roue 3", origin="data/ctrl/roue_3"),
        rrb.TimeSeriesView(name="Odometrie", origin="data/teensy"),
        rrb.TimeSeriesView(name="Estimateur", origin="data/est"),
        rrb.TimeSeriesView(name="Sante", origin="data/health"),
    ]
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(name="Terrain", origin="world"),
                rrb.TextLogView(name="Evenements", origin="logs"),
                column_shares=[3, 1],
            ),
            rrb.Tabs(*curves),
            row_shares=[3, 2],
        ),
        collapse_panels=True,
    )


def export(run: Run, output) -> list[str]:
    """Write a run to a .rrd file.

    Args:
        run: the run, from load.load_run().
        output: path of the .rrd to write.

    Returns:
        list[str]: the entity paths written, table excepted.
    """
    rr.init("cdr_replay", recording_id=run.folder.name)
    rr.save(str(output), default_blueprint=blueprint())
    log_static_map()

    written = []
    odometry = run.stream("odometry")
    if not odometry.empty:
        written += _send_odometry_view(odometry)
        _draw_poses(odometry, "world/robot/odom", ("x", "y", "theta"),
                    rerun_bridge.C_ROBOT)
        written.append("world/robot/odom")
    ctrl = run.stream("ctrl")
    if not ctrl.empty:
        _draw_poses(ctrl, "world/robot/reference",
                    ("target_x", "target_y", "target_theta"), C_REFERENCE)
        written.append("world/robot/reference")
    state = run.stream("state")
    if not state.empty:
        written += _draw_state(state)
    written += _draw_truth(run)

    for stream in CURVE_STREAMS:
        if run.has(stream):
            written += _send_curves(stream, run.stream(stream))
    for stream in TEXT_STREAMS:
        if run.has(stream):
            written.append(_send_text(stream, run.stream(stream)))

    rr.disconnect()
    return written


def main() -> int:
    """Export a run folder from the command line."""
    parser = argparse.ArgumentParser(
        description="Exporte un dossier de run vers un enregistrement Rerun.",
    )
    parser.add_argument("folder", help="dossier du run")
    parser.add_argument("--out", help=f"fichier .rrd (defaut : {RRD_FILE} "
                                      f"dans le dossier du run)")
    parser.add_argument("--spawn", action="store_true",
                        help="ouvrir le viewer Rerun sur l'enregistrement")
    args = parser.parse_args()

    run = load_run(args.folder, verify=False)
    output = Path(args.out) if args.out else Path(args.folder) / RRD_FILE
    for warning in run.warnings:
        print(f"ATTENTION : {warning}")
    written = export(run, output)
    print(f"Ecrit : {output} ({len(written)} entites)")
    if args.spawn:
        subprocess.Popen([sys.executable, "-m", "rerun", str(output)])
    return 0


if __name__ == "__main__":
    sys.exit(main())
