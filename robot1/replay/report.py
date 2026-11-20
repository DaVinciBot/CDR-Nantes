"""Report of one run: link health, cycles, smoothness, step response.

    python -m robot1.replay.report logs/2027-01-06_1432_sequence_01
    python -m robot1.replay.report logs/... --no-write

Prints the report and writes report.json in the run folder. Every section
says what it needs: a stream that was not recorded makes it "non disponible",
with the stream named, never an error. A run of sim2d has no LOG_TICK, so no
cycle health nor command latency; a run of the robot with PID_TRACE off has
no step response on the wheels.

Sections:

  lien        verdict of log_reader, rejected frames, NACK, sequence gaps,
              lines dropped by the logger, incomplete ticks;
  cycles      work_us of the tick against its period, overruns, RS485
              failures per wheel and per cause, OTOS read time (LOG_TICK);
  commandes   latency between a SET_TARGET sent and the tick that took it,
              paired by rank (cmd_count of LOG_TICK);
  suivi       tracking error target - pose, acceleration and jerk of the
              command and of the motion, saturations, lost steps (PID_TRACE);
  echelons    response to each step of the target: 10-90 % rise time,
              overshoot, static error, settling time (PID_TRACE, or SET_TARGET
              and ODOMETRY);
  roues       pure delay, time constant and gain of each wheel, fitted on
              the command and the measured speed (PID_TRACE);
  verite      estimated end pose (or turns) against the ground truth.

The thresholds below are first values, to revisit with the first real runs.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from usb_com.python.messages import LogHealthFlag, Rs485Fail, TraceFlag

from .load import Run, load_run

REPORT_FILE = "report.json"

DEFAULT_PERIOD_US = 15_000

# A target that jumps this much between two ticks is a step, not a profile
# (1 m/s moves the reference 15 mm per tick).
STEP_MIN_MM = 40.0
STEP_MIN_RAD = 0.2
# A step is measured while its target holds, within this; a target held for
# less than STEP_MIN_WINDOW_S is not measured.
TARGET_HOLD_MM = 1.0
TARGET_HOLD_RAD = 0.005
STEP_MIN_WINDOW_S = 0.3
# Settled: within this share of the step, or this absolute band.
SETTLE_SHARE = 0.02
SETTLE_MIN_MM = 5.0
SETTLE_MIN_RAD = 0.01

# Moving, for the tracking error: commanded speed above this.
MOVING_MM_S = 5.0
MOVING_RAD_S = 0.02

# Lost step: command and measure of a wheel apart by more than this, for this
# many ticks in a row.
STEP_LOSS_RPM = 50.0
STEP_LOSS_TICKS = 20

# Wheel fit: the command must move at least this much, delays tried up to this.
MIN_EXCITATION_RPM = 5.0
MAX_DELAY_TICKS = 10

U32 = 0xFFFFFFFF


def unavailable(reason: str) -> dict:
    """A section that cannot be computed, and why."""
    return {"available": False, "reason": reason}


def _wrap(angle):
    """Wrap angles to [-pi, pi]."""
    return np.arctan2(np.sin(angle), np.cos(angle))


def _stats(values) -> dict:
    """p50, p95, max (and count) of a series, NaN removed."""
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {"n": 0}
    return {
        "n": int(values.size),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "max": float(values.max()),
    }


def _period_s(run: Run) -> float:
    """Control period of the Teensy, from LOG_PARAMS when it came."""
    params = run.meta.get("teensy_params") or {}
    return (params.get("period_us") or DEFAULT_PERIOD_US) * 1e-6


def _episodes(mask: np.ndarray, min_length: int = 1) -> int:
    """Runs of True at least min_length long."""
    count = 0
    length = 0
    for value in mask:
        length = length + 1 if value else 0
        if length == min_length:
            count += 1
    return count


# ── Sections ──────────────────────────────────────────────────────────────

def section_link(run: Run) -> dict:
    """Health of the link and of the log."""
    check = run.check
    stats = run.meta.get("stats") or {}
    return {
        "available": True,
        "ok": check.ok if check is not None else None,
        "verdict": check.summary() if check is not None else None,
        "messages": check.messages if check is not None else None,
        "rejected": len(run.stream("rejected")),
        "nack": len(run.stream("nack")),
        "undecoded": len(run.stream("raw")),
        "seq_gaps": run.counters.get("seq_gaps", {}),
        "dropped": stats.get("dropped", 0),
        "incomplete_ticks": run.counters.get("incomplete_ticks", 0),
        "streams": {name: len(frame) for name, frame in
                    sorted(run.frames.items(), key=lambda item: str(item[0]))},
    }


def section_cycles(run: Run) -> dict:
    """Work of the tick and failures of the sensors (LOG_TICK health)."""
    health = run.stream("health")
    if health.empty:
        return unavailable("flux health absent (LOG_TICK)")
    period_us = _period_s(run) * 1e6
    work = health["work_us"].to_numpy(dtype=float)
    flags = health["health_flags"].to_numpy(dtype=np.int64)

    rs485 = {}
    for wheel in (1, 2, 3):
        column = f"rs485_fail_{wheel}"
        if column not in health:
            continue
        causes = {}
        for value, count in health[column].value_counts().items():
            if value == Rs485Fail.OK.value:
                continue
            try:
                name = Rs485Fail(int(value)).name
            except ValueError:
                name = f"code_{int(value)}"
            causes[name] = int(count)
        rs485[str(wheel)] = causes

    return {
        "available": True,
        "ticks": int(len(health)),
        "period_us": period_us,
        "work_us": _stats(work),
        "work_p99_us": float(np.percentile(work, 99)),
        "budget_used_p99": float(np.percentile(work, 99) / period_us),
        "overruns": int(((flags & LogHealthFlag.OVERRUN) != 0).sum()),
        "writes_failed": int(((flags & LogHealthFlag.WRITE_FAILED) != 0).sum()),
        "log_dropped": int(health["log_dropped"].iloc[-1]),
        "rs485_fail": rs485,
        "otos_read_us": _stats(health["otos_read_us"]),
    }


def section_commands(run: Run) -> dict:
    """Latency of SET_TARGET, sent by the Pi and taken by a tick."""
    received = run.stream("cmd_rx")
    if received.empty:
        return unavailable("flux cmd_rx absent (LOG_TICK)")
    sent = run.stream("cmd_tx")
    if sent.empty or "msg" not in sent:
        return unavailable("aucun message envoye")
    sent = sent[sent["msg"] == "SET_TARGET"]
    if sent.empty:
        return unavailable("aucun SET_TARGET envoye")

    latencies = []
    for segment, rows in received.groupby("segment", sort=False):
        first = rows.iloc[0]
        base = int(first["cmd_count"])
        after = sent[(sent["segment"] == segment) & (sent["t"] > first["t"])]
        times = after["t"].to_numpy()
        previous = base
        for row in rows.itertuples(index=False):
            count = int(row.cmd_count)
            if count == previous:
                continue
            previous = count
            rank = count - base
            if not 1 <= rank <= len(times):
                continue
            # cmd_t_us is the micros() of the last command taken; the line
            # is dated by its own t_us.
            age_s = ((int(row.t_us) - int(row.cmd_t_us)) & U32) * 1e-6
            latencies.append((row.t - age_s - times[rank - 1]) * 1e3)

    if not latencies:
        return unavailable("aucun SET_TARGET apparie")
    negative = sum(1 for value in latencies if value < 0.0)
    return {
        "available": True,
        "paired": len(latencies),
        "latency_ms": _stats(latencies),
        "negative": negative,
        # A negative latency means the ranks slipped: a command lost, or
        # one sent before the first LOG_TICK and taken after it.
        "pairing_doubtful": negative > 0.1 * len(latencies),
    }


def _derivatives(t, vx, vy) -> tuple:
    """Acceleration and jerk norms of a planar velocity, by differences."""
    t = np.asarray(t, dtype=float)
    if len(t) < 3:
        return np.array([]), np.array([])
    dt = np.diff(t)
    dt[dt <= 0.0] = np.nan
    ax = np.diff(vx) / dt
    ay = np.diff(vy) / dt
    accel = np.hypot(ax, ay)
    t_mid = 0.5 * (t[1:] + t[:-1])
    dt2 = np.diff(t_mid)
    dt2[dt2 <= 0.0] = np.nan
    jerk = np.hypot(np.diff(ax) / dt2, np.diff(ay) / dt2)
    return accel, jerk


def section_tracking(run: Run) -> dict:
    """Tracking error, smoothness, saturations, lost steps (PID_TRACE)."""
    ctrl = run.stream("ctrl")
    if ctrl.empty:
        return unavailable("flux ctrl absent (PID_TRACE)")

    error_xy = np.hypot(ctrl["target_x"] - ctrl["pose_x"],
                        ctrl["target_y"] - ctrl["pose_y"]).to_numpy()
    error_theta = np.abs(_wrap((ctrl["target_theta"]
                                - ctrl["pose_theta"]).to_numpy()))
    moving = ((np.hypot(ctrl["velocity_cmd_vx"], ctrl["velocity_cmd_vy"])
               > MOVING_MM_S)
              | (ctrl["velocity_cmd_omega"].abs() > MOVING_RAD_S)).to_numpy()

    def rms(values):
        values = values[np.isfinite(values)]
        return float(np.sqrt(np.mean(values ** 2))) if values.size else None

    cmd_accel, cmd_jerk = _derivatives(ctrl["t"], ctrl["velocity_cmd_vx"],
                                       ctrl["velocity_cmd_vy"])
    motion = {}
    odometry = run.stream("odometry")
    if not odometry.empty:
        accel, jerk = _derivatives(odometry["t"], odometry["vx"],
                                   odometry["vy"])
        motion = {"accel_mm_s2": _stats(accel), "jerk_mm_s3": _stats(jerk)}

    flags = ctrl["flags"].to_numpy(dtype=np.int64)
    saturations = {}
    for flag in TraceFlag:
        mask = (flags & flag) != 0
        saturations[flag.name] = {"ticks": int(mask.sum()),
                                  "episodes": _episodes(mask)}

    lost = {}
    for wheel in (1, 2, 3):
        gap = (ctrl[f"wheel_cmd_{wheel}"]
               - ctrl[f"wheel_measured_{wheel}"]).abs().to_numpy()
        lost[str(wheel)] = _episodes(gap > STEP_LOSS_RPM, STEP_LOSS_TICKS)

    return {
        "available": True,
        "ticks": int(len(ctrl)),
        "moving_ticks": int(moving.sum()),
        "error_mm": {"rms": rms(error_xy), "max": float(np.nanmax(error_xy)),
                     "rms_moving": rms(error_xy[moving])},
        "error_rad": {"rms": rms(error_theta),
                      "max": float(np.nanmax(error_theta))},
        "command": {"accel_mm_s2": _stats(cmd_accel),
                    "jerk_mm_s3": _stats(cmd_jerk)},
        "motion": motion,
        "saturations": saturations,
        "lost_steps": lost,
    }


# ── Step response of the pose ─────────────────────────────────────────────

def _step_sources(run: Run):
    """Targets and poses to measure the steps on, and where they come from.

    Returns:
        tuple: (name, targets, poses) or None. targets: t, x, y, theta and
        "may open a step" (1 or 0) of every target in order; poses: t, x, y,
        theta.
    """
    ctrl = run.stream("ctrl")
    if not ctrl.empty:
        targets = ctrl[["t", "target_x", "target_y", "target_theta"]]
        targets = np.column_stack((targets.to_numpy(float),
                                   np.ones(len(targets))))
        poses = ctrl[["t", "pose_x", "pose_y", "pose_theta"]]
        return "ctrl", targets, poses.to_numpy(float)
    sent = run.stream("cmd_tx")
    odometry = run.stream("odometry")
    if sent.empty or odometry.empty or "msg" not in sent:
        return None
    sent = sent[sent["msg"] == "SET_TARGET"]
    if sent.empty:
        return None
    # A target with a velocity is a point of a trajectory, not a step; it
    # still ends the hold of the step before it.
    plain = ((sent["vx"] == 0.0) & (sent["vy"] == 0.0)
             & (sent["omega"] == 0.0)).to_numpy(float)
    targets = np.column_stack((sent[["t", "x", "y", "theta"]].to_numpy(float),
                               plain))
    return ("cmd_tx+odometry", targets,
            odometry[["t", "x", "y", "theta"]].to_numpy(float))


def detect_steps(targets: np.ndarray) -> list[tuple]:
    """Jumps of the target: (time, index, translation?, rotation?)."""
    steps = []
    for index in range(1, len(targets)):
        if not targets[index, 4]:
            continue
        jump_xy = math.hypot(targets[index, 1] - targets[index - 1, 1],
                             targets[index, 2] - targets[index - 1, 2])
        jump_theta = abs(float(_wrap(targets[index, 3] - targets[index - 1, 3])))
        if jump_xy >= STEP_MIN_MM or jump_theta >= STEP_MIN_RAD:
            steps.append((targets[index, 0], index, jump_xy >= STEP_MIN_MM,
                          jump_theta >= STEP_MIN_RAD))
    return steps


def hold_end(targets: np.ndarray, index: int, end: float) -> float:
    """Time the target set at index stops holding still (or end of run)."""
    moved_xy = np.hypot(targets[index + 1:, 1] - targets[index, 1],
                        targets[index + 1:, 2] - targets[index, 2])
    moved_theta = np.abs(_wrap(targets[index + 1:, 3] - targets[index, 3]))
    changed = np.flatnonzero((moved_xy > TARGET_HOLD_MM)
                             | (moved_theta > TARGET_HOLD_RAD))
    return float(targets[index + 1 + changed[0], 0]) if changed.size else end


def step_metrics(t, progress, t_step: float, amplitude: float,
                 band: float) -> dict:
    """Metrics of one step response, given its normalised progress.

    Args:
        t: times of the samples after the step, s.
        progress: (pose - pose at step) / step along the step: 0 then 1.
        t_step: time of the step, s.
        amplitude: size of the step (mm or rad), for the absolute errors.
        band: settling band, in the same unit as amplitude.

    Returns:
        dict: rise_s (10-90 %), overshoot_pct, static_error, settling_s,
        start_s (5 % reached). None where not reached.
    """
    t = np.asarray(t, dtype=float)
    progress = np.asarray(progress, dtype=float)

    def first_time(mask):
        hits = np.flatnonzero(mask)
        return float(t[hits[0]]) if hits.size else None

    t10 = first_time(progress >= 0.1)
    t90 = first_time(progress >= 0.9)
    start = first_time(progress >= 0.05)
    outside = np.abs(1.0 - progress) * amplitude > band
    settling = None
    if not outside[-1]:
        last_out = np.flatnonzero(outside)
        index = last_out[-1] + 1 if last_out.size else 0
        settling = float(t[index] - t_step)
    return {
        "rise_s": t90 - t10 if t10 is not None and t90 is not None else None,
        "overshoot_pct": float(max(0.0, (progress.max() - 1.0) * 100.0)),
        "static_error": float((1.0 - progress[-1]) * amplitude),
        "settling_s": settling,
        "start_s": start - t_step if start is not None else None,
    }


def section_steps(run: Run) -> dict:
    """Response of the pose to each step of the target."""
    source = _step_sources(run)
    if source is None:
        return unavailable("ni ctrl (PID_TRACE), ni SET_TARGET + odometry")
    name, targets, poses = source
    steps = detect_steps(targets)
    if not steps:
        return unavailable("aucun echelon de cible dans le run")

    end = poses[-1, 0]
    measured = []
    for t_step, index, translation, rotation in steps:
        t_next = hold_end(targets, index, end)
        if t_next - t_step < STEP_MIN_WINDOW_S:
            continue
        before = poses[poses[:, 0] <= t_step]
        window = poses[(poses[:, 0] > t_step) & (poses[:, 0] < t_next)]
        if before.size == 0 or len(window) < 3:
            continue
        start = before[-1]
        target = targets[index]
        if translation:
            dx, dy = target[1] - start[1], target[2] - start[2]
            length = math.hypot(dx, dy)
            if length >= STEP_MIN_MM:
                ux, uy = dx / length, dy / length
                along = ((window[:, 1] - start[1]) * ux
                         + (window[:, 2] - start[2]) * uy)
                lateral = (-(window[:, 1] - start[1]) * uy
                           + (window[:, 2] - start[2]) * ux)
                metrics = step_metrics(
                    window[:, 0], along / length, t_step, length,
                    max(SETTLE_SHARE * length, SETTLE_MIN_MM))
                metrics.update(kind="translation", t=float(t_step),
                               size=float(length),
                               lateral_max_mm=float(np.abs(lateral).max()))
                measured.append(metrics)
        if rotation:
            turn = float(_wrap(target[3] - start[3]))
            if abs(turn) >= STEP_MIN_RAD:
                turned = np.unwrap(np.concatenate(([start[3]], window[:, 3])))
                metrics = step_metrics(
                    window[:, 0], (turned[1:] - start[3]) / turn, t_step,
                    abs(turn), max(SETTLE_SHARE * abs(turn), SETTLE_MIN_RAD))
                metrics.update(kind="rotation", t=float(t_step),
                               size=float(turn))
                measured.append(metrics)

    if not measured:
        return unavailable("aucune cible tenue assez longtemps pour mesurer "
                           "un echelon")
    return {"available": True, "source": name, "steps": measured}


# ── Wheels: pure delay and time constant ──────────────────────────────────

def fit_first_order(command, measured, dt: float,
                    max_delay: int = MAX_DELAY_TICKS) -> dict | None:
    """Fit measured[k+1] = a measured[k] + b command[k - d], by least squares.

    The delay d is tried from 0 to max_delay ticks, the one with the smallest
    residual is kept. The measure is taken at the start of a tick and shows
    the command of the period before: d = 0 already means one tick between a
    command and its first effect.

    Args:
        command: command of each tick (rpm), one segment, in order.
        measured: measured speed of each tick (rpm), same length.
        dt: tick period, s.
        max_delay: longest delay tried, ticks.

    Returns:
        dict: delay_ticks, delay_s, tau_s, gain, r2; None when the command
        does not move enough to tell anything.
    """
    u = np.asarray(command, dtype=float)
    y = np.asarray(measured, dtype=float)
    if len(u) < max_delay + 10 or np.nanstd(u) < MIN_EXCITATION_RPM:
        return None

    best = None
    for delay in range(max_delay + 1):
        target = y[delay + 1:]
        regressors = np.column_stack((y[delay:-1], u[:len(u) - delay - 1]))
        valid = np.isfinite(target) & np.isfinite(regressors).all(axis=1)
        if valid.sum() < 10:
            continue
        coeffs, *_ = np.linalg.lstsq(regressors[valid], target[valid],
                                     rcond=None)
        residual = target[valid] - regressors[valid] @ coeffs
        sse = float(residual @ residual)
        if best is None or sse < best[0] * (1.0 - 1e-9):
            spread = target[valid] - target[valid].mean()
            total = float(spread @ spread)
            best = (sse, delay, coeffs, 1.0 - sse / total if total else 1.0)
    if best is None:
        return None

    _, delay, (a, b), r2 = best
    tau = -dt / math.log(a) if 0.0 < a < 1.0 else 0.0
    gain = b / (1.0 - a) if a < 1.0 else float("nan")
    return {"delay_ticks": int(delay), "delay_s": delay * dt,
            "tau_s": float(tau), "gain": float(gain), "a": float(a),
            "r2": float(r2)}


def section_wheels(run: Run) -> dict:
    """Pure delay, time constant and gain of each wheel (PID_TRACE)."""
    ctrl = run.stream("ctrl")
    if ctrl.empty:
        return unavailable("flux ctrl absent (PID_TRACE)")
    # The largest segment: a restart breaks the sequence of ticks.
    segment = ctrl["segment"].value_counts().idxmax()
    rows = ctrl[ctrl["segment"] == segment]
    steps = np.diff(rows["t_us"].to_numpy(dtype=np.int64)) & U32
    dt = float(np.median(steps)) * 1e-6 if steps.size else _period_s(run)

    wheels = {}
    for wheel in (1, 2, 3):
        fit = fit_first_order(rows[f"wheel_cmd_{wheel}"],
                              rows[f"wheel_measured_{wheel}"], dt)
        wheels[str(wheel)] = fit or unavailable("roue trop peu sollicitee")
    return {"available": True, "tick_s": dt, "wheels": wheels}


# ── Ground truth ──────────────────────────────────────────────────────────

def _estimated_poses(run: Run):
    """Estimated pose over the run: (source, t, x, y, theta) or None."""
    for stream, prefix in (("est", "est_pose_"), ("odometry", ""),
                           ("ctrl", "pose_")):
        frame = run.stream(stream)
        if not frame.empty:
            columns = [f"{prefix}{axis}" for axis in ("x", "y", "theta")]
            values = frame[["t", *columns]].to_numpy(float)
            return stream, values
    return None


def section_truth(run: Run) -> dict:
    """End pose or turns of the estimate, against the ground truth."""
    truth = run.truth
    if truth is None:
        return unavailable("pas de truth.json")
    kind = truth.get("kind")
    if kind == "none":
        return unavailable(f"pas de verite terrain : {truth.get('reason')}")
    estimate = _estimated_poses(run)
    if estimate is None:
        return unavailable("aucune pose estimee dans le run")
    source, poses = estimate

    if kind == "pose":
        x, y, theta = truth["final_pose_table"]
        end = poses[-1]
        dx, dy = end[1] - x, end[2] - y
        return {
            "available": True, "kind": kind, "source": source,
            "estimated": [float(v) for v in end[1:]],
            "truth": [x, y, theta],
            "error_mm": float(math.hypot(dx, dy)),
            "error_xy_mm": [float(dx), float(dy)],
            "error_rad": float(_wrap(end[3] - theta)),
            "uncertainty_mm": truth.get("uncertainty_mm"),
        }
    if kind == "turns":
        unrolled = np.unwrap(poses[:, 3])
        turns = float((unrolled[-1] - unrolled[0]) / (2.0 * math.pi))
        return {
            "available": True, "kind": kind, "source": source,
            "estimated_turns": turns, "truth_turns": truth["turns"],
            "error_rad": float((turns - truth["turns"]) * 2.0 * math.pi),
        }
    return unavailable(f"verite de type inconnu : {kind}")


# ── Report ────────────────────────────────────────────────────────────────

SECTIONS = (
    ("lien", section_link),
    ("cycles", section_cycles),
    ("commandes", section_commands),
    ("suivi", section_tracking),
    ("echelons", section_steps),
    ("roues", section_wheels),
    ("verite", section_truth),
)


def build_report(run: Run) -> dict:
    """All sections of the report, as plain data."""
    report = {
        "run": run.folder.name,
        "mode": run.meta.get("mode"),
        "test_type": run.meta.get("test_type"),
        "streams_asked": run.meta.get("streams"),
        "alignment": run.alignment.to_dict(),
        "warnings": list(run.warnings),
    }
    for name, section in SECTIONS:
        report[name] = section(run)
    return report


def _number(value, unit: str = "", digits: int = 1) -> str:
    """A number for the text report, or "-" when there is none."""
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "-"
    return f"{value:.{digits}f}{unit}"


def format_report(report: dict) -> list[str]:
    """The report as lines of text, in French, for the terminal."""
    out = [f"RAPPORT {report['run']} (mode {report['mode']}"
           + (f", essai {report['test_type']}" if report.get("test_type")
              else "") + ")"]
    alignment = report["alignment"]
    if alignment["method"] == "rasp_only":
        out.append("Temps : Rasp seule (aucune trame datee par la Teensy)")
    for segment in alignment["segments"]:
        out.append(f"Temps : segment {segment['index']}, decalage "
                   f"{segment['offset_s']:+.6f} s, derive "
                   f"{segment['drift_ppm']:+.1f} ppm, latence p50 "
                   f"{segment['latency_ms']['p50']:.2f} ms")
    for warning in report["warnings"]:
        out.append(f"ATTENTION : {warning}")

    for name, _ in SECTIONS:
        section = report[name]
        out.append("")
        out.append(f"[{name}]")
        if not section.get("available"):
            out.append(f"  non disponible : {section['reason']}")
            continue
        out.extend(f"  {line}" for line in _FORMATTERS[name](section))
    return out


def _lines_link(s: dict) -> list[str]:
    return [
        s["verdict"] or "verification du log non faite",
        f"trames refusees {s['rejected']}, NACK {s['nack']}, non decodees "
        f"{s['undecoded']}, lignes perdues par le logger {s['dropped']}",
        "trous de seq : " + (", ".join(f"{k} {v}" for k, v in
                                       s["seq_gaps"].items()) or "-")
        + f" ; ticks incomplets {s['incomplete_ticks']}",
    ]


def _lines_cycles(s: dict) -> list[str]:
    work = s["work_us"]
    lines = [
        f"{s['ticks']} ticks, travail p50 {_number(work.get('p50'), ' us', 0)}"
        f", p99 {_number(s['work_p99_us'], ' us', 0)}, max "
        f"{_number(work.get('max'), ' us', 0)} pour {s['period_us']:.0f} us "
        f"({s['budget_used_p99'] * 100:.0f} % au p99)",
        f"depassements {s['overruns']}, consignes non envoyees "
        f"{s['writes_failed']}, trames de log perdues {s['log_dropped']}",
        f"lecture OTOS p50 {_number(s['otos_read_us'].get('p50'), ' us', 0)}"
        f", max {_number(s['otos_read_us'].get('max'), ' us', 0)}",
    ]
    for wheel, causes in s["rs485_fail"].items():
        detail = ", ".join(f"{k} {v}" for k, v in causes.items()) or "aucun"
        lines.append(f"echecs RS485 roue {wheel} : {detail}")
    return lines


def _lines_commands(s: dict) -> list[str]:
    latency = s["latency_ms"]
    lines = [f"{s['paired']} SET_TARGET apparies, latence p50 "
             f"{_number(latency.get('p50'), ' ms', 2)}, p95 "
             f"{_number(latency.get('p95'), ' ms', 2)}, max "
             f"{_number(latency.get('max'), ' ms', 2)}"]
    if s["pairing_doubtful"]:
        lines.append(f"appariement douteux : {s['negative']} latences "
                     f"negatives (commande perdue ?)")
    return lines


def _lines_tracking(s: dict) -> list[str]:
    error = s["error_mm"]
    lines = [
        f"erreur de suivi : RMS {_number(error['rms'], ' mm')} "
        f"(en mouvement {_number(error['rms_moving'], ' mm')}), max "
        f"{_number(error['max'], ' mm')} ; cap RMS "
        f"{_number(s['error_rad']['rms'], ' rad', 3)}",
        f"commande : acceleration p95 "
        f"{_number(s['command']['accel_mm_s2'].get('p95'), ' mm/s2', 0)}, "
        f"jerk p95 {_number(s['command']['jerk_mm_s3'].get('p95'), ' mm/s3', 0)}",
    ]
    if s["motion"]:
        lines.append(
            f"mouvement : acceleration p95 "
            f"{_number(s['motion']['accel_mm_s2'].get('p95'), ' mm/s2', 0)}, "
            f"jerk p95 "
            f"{_number(s['motion']['jerk_mm_s3'].get('p95'), ' mm/s3', 0)}")
    lines.append("saturations : " + ", ".join(
        f"{name} {value['ticks']} ticks / {value['episodes']} fois"
        for name, value in s["saturations"].items()))
    lines.append("pertes de pas : " + ", ".join(
        f"roue {wheel} {count}" for wheel, count in s["lost_steps"].items()))
    return lines


def _lines_steps(s: dict) -> list[str]:
    lines = [f"source {s['source']}, {len(s['steps'])} echelon(s)"]
    for step in s["steps"]:
        unit, digits = (" mm", 1) if step["kind"] == "translation" else (" rad", 3)
        lines.append(
            f"t={step['t']:.2f} s {step['kind']} "
            f"{_number(step['size'], unit, digits)} : montee "
            f"{_number(step['rise_s'], ' s', 3)}, depassement "
            f"{step['overshoot_pct']:.1f} %, erreur statique "
            f"{_number(step['static_error'], unit, digits)}, etabli en "
            f"{_number(step['settling_s'], ' s', 2)}")
    return lines


def _lines_wheels(s: dict) -> list[str]:
    lines = [f"tick {s['tick_s'] * 1e3:.1f} ms ; retard compte en plus du "
             f"tick entre commande et mesure"]
    for wheel, fit in s["wheels"].items():
        if not fit.get("available", True):
            lines.append(f"roue {wheel} : non disponible ({fit['reason']})")
            continue
        lines.append(
            f"roue {wheel} : retard {fit['delay_ticks']} tick(s) "
            f"({fit['delay_s'] * 1e3:.0f} ms), tau_v "
            f"{fit['tau_s'] * 1e3:.0f} ms, gain {_number(fit['gain'], '', 3)}"
            f", r2 {fit['r2']:.3f}")
    return lines


def _lines_truth(s: dict) -> list[str]:
    if s["kind"] == "pose":
        return [f"pose finale ({s['source']}) a {s['error_mm']:.1f} mm de la "
                f"verite (dx {s['error_xy_mm'][0]:+.1f}, dy "
                f"{s['error_xy_mm'][1]:+.1f}), cap {s['error_rad']:+.4f} rad"]
    return [f"tours estimes ({s['source']}) {s['estimated_turns']:+.3f} pour "
            f"{s['truth_turns']:+.3f}, ecart {s['error_rad']:+.4f} rad"]


_FORMATTERS = {
    "lien": _lines_link,
    "cycles": _lines_cycles,
    "commandes": _lines_commands,
    "suivi": _lines_tracking,
    "echelons": _lines_steps,
    "roues": _lines_wheels,
    "verite": _lines_truth,
}


def _plain(value):
    """NaN and infinities to None, numpy scalars to Python, for JSON."""
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(value) else None
    if isinstance(value, (np.bool_,)):
        return bool(value)
    if value is pd.NA:
        return None
    return value


def write_report(report: dict, folder) -> Path:
    """Write report.json in the run folder."""
    target = Path(folder) / REPORT_FILE
    target.write_text(json.dumps(_plain(report), indent=2, ensure_ascii=False)
                      + "\n", encoding="utf-8")
    return target


def main() -> int:
    """Report on a run folder from the command line."""
    parser = argparse.ArgumentParser(
        description="Rapport d'un dossier de run : lien, cycles, suivi, "
                    "echelons, roues, verite terrain.",
    )
    parser.add_argument("folder", help="dossier du run")
    parser.add_argument("--no-write", action="store_true",
                        help="ne pas ecrire report.json")
    args = parser.parse_args()

    run = load_run(args.folder)
    report = build_report(run)
    for line in format_report(report):
        print(line)
    if not args.no_write:
        print()
        print(f"Ecrit : {write_report(report, args.folder)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
