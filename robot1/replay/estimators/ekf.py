"""The 6-state EKF run offline on a logged run, and whether it is consistent.

    python -m robot1.replay.estimators.ekf logs/2027-01-06_1432_seq_01
    python -m robot1.replay.estimators.ekf logs/... --params ekf.json
    python -m robot1.replay.estimators.ekf logs/... --set accel_noise=150
    python -m robot1.replay.estimators.ekf logs/... --fault optical_scale:0.8
    python -m robot1.replay.estimators.ekf logs/seq_* --compare

On board the complementary filter estimates the pose; the EKF of lib/kalman
is the same code run here, on the PC (est_replay), on the inputs the board
logged (est_in of LOG_TICK). Its constants are not in the log: they are what
this replay is for. Default: provisional_ekf6_params(), changed by a JSON file
(--params, as ekf_tune.py writes it) and by --set.

A filter is judged on its consistency, not on its error alone:

  NIS         per source (wheels, optical, heading), mean over the accepted
              updates. For a consistent filter it follows a chi-square with
              one degree of freedom, truncated by the 99 % gate: expected
              mean 0.925. Above 1.3: the filter is overconfident (Q or R too
              small); below 0.7: pessimistic. The band of the mean for that
              many updates is also given, as information: real innovations
              are correlated, it is far too narrow to judge by.
  bias        mean of the normalised innovations, as a z score: a calibration
              error (scale, mounting angle) or a drift pulls it off zero.
  whiteness   autocorrelation of the normalised innovations, lags 1 to 20,
              per scalar channel: a consistent filter leaves white
              innovations; a wrong model leaves structure.
  2 sigma     the end pose against the ground truth (truth.json, its own
              uncertainty added): inside the 2-sigma ellipse in (x, y), and
              within 2 sigma in heading. Over many trials, 86.5 % of the
              ellipses (chi-square 2 <= 4) and 95.4 % of the headings should
              hold the truth.

Against the board: the end error of the EKF next to the one of the estimate
the board logged (the complementary filter, during a calibration trial).

Where the replay starts: at a POSE_SET (the pose imposed by SET_ODOMETRIE),
with the initial uncertainty of EkfInitialSigmas. Without one, or after a
hole in the ticks, it starts warm at the estimate the board logged: such a
start is counted and said. The EKF runs over every tick, whatever estimator
the board was running.

Writes ekf_report.json in the run folder (not with --fault: a fault is a
what-if, the folder keeps the replay of what was logged); report.py shows it.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field, fields
from pathlib import Path

import numpy as np
import pandas as pd
from usb_com.python.messages import LogEvent

from ..load import Run, load_run
from . import faults as fault_tools
from .complementary import logged_pose, step_input
from .native import (
    UPDATE_SOURCES,
    EkfParams,
    MissingBinary,
    ReplayToolError,
    params_line,
    reset_line,
    run_batch,
    step_line,
)

RESULT_FILE = "ekf_report.json"

# 99th percentile of a chi-square with one degree of freedom: the gate of
# every scalar update (CHI2_1DOF_99 of lib/kalman).
GATE = 6.635

# Mean NIS judged consistent on the table (lib/kalman/include/ekf.h).
NIS_BAND = (0.7, 1.3)

# Whiteness: lags looked at, and how many of them may leave the 95 % bound
# of a white sequence before it is called coloured (one in twenty is chance).
WHITENESS_LAGS = 20
WHITENESS_MAX_OUTSIDE = 3
# Fewer accepted updates than this: no statistic on the channel.
MIN_UPDATES = 50

# Mean normalised innovation, as a z score, beyond which a channel is biased.
BIAS_Z = 4.0

# Coverage expected of a consistent filter.
COVERAGE_XY = 1.0 - math.exp(-2.0)          # chi-square 2 <= 4: 0.865
COVERAGE_THETA = math.erf(2.0 / math.sqrt(2.0))  # |z| <= 2: 0.954

GROUPS = {
    "roues": ("wheel_1", "wheel_2", "wheel_3"),
    "optique": ("optical_x", "optical_y"),
    "cap": ("heading",),
}


# ── Expected NIS ──────────────────────────────────────────────────────────

def truncated_nis_moments(gate: float = GATE) -> tuple[float, float]:
    """Mean and variance of a chi-square(1) truncated at gate.

    The NIS of a scalar update is z^2, z normal; only |z| <= sqrt(gate) is
    accepted, and the statistic is over accepted updates.
    """
    c = math.sqrt(gate)
    phi = math.exp(-0.5 * c * c) / math.sqrt(2.0 * math.pi)
    inside = math.erf(c / math.sqrt(2.0))
    m2 = 1.0 - 2.0 * c * phi / inside
    m4 = 3.0 - 2.0 * phi * (c ** 3 + 3.0 * c) / inside
    return m2, m4 - m2 * m2


NIS_EXPECTED, NIS_VARIANCE = truncated_nis_moments()


# ── Parameters ────────────────────────────────────────────────────────────

def _known_fields() -> dict:
    return {f.name: f for f in fields(EkfParams)}


def coerce_values(values: dict | None) -> dict:
    """Check names and types of EKF constants given by hand or in a file.

    Raises:
        KeyError: an unknown name, with the known ones (in French).
    """
    known = _known_fields()
    out = {}
    for name, value in (values or {}).items():
        if name not in known:
            raise KeyError(f"parametre EKF inconnu : {name} (connus : "
                           f"{', '.join(known)})")
        out[name] = (int(float(value)) if known[name].type in (int, "int")
                     else float(value))
    return out


def params_from_log(row: dict, values: dict | None = None) -> EkfParams:
    """Geometry and period from a LOG_PARAMS line, then the given constants."""
    diameters = row["wheel_diameter_mm"]
    diameter = diameters[0] if isinstance(diameters, (list, tuple)) else diameters
    period_us = float(row.get("period_us") or 15000)
    return EkfParams.for_period(
        float(row["robot_radius_mm"]),
        float(diameter) * math.pi / float(row["counts_per_rev"]),
        period_us * 1e-6,
        **coerce_values(values),
    )


def read_params_file(path) -> dict:
    """The constants of a JSON file: {"accel_noise": 150, ...}."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return coerce_values(data.get("params", data))


# ── Replay ────────────────────────────────────────────────────────────────

@dataclass
class _Segment:
    """What one segment of the board sends to est_replay, P line aside."""

    segment: int
    log_params: dict
    lines: list = field(default_factory=list)   # R and S lines
    rows: list = field(default_factory=list)    # one per S line
    anchors: list = field(default_factory=list)
    warm_starts: int = 0
    gaps: int = 0


@dataclass
class Prepared:
    """A run turned into est_replay lines once, replayed with any constants."""

    folder: Path
    segments: list
    truth: dict | None
    faults: list
    warnings: list


@dataclass
class EkfReplay:
    """One replay: a row per tick, a row per scalar update."""

    params: EkfParams
    ticks: pd.DataFrame
    updates: pd.DataFrame
    anchors: list
    warm_starts: int
    gaps: int


def _segment_log_params(run: Run, segment) -> dict | None:
    params = run.stream("params")
    if not params.empty and "segment" in params:
        mine = params[params["segment"] == segment]
        if not mine.empty:
            return mine.iloc[-1].to_dict()
    return run.meta.get("teensy_params")


def prepare(run: Run, faults=()) -> Prepared:
    """Build the lines of every segment: resets, then one step per tick."""
    ticks = run.tick_table()
    if ticks.empty or "dt_us" not in ticks or "est_pose_x" not in ticks:
        raise ValueError("flux est_in / est absents (LOG_TICK)")
    ticks = fault_tools.apply_faults(ticks, list(faults))
    events = run.stream("event")
    warnings = []
    segments = []
    for segment, group in ticks.groupby("segment", sort=True):
        log_params = _segment_log_params(run, segment)
        if log_params is None:
            warnings.append(f"segment {segment} : pas de LOG_PARAMS, non rejoue")
            continue
        pending = []
        if not events.empty and "segment" in events:
            mine = events[(events["segment"] == segment)
                          & (events["event"] == LogEvent.POSE_SET.value)]
            pending = sorted(mine.to_dict("records"),
                             key=lambda e: (e["tick"], e.get("seq", 0)))
        seg = _Segment(int(segment), log_params)
        alive = False
        prev_tick = None
        for _, row in group.sort_values("tick", kind="stable").iterrows():
            tick = int(row["tick"])
            if prev_tick is not None and tick != prev_tick + 1:
                seg.gaps += 1
                alive = False
            # A POSE_SET dated tick N happened before tick N + 1.
            while pending and int(pending[0]["tick"]) < tick:
                event = pending.pop(0)
                seg.lines.append(reset_line(event["value_x"], event["value_y"],
                                            event["value_theta"]))
                seg.anchors.append(tick)
                alive = True
            prev_tick = tick
            if not alive:
                seg.lines.append(reset_line(*logged_pose(row)))
                seg.warm_starts += 1
                alive = True
                continue
            seg.lines.append(step_line(tick, step_input(row)))
            seg.rows.append((tick, float(row["t"]), logged_pose(row)))
        segments.append(seg)
    return Prepared(run.folder, segments, run.truth, list(faults), warnings)


def execute(prepared: Prepared, values: dict | None = None,
            binary=None) -> EkfReplay:
    """Run the EKF over a prepared run with these constants.

    Raises:
        MissingBinary: est_replay is not built.
        ReplayToolError: the tool refused a line.
    """
    tick_rows, update_rows = [], []
    params = None
    anchors, warm, gaps = [], 0, 0
    for seg in prepared.segments:
        params = params_from_log(seg.log_params, values)
        outputs = run_batch([params_line(params), *seg.lines], "ekf6", binary)
        if len(outputs) != len(seg.rows):
            raise ReplayToolError(f"est_replay : {len(outputs)} reponses pour "
                                  f"{len(seg.rows)} pas")
        anchors += [{"segment": seg.segment, "tick": t} for t in seg.anchors]
        warm += seg.warm_starts
        gaps += seg.gaps
        for out, (tick, t, board) in zip(outputs, seg.rows):
            tick_rows.append((
                seg.segment, tick, t, out.x, out.y, out.theta, out.vx, out.vy,
                out.omega, out.pxx, out.pyy, out.ptt, out.pxy, out.pxt, out.pyt,
                out.covariance_healthy, out.optical_ok,
                out.optical_consecutive_rejections, out.optical_ratio,
                out.optical_ratio_windows, out.optical_ratio_windows_out,
                out.health, *board))
            for source, innovation, variance, nis, accepted in out.updates:
                update_rows.append((seg.segment, tick, t, source, innovation,
                                    variance, nis, accepted))
    ticks = pd.DataFrame(tick_rows, columns=[
        "segment", "tick", "t", "x", "y", "theta", "vx", "vy", "omega",
        "pxx", "pyy", "ptt", "pxy", "pxt", "pyt", "covariance_healthy",
        "optical_ok", "optical_consecutive_rejections", "optical_ratio",
        "optical_ratio_windows", "optical_ratio_windows_out", "health",
        "board_x", "board_y", "board_theta"])
    updates = pd.DataFrame(update_rows, columns=[
        "segment", "tick", "t", "source", "innovation", "variance", "nis",
        "accepted"])
    return EkfReplay(params, ticks, updates, anchors, warm, gaps)


def replay_run(run: Run, values: dict | None = None, faults=(),
               binary=None) -> EkfReplay:
    """prepare() then execute(), for one run."""
    return execute(prepare(run, faults), values, binary)


# ── Indicators ────────────────────────────────────────────────────────────

def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def nis_group(updates: pd.DataFrame, sources) -> dict:
    """Mean NIS of a group of sources, over accepted updates, and its verdict."""
    mine = updates[updates["source"].isin(sources)]
    accepted = mine[mine["accepted"]]
    n, rejected = len(accepted), len(mine) - len(accepted)
    out = {"accepted": n, "rejected": rejected,
           "rejection_rate": rejected / len(mine) if len(mine) else None,
           "expected": NIS_EXPECTED, "band": list(NIS_BAND)}
    if n == 0:
        out.update(mean_nis=None, verdict="aucune mise a jour")
        return out
    mean = float(accepted["nis"].mean())
    half = 1.96 * math.sqrt(NIS_VARIANCE / n)
    out["mean_nis"] = mean
    out["statistical_band"] = [NIS_EXPECTED - half, NIS_EXPECTED + half]
    if n < MIN_UPDATES:
        out["verdict"] = "trop peu de mises a jour"
    elif mean > NIS_BAND[1]:
        out["verdict"] = "trop confiant (Q ou R trop petits)"
    elif mean < NIS_BAND[0]:
        out["verdict"] = "trop pessimiste (Q ou R trop grands)"
    else:
        out["verdict"] = "coherent"
    return out


def autocorrelation(values: np.ndarray, lags: int) -> np.ndarray:
    """Normalised autocorrelation at lags 1..lags."""
    centred = values - values.mean()
    denominator = float(np.dot(centred, centred))
    if denominator == 0.0:
        return np.zeros(lags)
    return np.array([float(np.dot(centred[:-k], centred[k:])) / denominator
                     for k in range(1, lags + 1)])


def channel(updates: pd.DataFrame, source: str) -> dict:
    """Bias and whiteness of one scalar channel (accepted updates, in order)."""
    mine = updates[(updates["source"] == source) & updates["accepted"]]
    n = len(mine)
    out = {"accepted": n}
    if n < MIN_UPDATES:
        out["verdict"] = "trop peu de mises a jour"
        return out
    normalised = (mine["innovation"] / np.sqrt(mine["variance"])).to_numpy()
    z = float(normalised.mean() * math.sqrt(n))
    lags = min(WHITENESS_LAGS, n // 4)
    rho = autocorrelation(normalised, lags)
    bound = 1.96 / math.sqrt(n)
    outside = int(np.sum(np.abs(rho) > bound))
    out.update(
        mean=float(normalised.mean()), bias_z=z, biased=abs(z) > BIAS_Z,
        lags=lags, bound=bound, rho_max=float(np.max(np.abs(rho))),
        rho_lag1=float(rho[0]), lags_outside=outside,
        white=outside <= WHITENESS_MAX_OUTSIDE,
    )
    return out


def end_against_truth(replay: EkfReplay, truth: dict | None) -> dict | None:
    """End error of the EKF and of the board, and the 2-sigma test.

    The truth's own uncertainty is added to the covariance of the EKF: a
    truth read to 2 mm cannot judge a filter to better than that.
    """
    if not truth or truth.get("kind") != "pose" or replay.ticks.empty:
        return None
    last = replay.ticks.iloc[-1]
    tx, ty, tt = truth["final_pose_table"]
    ex, ey = float(last["x"]) - tx, float(last["y"]) - ty
    et = _wrap(float(last["theta"]) - tt)
    u_mm = float(truth.get("uncertainty_mm") or 0.0)
    u_rad = float(truth.get("uncertainty_rad") or 0.0)
    cov = np.array([[last["pxx"] + u_mm ** 2, last["pxy"]],
                    [last["pxy"], last["pyy"] + u_mm ** 2]], dtype=float)
    try:
        m2 = float(np.array([ex, ey]) @ np.linalg.solve(cov, [ex, ey]))
    except np.linalg.LinAlgError:
        m2 = math.inf
    sigma_theta = math.sqrt(max(float(last["ptt"]), 0.0) + u_rad ** 2)
    board_xy = math.hypot(float(last["board_x"]) - tx, float(last["board_y"]) - ty)
    return {
        "ekf": {"error_mm": math.hypot(ex, ey), "error_xy_mm": [ex, ey],
                "error_rad": et},
        "board": {"error_mm": board_xy,
                  "error_rad": _wrap(float(last["board_theta"]) - tt)},
        "sigma_xy_mm": [math.sqrt(max(cov[0, 0], 0.0)),
                        math.sqrt(max(cov[1, 1], 0.0))],
        "sigma_theta_rad": sigma_theta,
        "mahalanobis2_xy": m2,
        "inside_2sigma_xy": m2 <= 4.0,
        "inside_2sigma_theta": abs(et) <= 2.0 * sigma_theta,
    }


def diagnostics(replay: EkfReplay) -> dict:
    ticks = replay.ticks
    if ticks.empty:
        return {}
    last = ticks.iloc[-1]
    return {
        "ticks": len(ticks),
        "covariance_unhealthy_ticks": int((~ticks["covariance_healthy"]).sum()),
        "optical_not_ok_ticks": int((~ticks["optical_ok"]).sum()),
        "optical_max_consecutive_rejections":
            int(ticks["optical_consecutive_rejections"].max()),
        "optical_ratio_windows": int(last["optical_ratio_windows"]),
        "optical_ratio_windows_out": int(last["optical_ratio_windows_out"]),
        "optical_ratio_min": _finite_min(ticks, "optical_ratio"),
    }


def _finite_min(ticks, column):
    nonzero = ticks.loc[ticks[column] > 0.0, column]
    return float(nonzero.min()) if len(nonzero) else None


def indicators(replay: EkfReplay, truth: dict | None) -> dict:
    """Every indicator of one replay."""
    return {
        "nis": {name: nis_group(replay.updates, sources)
                for name, sources in GROUPS.items()},
        "channels": {source: channel(replay.updates, source)
                     for source in UPDATE_SOURCES},
        "truth": end_against_truth(replay, truth),
        "diagnostics": diagnostics(replay),
    }


def evaluate(prepared: Prepared, values: dict | None = None,
             binary=None) -> dict:
    """Replay and judge one prepared run: the content of ekf_report.json."""
    replay = execute(prepared, values, binary)
    result = {
        "available": True,
        "run": prepared.folder.name,
        "params": replay.params.to_dict(),
        "faults": fault_tools.describe(prepared.faults),
        "anchors": replay.anchors,
        "warm_starts": replay.warm_starts,
        "gaps": replay.gaps,
        "warnings": list(prepared.warnings),
    }
    if replay.warm_starts:
        result["warnings"].append(
            f"{replay.warm_starts} depart(s) a chaud sur l'estimation de la "
            f"carte : incertitude initiale sous-estimee")
    result.update(indicators(replay, prepared.truth))
    return result


def unavailable(reason: str) -> dict:
    return {"available": False, "reason": reason}


def evaluate_run(run: Run, values: dict | None = None, faults=(),
                 binary=None) -> dict:
    """evaluate() for a loaded run, or "available": False with the reason."""
    try:
        prepared = prepare(run, faults)
    except ValueError as error:
        return unavailable(str(error))
    if not prepared.segments:
        return unavailable("; ".join(prepared.warnings) or "aucun segment")
    return evaluate(prepared, values, binary)


# ── Several runs ──────────────────────────────────────────────────────────

def summarize(results: list[dict]) -> dict:
    """EKF against the board over several runs, and the 2-sigma coverage."""
    judged = [r for r in results if r.get("available") and r.get("truth")]
    out = {"runs": len(results), "with_truth": len(judged),
           "expected_xy": COVERAGE_XY, "expected_theta": COVERAGE_THETA}
    if not judged:
        return out
    ekf = [r["truth"]["ekf"]["error_mm"] for r in judged]
    board = [r["truth"]["board"]["error_mm"] for r in judged]
    out.update(
        ekf_error_mm_mean=float(np.mean(ekf)), ekf_error_mm_max=float(np.max(ekf)),
        board_error_mm_mean=float(np.mean(board)),
        board_error_mm_max=float(np.max(board)),
        ekf_better=sum(e < b for e, b in zip(ekf, board)),
        coverage_xy=float(np.mean([r["truth"]["inside_2sigma_xy"] for r in judged])),
        coverage_theta=float(np.mean([r["truth"]["inside_2sigma_theta"]
                                      for r in judged])),
    )
    return out


# ── Output ────────────────────────────────────────────────────────────────

def _num(value, digits=2, unit=""):
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "-"
    return f"{value:.{digits}f}{unit}"


def format_result(result: dict) -> list[str]:
    """One run, as lines of text in French."""
    if not result.get("available"):
        return [f"EKF hors ligne non disponible : {result['reason']}"]
    out = [f"EKF hors ligne : {result['run']}"]
    if result["faults"]:
        out.append("  defauts injectes : " + ", ".join(result["faults"]))
    out.append(f"  ancres POSE_SET : {len(result['anchors'])}, departs a chaud : "
               f"{result['warm_starts']}, trous de tick : {result['gaps']}")
    for name, group in result["nis"].items():
        out.append(f"  NIS {name:7s} : moyenne {_num(group['mean_nis'])} "
                   f"(attendu {group['expected']:.3f}, bande {group['band'][0]}"
                   f"-{group['band'][1]}), {group['accepted']} acceptees, "
                   f"{group['rejected']} rejetees : {group['verdict']}")
    for source, ch in result["channels"].items():
        if "bias_z" not in ch:
            continue
        flags = []
        if ch["biased"]:
            flags.append("BIAISE")
        if not ch["white"]:
            flags.append("NON BLANC")
        out.append(f"  innovations {source:9s} : biais z {ch['bias_z']:+.1f}, "
                   f"rho(1) {ch['rho_lag1']:+.2f}, {ch['lags_outside']}/"
                   f"{ch['lags']} retards hors bande"
                   + (f" -> {', '.join(flags)}" if flags else ""))
    truth = result.get("truth")
    if truth:
        out.append(
            f"  verite : EKF a {truth['ekf']['error_mm']:.1f} mm "
            f"({truth['ekf']['error_rad']:+.4f} rad), carte a "
            f"{truth['board']['error_mm']:.1f} mm "
            f"({truth['board']['error_rad']:+.4f} rad)")
        out.append(
            f"  2 sigma : ellipse ({truth['sigma_xy_mm'][0]:.1f}, "
            f"{truth['sigma_xy_mm'][1]:.1f}) mm "
            f"{'contient' if truth['inside_2sigma_xy'] else 'NE contient PAS'} "
            f"la verite ; cap +/-{2 * truth['sigma_theta_rad']:.4f} rad "
            f"{'contient' if truth['inside_2sigma_theta'] else 'NE contient PAS'}"
            f" la verite")
    else:
        out.append("  verite : pas de pose de verite terrain, ni erreur ni "
                   "couverture")
    d = result.get("diagnostics") or {}
    if d:
        out.append(
            f"  diagnostics : optique non OK sur {d['optical_not_ok_ticks']} "
            f"ticks (rejets consecutifs max "
            f"{d['optical_max_consecutive_rejections']}), rapport optique / "
            f"roues hors bande sur {d['optical_ratio_windows_out']}/"
            f"{d['optical_ratio_windows']} fenetres (min "
            f"{_num(d['optical_ratio_min'], 3)}), covariance hors bornes sur "
            f"{d['covariance_unhealthy_ticks']} ticks")
    for warning in result["warnings"]:
        out.append(f"  ATTENTION : {warning}")
    return out


def format_summary(summary: dict, results: list[dict]) -> list[str]:
    """The comparison table over several runs."""
    out = ["", "EKF contre carte :",
           f"  {'essai':32s} {'EKF mm':>8s} {'carte mm':>9s} {'2s xy':>6s} "
           f"{'2s cap':>6s} {'NIS roues':>9s} {'optique':>8s} {'cap':>6s}"]
    for r in results:
        if not r.get("available"):
            out.append(f"  {'?':32s} non disponible : {r['reason']}")
            continue
        t = r.get("truth")
        nis = r["nis"]
        out.append(
            f"  {r['run'][:32]:32s} "
            f"{_num(t['ekf']['error_mm'] if t else None, 1):>8s} "
            f"{_num(t['board']['error_mm'] if t else None, 1):>9s} "
            f"{('oui' if t['inside_2sigma_xy'] else 'non') if t else '-':>6s} "
            f"{('oui' if t['inside_2sigma_theta'] else 'non') if t else '-':>6s} "
            f"{_num(nis['roues']['mean_nis']):>9s} "
            f"{_num(nis['optique']['mean_nis']):>8s} "
            f"{_num(nis['cap']['mean_nis']):>6s}")
    if summary["with_truth"]:
        out.append(
            f"  {summary['with_truth']} essai(s) avec verite : erreur moyenne "
            f"EKF {summary['ekf_error_mm_mean']:.1f} mm, carte "
            f"{summary['board_error_mm_mean']:.1f} mm ; EKF meilleur sur "
            f"{summary['ekf_better']}/{summary['with_truth']}")
        out.append(
            f"  couverture 2 sigma : xy {summary['coverage_xy']:.0%} (attendu "
            f"{COVERAGE_XY:.1%}), cap {summary['coverage_theta']:.0%} (attendu "
            f"{COVERAGE_THETA:.1%})")
        if summary["with_truth"] < 10:
            out.append("  (moins de 10 essais : la couverture n'est qu'une "
                       "indication)")
    else:
        out.append("  aucun essai avec une pose de verite terrain")
    return out


def write_result(result: dict, folder) -> Path:
    """Write ekf_report.json in the run folder."""
    from ..report import _plain

    target = Path(folder) / RESULT_FILE
    target.write_text(json.dumps(_plain(result), indent=2, ensure_ascii=False)
                      + "\n", encoding="utf-8")
    return target


def parse_set(items: list[str]) -> dict:
    values = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep:
            raise SystemExit(f"--set attend nom=valeur, recu : {item}")
        values[name.strip()] = value.strip()
    return values


def main() -> int:
    """Exit code: 0 done, 2 nothing replayable, tool missing or bad input."""
    parser = argparse.ArgumentParser(
        description="Rejoue l'EKF 6 etats sur les entrees de LOG_TICK et juge "
                    "sa coherence (NIS, biais, blancheur, couverture 2 sigma).")
    parser.add_argument("folders", nargs="+", help="dossier(s) de run")
    parser.add_argument("--params", help="constantes EKF (JSON, ecrit par "
                        "ekf_tune.py)")
    parser.add_argument("--set", action="append", default=[], metavar="NOM=VAL",
                        help="change une constante de l'EKF")
    parser.add_argument("--fault", action="append", default=[],
                        metavar="TYPE:VAL",
                        help="defaut injecte : optical_scale:F, "
                             "optical_dropout:T0,D, gyro_drift:R")
    parser.add_argument("--compare", action="store_true",
                        help="tableau EKF contre carte sur tous les runs")
    parser.add_argument("--bin", help="chemin de est_replay")
    parser.add_argument("--no-write", action="store_true",
                        help=f"ne pas ecrire {RESULT_FILE}")
    args = parser.parse_args()

    try:
        values = read_params_file(args.params) if args.params else {}
        values.update(coerce_values(parse_set(args.set)))
        faults = [fault_tools.parse_fault(f) for f in args.fault]
    except (KeyError, ValueError, OSError) as error:
        print(error)
        return 2

    results = []
    for folder in args.folders:
        run = load_run(folder, verify=False)
        try:
            result = evaluate_run(run, values, faults, args.bin)
        except MissingBinary as error:
            print(error)
            return 2
        results.append(result)
        if not result.get("available"):
            result["run"] = Path(folder).name
        for line in format_result(result):
            print(line)
        if result.get("available") and not faults and not args.no_write:
            print(f"  Ecrit : {write_result(result, folder)}")
    if args.compare or len(results) > 1:
        for line in format_summary(summarize(results), results):
            print(line)
    return 0 if any(r.get("available") for r in results) else 2


if __name__ == "__main__":
    sys.exit(main())
