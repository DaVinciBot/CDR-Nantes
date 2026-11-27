"""Replay of the complementary filter: the board's code, fed with its own inputs.

    python -m robot1.replay.estimators.complementary logs/2027-01-06_1432_seq_01
    python -m robot1.replay.estimators.complementary logs/... --set alpha_encoders=0.3

LOG_TICK carries, every tick, the exact input handed to the estimator (est_in)
and what it answered (est). The filter of lib/pose_estimator is run again on
the PC (native.py) on the logged inputs, and its answer is compared with the
logged one, tick by tick. Without --set this is a non-regression test: the
replay must give the logged estimate back, within the tolerances below, with
exactly the same flags. With --set, a constant is changed and the replay
gives what the filter would have estimated (tuning of alpha_encoders against
the ground truth); there is then no verdict.

Where the replay starts. LOG_TICK is off at boot, so a replay never starts
from the boot. The state of the filter is known exactly only right after a
reset, logged as an event:

  POSE_SET              reset() at the pose imposed by SET_ODOMETRIE;
  ESTIMATOR_SELECTED    the complementary filter takes over: reset() at the
                        last estimate (the one of the tick before).

The board dates the event after the reset, outside its critical section: a
tick may come in between. An event of tick N happened before tick N + 1, or
before tick N. Both are tried, the one the log agrees with is kept.

Elsewhere (start of the log, after a hole in the ticks) the replay starts
warm, at the logged estimate, with the encoder counts the filter was still
holding unknown: those ticks are compared and shown as "approximate", out of
the verdict, until the next reset. The continuity of the ticks is judged on
"tick", which counts every tick of the board, logged or not.

Ticks run by the EKF are skipped (the EKF is replayed by ekf.py).

Writes est_replay.json in the run folder; report.py shows it.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import fields
from pathlib import Path

import pandas as pd
from usb_com.python.messages import Estimator, LogEvent, LogInputFlag

from ..load import Run, load_run
from .native import (
    ComplementaryParams,
    EstimatorProcess,
    MissingBinary,
    StepInput,
    StepOutput,
)

RESULT_FILE = "est_replay.json"

# Agreement between the board and the replay. The same C++ on both sides; the
# differences left come from the maths libraries (sin, cos) and from the
# compilers fusing or not a multiplication and an addition: a few units in
# the last place of a float. First values, to revisit with the first runs.
TOL_POSITION_MM = 1e-3
TOL_THETA_RAD = 1e-6
TOL_VELOCITY_MM_S = 1e-2
TOL_OMEGA_RAD_S = 1e-4
# Flags (health, step) must be equal.

# Verdicts
OK = "OK"
MISMATCH = "ECART"
INCONCLUSIVE = "NON CONCLUANT"
NO_VERDICT = "SANS VERDICT"


# ── Inputs from the log ───────────────────────────────────────────────────

def params_from_log(row: dict) -> ComplementaryParams:
    """The constants the board ran with, from a LOG_PARAMS line.

    counts_to_mm is not in the message: it is computed as config.h does,
    in double, from the wheel diameter and the counts per revolution.
    """
    diameters = row["wheel_diameter_mm"]
    diameter = diameters[0] if isinstance(diameters, (list, tuple)) else diameters
    return ComplementaryParams(
        robot_radius_mm=float(row["robot_radius_mm"]),
        counts_to_mm=float(diameter) * math.pi / float(row["counts_per_rev"]),
        encoder_noise_counts=float(row["encoder_noise_counts"]),
        imu_max_age_us=int(row["imu_max_age_us"]),
        optical_outlier_mm_s=float(row["optical_outlier_mm_s"]),
        optical_noise_mm_s=float(row["optical_noise_mm_s"]),
        pure_rotation_min_rad=float(row["pure_rotation_min_rad"]),
        pure_rotation_min_counts=float(row["pure_rotation_min_counts"]),
        pure_rotation_tolerance=float(row["pure_rotation_tolerance"]),
        alpha_encoders=float(row["alpha_encoders"]),
        use_encoders=bool(row["use_encoders"]),
    )


def apply_overrides(params: ComplementaryParams,
                    overrides: dict | None) -> ComplementaryParams:
    """A copy of params with some constants changed ("name": value)."""
    if not overrides:
        return params
    values = params.to_dict()
    known = {f.name: f.type for f in fields(ComplementaryParams)}
    for name, value in overrides.items():
        if name not in known:
            raise KeyError(f"parametre inconnu : {name} (connus : "
                           f"{', '.join(known)})")
        current = values[name]
        if isinstance(current, bool):
            values[name] = str(value).lower() in ("1", "true", "oui", "yes")
        elif isinstance(current, int):
            values[name] = int(float(value))
        else:
            values[name] = float(value)
    return ComplementaryParams(**values)


def step_input(row) -> StepInput:
    """The EstimatorInput of a tick, from its est_in columns."""
    flags = LogInputFlag(int(row["input_flags"]))
    return StepInput(
        d1=int(row["d_1"]), d2=int(row["d_2"]), d3=int(row["d_3"]),
        dt_us=int(row["dt_us"]),
        encoder_age_us=int(row["encoder_age_us"]),
        first_frame=LogInputFlag.FIRST_FRAME in flags,
        encoders_fresh=LogInputFlag.ENCODERS_FRESH in flags,
        imu_valid=LogInputFlag.IMU_VALID in flags,
        heading_rad=float(row["heading_rad"]),
        imu_timestamp_us=int(row["imu_timestamp_us"]),
        imu_age_us=int(row["imu_age_us"]),
        optical_timestamp_us=int(row["optical_timestamp_us"]),
        optical_step_valid=LogInputFlag.OPTICAL_STEP_VALID in flags,
        optical_dx_mm=float(row["optical_dx_mm"]),
        optical_dy_mm=float(row["optical_dy_mm"]),
        optical_step_dt_us=int(row["optical_step_dt_us"]),
        optical_velocity_valid=LogInputFlag.OPTICAL_VELOCITY_VALID in flags,
        optical_vx_mm_s=float(row["optical_vx_mm_s"]),
        optical_vy_mm_s=float(row["optical_vy_mm_s"]),
    )


def logged_pose(row) -> tuple[float, float, float]:
    """The estimate the board logged at a tick."""
    return (float(row["est_pose_x"]), float(row["est_pose_y"]),
            float(row["est_pose_theta"]))


# ── Comparison ────────────────────────────────────────────────────────────

def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def differences(out: StepOutput, row) -> dict:
    """Replay minus board, per field (angles wrapped)."""
    return {
        "x": out.x - float(row["est_pose_x"]),
        "y": out.y - float(row["est_pose_y"]),
        "theta": _wrap(out.theta - float(row["est_pose_theta"])),
        "vx": out.vx - float(row["est_vel_vx"]),
        "vy": out.vy - float(row["est_vel_vy"]),
        "omega": out.omega - float(row["est_vel_omega"]),
        "health": out.health ^ int(row["est_health"]),
        "flags": out.flags ^ int(row["est_flags"]),
    }


_TOLERANCES = {
    "x": TOL_POSITION_MM, "y": TOL_POSITION_MM, "theta": TOL_THETA_RAD,
    "vx": TOL_VELOCITY_MM_S, "vy": TOL_VELOCITY_MM_S,
    "omega": TOL_OMEGA_RAD_S,
}


def out_of_tolerance(diff: dict) -> list[str]:
    """Fields of a tick where the replay leaves the board."""
    bad = [name for name, tol in _TOLERANCES.items() if abs(diff[name]) > tol]
    bad.extend(name for name in ("health", "flags") if diff[name])
    return bad


def tolerances() -> dict:
    return {"position_mm": TOL_POSITION_MM, "theta_rad": TOL_THETA_RAD,
            "velocity_mm_s": TOL_VELOCITY_MM_S, "omega_rad_s": TOL_OMEGA_RAD_S,
            "flags": "egalite"}


# ── One segment of the board (between two boots) ──────────────────────────

class _SegmentReplay:
    """Walks the ticks of one segment, resets included, and keeps the score."""

    def __init__(self, est: EstimatorProcess, params: ComplementaryParams,
                 estimator: int, events: list[dict]):
        self.est = est
        self.active = estimator
        self.pending = sorted(events, key=lambda e: (e["tick"], e.get("seq", 0)))
        self.alive = False      # the replay holds a state for the filter
        self.anchored = False   # ... and that state is exact
        self.prev = None        # last tick row walked
        self.rows = []          # replayed ticks
        self.anchors = []
        self.gaps = 0
        self.skipped_ekf = 0
        self.warm_starts = 0
        est.params(params)

    # Events

    def _apply(self, event: dict, before_tick: int, placement: str) -> None:
        """Do what the board did for this event, just before before_tick."""
        kind = event["event"]
        if kind == LogEvent.POSE_SET.value:
            if self.active == Estimator.COMPLEMENTARY.value:
                self.est.reset(event["value_x"], event["value_y"],
                               event["value_theta"])
                self.alive = self.anchored = True
                self.anchors.append({"tick": before_tick, "event": "POSE_SET",
                                     "placement": placement})
        elif kind == LogEvent.ESTIMATOR_SELECTED.value:
            self.active = int(event["arg"])
            if self.active != Estimator.COMPLEMENTARY.value:
                self.alive = self.anchored = False
                return
            # The board resets from the last estimate, the one of the tick
            # before: exact only if that tick is in the log.
            if self.prev is not None and int(self.prev["tick"]) == before_tick - 1:
                self.est.reset(*logged_pose(self.prev))
                self.alive = self.anchored = True
                self.anchors.append({"tick": before_tick,
                                     "event": "ESTIMATOR_SELECTED",
                                     "placement": placement})
            else:
                self.alive = self.anchored = False

    def _take(self, predicate) -> list[dict]:
        taken = [e for e in self.pending if predicate(int(e["tick"]))]
        self.pending = [e for e in self.pending if e not in taken]
        return taken

    # Ticks

    def tick(self, row) -> None:
        tick = int(row["tick"])
        if self.prev is not None and tick != int(self.prev["tick"]) + 1:
            self.gaps += 1
            self.alive = self.anchored = False

        # Events dated before this tick happened before it.
        for event in self._take(lambda n: n < tick):
            self._apply(event, tick, "apres le tick date")

        # Events dated this tick: before it (race) or after it.
        ambiguous = [e for e in self.pending if int(e["tick"]) == tick]
        out = None
        if ambiguous:
            out = self._try_before(row, ambiguous)

        if out is None:
            if self.active != Estimator.COMPLEMENTARY.value:
                self.skipped_ekf += 1
                self.prev = row
                return
            if not self.alive:
                # Warm start: this tick is the state, the next one is compared.
                self.est.reset(*logged_pose(row))
                self.alive = True
                self.anchored = False
                self.warm_starts += 1
                self.prev = row
                return
            out = self.est.step(tick, step_input(row))
        self._record(row, out)
        self.prev = row

    def _try_before(self, row, events: list[dict]) -> StepOutput | None:
        """Place events dated this tick: before it if the log says so.

        Returns the step output when the events went before the tick (they
        are then consumed), None when they stay for after it.
        """
        tick = int(row["tick"])
        state = (self.active, self.alive, self.anchored, len(self.anchors))
        complementary = self.active == Estimator.COMPLEMENTARY.value
        after_out = None
        if self.alive and complementary:
            self.est.keep()
            after_out = self.est.step(tick, step_input(row))
            self.est.back()
        for event in events:
            self._apply(event, tick, "avant le tick date")
        before_ok = False
        before_out = None
        if self.alive and self.active == Estimator.COMPLEMENTARY.value:
            before_out = self.est.step(tick, step_input(row))
            before_ok = not out_of_tolerance(differences(before_out, row))
        after_ok = (after_out is not None
                    and not out_of_tolerance(differences(after_out, row)))
        if before_ok and not after_ok:
            self.pending = [e for e in self.pending if e not in events]
            return before_out
        # The usual order: events after the tick. Undo the trial; without a
        # state to come back to (not alive), the tick starts warm anyway.
        self.active, self.alive, self.anchored = state[:3]
        del self.anchors[state[3]:]
        if after_out is not None:
            self.est.back()
        return None

    def _record(self, row, out: StepOutput) -> None:
        diff = differences(out, row)
        self.rows.append({
            "tick": int(row["tick"]), "t": float(row["t"]),
            "anchored": self.anchored,
            "x": out.x, "y": out.y, "theta": out.theta,
            "vx": out.vx, "vy": out.vy, "omega": out.omega,
            "health": out.health, "flags": out.flags,
            **{f"d_{name}": value for name, value in diff.items()},
            "bad": ",".join(out_of_tolerance(diff)),
        })


def _summary(segment: int, walker: _SegmentReplay, compare: bool) -> dict:
    table = pd.DataFrame(walker.rows)
    summary = {
        "segment": segment,
        "compared": len(table),
        "anchors": walker.anchors,
        "gaps": walker.gaps,
        "warm_starts": walker.warm_starts,
        "skipped_ekf": walker.skipped_ekf,
    }
    if table.empty:
        summary.update(anchored=0, approximate=0, mismatches=0,
                       first_mismatch=None, max_abs={})
        return summary
    anchored = table[table["anchored"]]
    summary["anchored"] = len(anchored)
    summary["approximate"] = len(table) - len(anchored)
    summary["max_abs"] = {
        "anchored": _max_abs(anchored), "approximate":
        _max_abs(table[~table["anchored"]]),
    }
    last = table.iloc[-1]
    summary["final"] = {"tick": int(last["tick"]),
                        "replay": [last["x"], last["y"], last["theta"]],
                        "board": [last["x"] - last["d_x"],
                                  last["y"] - last["d_y"],
                                  _wrap(last["theta"] - last["d_theta"])]}
    if compare:
        bad = anchored[anchored["bad"] != ""]
        summary["mismatches"] = len(bad)
        if bad.empty:
            summary["first_mismatch"] = None
        else:
            first = bad.iloc[0]
            summary["first_mismatch"] = {
                "tick": int(first["tick"]), "t": float(first["t"]),
                "fields": first["bad"].split(","),
                "diff": {name: float(first[f"d_{name}"])
                         for name in ("x", "y", "theta", "vx", "vy", "omega")},
                "replay_flags": int(first["flags"]),
                "replay_health": int(first["health"]),
                "board_flags": int(first["flags"]) ^ int(first["d_flags"]),
                "board_health": int(first["health"]) ^ int(first["d_health"]),
            }
    return summary


def _max_abs(table: pd.DataFrame) -> dict:
    if table.empty:
        return {}
    return {name: float(table[f"d_{name}"].abs().max())
            for name in ("x", "y", "theta", "vx", "vy", "omega")}


# ── The whole run ─────────────────────────────────────────────────────────

def _segment_params(run: Run, segment) -> dict | None:
    """The LOG_PARAMS lines of a segment (first and last), or the meta copy."""
    params = run.stream("params")
    if not params.empty and "segment" in params:
        mine = params[params["segment"] == segment]
        if not mine.empty:
            return {"first": mine.iloc[0].to_dict(), "last": mine.iloc[-1].to_dict()}
    copy = run.meta.get("teensy_params")
    if copy:
        return {"first": copy, "last": copy}
    return None


def unavailable(reason: str) -> dict:
    return {"available": False, "reason": reason}


def replay_run(run: Run, overrides: dict | None = None, binary=None) -> dict:
    """Replay the complementary filter over a loaded run.

    Args:
        run: the run, from load_run().
        overrides: constants to change ("alpha_encoders": 0.3); none for the
            non-regression test.
        binary: path of est_replay, default found by native.find_binary().

    Returns:
        dict: the result, as written in est_replay.json. "available" is
        False, with the reason, when the run cannot be replayed.

    Raises:
        MissingBinary: est_replay is not built.
    """
    ticks = run.tick_table()
    if ticks.empty or "dt_us" not in ticks or "est_pose_x" not in ticks:
        return unavailable("flux est_in / est absents (LOG_TICK)")
    compare = not overrides
    events = run.stream("event")

    result = {
        "available": True,
        "estimator": "complementary",
        "overrides": dict(overrides or {}),
        "tolerances": tolerances(),
        "segments": [],
        "warnings": [],
    }
    with EstimatorProcess(binary) as est:
        for segment, group in ticks.groupby("segment", sort=True):
            found = _segment_params(run, segment)
            if found is None:
                result["warnings"].append(
                    f"segment {segment} : pas de LOG_PARAMS, non rejoue")
                continue
            diameters = found["last"].get("wheel_diameter_mm")
            if isinstance(diameters, (list, tuple)) and len(set(diameters)) > 1:
                result["warnings"].append(
                    f"segment {segment} : diametres de roue differents "
                    f"{diameters}, le filtre n'en prend qu'un (roue 1)")
            params = apply_overrides(params_from_log(found["last"]), overrides)
            result["params"] = params.to_dict()
            mine = []
            if not events.empty and "segment" in events:
                mine = events[events["segment"] == segment].to_dict("records")
            walker = _SegmentReplay(est, params, int(found["first"]["estimator"]),
                                    mine)
            for _, row in group.sort_values("tick", kind="stable").iterrows():
                walker.tick(row)
            result["segments"].append(_summary(int(segment), walker, compare))

    result.update(_verdict(result, compare))
    if overrides:
        result["truth"] = _against_truth(run, result)
    return result


def _verdict(result: dict, compare: bool) -> dict:
    segments = result["segments"]
    anchored = sum(s["anchored"] for s in segments)
    if not compare:
        return {"verdict": NO_VERDICT, "anchored": anchored}
    mismatches = sum(s["mismatches"] for s in segments)
    if mismatches:
        verdict = MISMATCH
    elif anchored == 0:
        verdict = INCONCLUSIVE
    else:
        verdict = OK
    return {"verdict": verdict, "anchored": anchored, "mismatches": mismatches}


def _against_truth(run: Run, result: dict) -> dict | None:
    """End pose of the replay and of the board against a pose ground truth."""
    truth = run.truth
    if not truth or truth.get("kind") != "pose" or not result["segments"]:
        return None
    final = result["segments"][-1].get("final")
    if not final:
        return None
    tx, ty, tt = truth["final_pose_table"]

    def error(pose):
        return {"error_mm": math.hypot(pose[0] - tx, pose[1] - ty),
                "error_rad": _wrap(pose[2] - tt)}

    return {"replay": error(final["replay"]), "board": error(final["board"])}


# ── Output ────────────────────────────────────────────────────────────────

def format_result(result: dict) -> list[str]:
    """The result as lines of text, in French."""
    if not result.get("available"):
        return [f"rejeu non disponible : {result['reason']}"]
    out = []
    if result["overrides"]:
        out.append("parametres modifies : " + ", ".join(
            f"{k}={v}" for k, v in result["overrides"].items()))
    for segment in result["segments"]:
        anchors = ", ".join(f"{a['event']} tick {a['tick']}"
                            for a in segment["anchors"]) or "aucune"
        out.append(
            f"segment {segment['segment']} : {segment['compared']} ticks "
            f"rejoues, {segment['anchored']} depuis une ancre exacte, "
            f"{segment['approximate']} approches ; ancres : {anchors}")
        extra = []
        if segment["gaps"]:
            extra.append(f"{segment['gaps']} trou(s) de tick")
        if segment["warm_starts"]:
            extra.append(f"{segment['warm_starts']} depart(s) a chaud")
        if segment["skipped_ekf"]:
            extra.append(f"{segment['skipped_ekf']} ticks sous EKF ignores")
        if extra:
            out.append("  " + ", ".join(extra))
        worst = segment.get("max_abs", {}).get("anchored")
        if worst:
            out.append(
                f"  ecart max (ancre) : x {worst['x']:.2e} mm, y "
                f"{worst['y']:.2e} mm, cap {worst['theta']:.2e} rad, vitesse "
                f"{max(worst['vx'], worst['vy']):.2e} mm/s")
        first = segment.get("first_mismatch")
        if first:
            out.append(
                f"  premier ecart : tick {first['tick']} (t={first['t']:.3f} s),"
                f" champs {', '.join(first['fields'])} ; drapeaux rejeu "
                f"{first['replay_flags']} / carte {first['board_flags']}, "
                f"sante {first['replay_health']} / {first['board_health']}")
        final = segment.get("final")
        if final and result["overrides"]:
            out.append(f"  pose finale rejeu ({final['replay'][0]:.1f}, "
                       f"{final['replay'][1]:.1f}, {final['replay'][2]:+.4f}) "
                       f"; carte ({final['board'][0]:.1f}, "
                       f"{final['board'][1]:.1f}, {final['board'][2]:+.4f})")
    truth = result.get("truth")
    if truth:
        out.append(f"verite terrain : rejeu a {truth['replay']['error_mm']:.1f} "
                   f"mm, carte a {truth['board']['error_mm']:.1f} mm")
    for warning in result["warnings"]:
        out.append(f"ATTENTION : {warning}")
    verdict = result["verdict"]
    if verdict == NO_VERDICT:
        out.append("REJEU : sans verdict (parametres modifies)")
    else:
        out.append(f"REJEU : {verdict}")
        if verdict == INCONCLUSIVE:
            out.append("  aucune ancre exacte (POSE_SET ou ESTIMATOR_SELECTED "
                       "pendant LOG_TICK) : rien a juger")
    return out


def write_result(result: dict, folder) -> Path:
    """Write est_replay.json in the run folder."""
    from ..report import _plain

    target = Path(folder) / RESULT_FILE
    target.write_text(json.dumps(_plain(result), indent=2, ensure_ascii=False)
                      + "\n", encoding="utf-8")
    return target


def _parse_set(items: list[str]) -> dict:
    overrides = {}
    for item in items:
        name, sep, value = item.partition("=")
        if not sep:
            raise SystemExit(f"--set attend nom=valeur, recu : {item}")
        overrides[name.strip()] = value.strip()
    return overrides


def main() -> int:
    """Replay a run folder from the command line.

    Exit code: 0 replay OK (or no verdict asked), 1 mismatch, 2 nothing to
    judge or the tool is missing.
    """
    parser = argparse.ArgumentParser(
        description="Rejoue le filtre complementaire de la carte sur les "
                    "entrees de LOG_TICK et compare a l'estimation loggee.",
    )
    parser.add_argument("folder", help="dossier du run")
    parser.add_argument("--set", action="append", default=[], metavar="NOM=VAL",
                        help="change une constante du filtre (pas de verdict)")
    parser.add_argument("--bin", help="chemin de est_replay")
    parser.add_argument("--no-write", action="store_true",
                        help=f"ne pas ecrire {RESULT_FILE}")
    args = parser.parse_args()

    overrides = _parse_set(args.set)
    run = load_run(args.folder, verify=False)
    try:
        result = replay_run(run, overrides, args.bin)
    except MissingBinary as error:
        print(error)
        return 2
    for line in format_result(result):
        print(line)
    if not args.no_write:
        print()
        print(f"Ecrit : {write_result(result, args.folder)}")
    if not result.get("available"):
        return 2
    return {OK: 0, NO_VERDICT: 0, MISMATCH: 1}.get(result["verdict"], 2)


if __name__ == "__main__":
    sys.exit(main())
