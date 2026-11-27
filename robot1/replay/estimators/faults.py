"""Sensor faults injected into logged estimator inputs, before a replay.

A fault rewrites the est_in columns of the tick table (load.Run.tick_table()),
never the run folder. It tells whether a diagnostic of the EKF catches what
it is meant to catch, on real logs as on synthetic ones:

  optical_scale:F           the optical sensor under-counts (F < 1, 0.7 to
                            0.9 is the typical dusty or badly mounted
                            sensor): steps and velocities times F. The gate
                            does NOT see it, the optical / wheel speed ratio
                            does.
  optical_dropout:T0,D      the sensor loses track from T0 for D seconds
                            (from the first tick): it keeps answering, with
                            zero motion, while the wheels turn. Seen by the
                            gate, then by the consecutive rejections.
  gyro_drift:R              the IMU heading drifts by R rad/s from the first
                            tick. Seen as a bias of the innovations (heading
                            and wheels pull apart).

Several faults may be given; they apply in the order given.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pandas as pd

KINDS = {
    "optical_scale": 1,
    "optical_dropout": 2,
    "gyro_drift": 1,
}

_OPTICAL = ("optical_dx_mm", "optical_dy_mm", "optical_vx_mm_s",
            "optical_vy_mm_s")


@dataclass(frozen=True)
class Fault:
    """One fault: its kind and its numbers."""

    kind: str
    values: tuple

    def __str__(self) -> str:
        return f"{self.kind}:{','.join(f'{v:g}' for v in self.values)}"


def parse_fault(text: str) -> Fault:
    """A fault from its command-line form, "kind:v1[,v2]".

    Raises:
        ValueError: unknown kind or wrong numbers, with the reason in French.
    """
    kind, _, rest = text.partition(":")
    kind = kind.strip()
    if kind not in KINDS:
        raise ValueError(f"defaut inconnu : {kind} (connus : "
                         f"{', '.join(KINDS)})")
    try:
        values = tuple(float(v) for v in rest.split(",") if v.strip())
    except ValueError as error:
        raise ValueError(f"defaut {kind} : nombres illisibles : {rest}") from error
    if len(values) != KINDS[kind]:
        raise ValueError(f"defaut {kind} : {KINDS[kind]} valeur(s) attendue(s), "
                         f"{len(values)} donnee(s)")
    if kind == "optical_scale" and not values[0] > 0.0:
        raise ValueError("defaut optical_scale : facteur > 0 attendu")
    if kind == "optical_dropout" and not values[1] > 0.0:
        raise ValueError("defaut optical_dropout : duree > 0 attendue")
    return Fault(kind, values)


def _float32(values) -> np.ndarray:
    return np.asarray(values, dtype=np.float64).astype(np.float32).astype(
        np.float64)


def _elapsed_s(ticks: pd.DataFrame) -> np.ndarray:
    """Seconds since the first tick, from the tick counter and dt_us."""
    period_s = float(ticks["dt_us"].median()) * 1e-6 if len(ticks) else 0.0
    tick = ticks["tick"].to_numpy(dtype=np.float64)
    return (tick - tick.min()) * period_s


def apply_faults(ticks: pd.DataFrame, faults) -> pd.DataFrame:
    """A copy of the tick table with the faults applied to est_in.

    Values stay float32, as a LOG_TICK carries them.
    """
    out = ticks.copy()
    if out.empty:
        return out
    elapsed = _elapsed_s(out)
    for fault in faults:
        if fault.kind == "optical_scale":
            for column in _OPTICAL:
                out[column] = _float32(out[column] * fault.values[0])
        elif fault.kind == "optical_dropout":
            t0, duration = fault.values
            lost = (elapsed >= t0) & (elapsed < t0 + duration)
            for column in _OPTICAL:
                out.loc[lost, column] = 0.0
        elif fault.kind == "gyro_drift":
            heading = out["heading_rad"].to_numpy(dtype=np.float64)
            heading = heading + fault.values[0] * elapsed
            out["heading_rad"] = _float32(np.arctan2(np.sin(heading),
                                                     np.cos(heading)))
    return out


def describe(faults) -> list[str]:
    return [str(fault) for fault in faults]
