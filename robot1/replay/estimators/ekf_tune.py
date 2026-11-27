"""Tune the noises of the EKF on some runs, judge the result on others.

    python -m robot1.replay.estimators.ekf_tune \\
        --tune logs/seq_01 logs/seq_02 --validate logs/seq_03 logs/seq_04 \\
        --grid grid.json --out ekf.json

grid.json gives the values to try, every combination is tried:

    {"accel_noise": [100, 200, 400], "alpha_noise": [1, 2, 4],
     "optical_sigma_mm_s": [5, 10, 20]}

Any constant of the EKF may be in the grid (heading_sigma_rad too); those
that are not keep their default, or the value of --params.

Criterion, on the tuning runs taken together: for each source (wheels,
optical, heading) the mean NIS against what a consistent filter gives
(0.925), as |ln(mean / expected)|, summed over the sources. A filter that is
consistent is what is sought, not the one with the smallest error: the
lowest error is often reached by an overconfident filter. Ties (to 1e-3)
go to the smaller mean end error against the ground truth.

The validation runs are then replayed with the best constants and judged as
ekf.py does. They must not be tuning runs: a filter judged on the runs that
tuned it says nothing. The tool refuses a run given in both sets.

--out writes the best constants, readable by ekf.py --params and by this
tool's own --params.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

from ..load import load_run
from . import ekf
from .native import MissingBinary

# Sources with fewer accepted updates are left out of the criterion.
MIN_UPDATES = 50
# Costs closer than this are a tie.
TIE = 1e-3


def read_grid(path) -> dict:
    """The grid: {name: [values]}, names checked.

    Raises:
        KeyError: unknown constant.
        ValueError: empty or malformed grid.
    """
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not data:
        raise ValueError("grille vide ou mal formee : {nom: [valeurs]} attendu")
    grid = {}
    for name, values in data.items():
        if not isinstance(values, list) or not values:
            raise ValueError(f"grille : {name} doit etre une liste non vide")
        ekf.coerce_values({name: values[0]})
        grid[name] = values
    return grid


def combinations(grid: dict, base: dict | None = None):
    """Every set of constants of the grid, on top of base."""
    names = list(grid)
    for combo in itertools.product(*(grid[name] for name in names)):
        values = dict(base or {})
        values.update(zip(names, combo))
        yield ekf.coerce_values(values)


def overlap(tune, validate) -> list[str]:
    """Runs given in both sets (same folder, whatever the spelling)."""
    tuned = {Path(f).resolve() for f in tune}
    return sorted(str(f) for f in validate if Path(f).resolve() in tuned)


def cost(replays) -> tuple[float, dict]:
    """The criterion over several replays: sum of |ln(mean NIS / expected)|."""
    updates = pd.concat([r.updates for r in replays], ignore_index=True)
    per_group = {}
    total = 0.0
    for name, sources in ekf.GROUPS.items():
        mine = updates[updates["source"].isin(sources) & updates["accepted"]]
        if len(mine) < MIN_UPDATES:
            per_group[name] = None
            continue
        mean = float(mine["nis"].mean())
        per_group[name] = mean
        total += abs(math.log(max(mean, 1e-12) / ekf.NIS_EXPECTED))
    return total, per_group


def end_error(replays, prepared) -> float | None:
    """Mean end error of the EKF against the ground truths, mm."""
    errors = []
    for replay, prep in zip(replays, prepared):
        found = ekf.end_against_truth(replay, prep.truth)
        if found:
            errors.append(found["ekf"]["error_mm"])
    return float(np.mean(errors)) if errors else None


def sweep(prepared: list, grid: dict, base: dict | None = None,
          binary=None) -> list[dict]:
    """Every combination on the tuning runs, best first."""
    rows = []
    for values in combinations(grid, base):
        replays = [ekf.execute(p, values, binary) for p in prepared]
        total, per_group = cost(replays)
        rows.append({"values": values, "cost": total, "nis": per_group,
                     "error_mm": end_error(replays, prepared)})

    def key(row):
        error = row["error_mm"]
        return (round(row["cost"] / TIE), math.inf if error is None else error)

    return sorted(rows, key=key)


def tune(tune_folders, validate_folders, grid: dict, base: dict | None = None,
         binary=None) -> dict:
    """The whole procedure: sweep on one set, judge on the other.

    Raises:
        ValueError: a run in both sets, or a run that cannot be replayed.
        MissingBinary: est_replay is not built.
    """
    both = overlap(tune_folders, validate_folders)
    if both:
        raise ValueError("run(s) a la fois de reglage et de validation : "
                         + ", ".join(both))
    prepared = []
    for folder in tune_folders:
        try:
            prepared.append(ekf.prepare(load_run(folder, verify=False)))
        except ValueError as error:
            raise ValueError(f"{folder} : {error}") from error
    ranking = sweep(prepared, grid, base, binary)
    best = ranking[0]
    validation = [ekf.evaluate_run(load_run(folder, verify=False),
                                   best["values"], binary=binary)
                  for folder in validate_folders]
    return {
        "grid": grid,
        "base": dict(base or {}),
        "tune_runs": [str(f) for f in tune_folders],
        "validate_runs": [str(f) for f in validate_folders],
        "ranking": ranking,
        "best": best,
        "validation": validation,
        "validation_summary": ekf.summarize(validation),
    }


def format_tuning(result: dict, top: int = 5) -> list[str]:
    out = [f"Reglage sur {len(result['tune_runs'])} run(s), "
           f"{len(result['ranking'])} combinaisons"]
    names = list(result["grid"])
    for rank, row in enumerate(result["ranking"][:top], start=1):
        values = ", ".join(f"{n}={row['values'][n]:g}" for n in names)
        nis = ", ".join(f"{g} {ekf._num(m)}" for g, m in row["nis"].items())
        out.append(f"  {rank}. {values} : critere {row['cost']:.3f} (NIS {nis}"
                   f"), erreur {ekf._num(row['error_mm'], 1, ' mm')}")
    if not result["validate_runs"]:
        out.append("ATTENTION : aucun run de validation, le reglage n'est pas "
                   "juge")
        return out
    out.append("")
    out.append(f"Validation sur {len(result['validate_runs'])} run(s) avec les "
               f"meilleures constantes :")
    for r in result["validation"]:
        out += ekf.format_result(r)
    out += ekf.format_summary(result["validation_summary"], result["validation"])
    return out


def main() -> int:
    """Exit code: 0 done, 2 bad input or tool missing."""
    parser = argparse.ArgumentParser(
        description="Regle les bruits de l'EKF par NIS sur des runs de "
                    "reglage, juge le resultat sur d'autres runs.")
    parser.add_argument("--tune", nargs="+", required=True,
                        help="runs de reglage")
    parser.add_argument("--validate", nargs="*", default=[],
                        help="runs de validation (jamais des runs de reglage)")
    parser.add_argument("--grid", required=True, help="grille (JSON)")
    parser.add_argument("--params", help="constantes de depart (JSON)")
    parser.add_argument("--out", help="ecrit les meilleures constantes (JSON)")
    parser.add_argument("--bin", help="chemin de est_replay")
    args = parser.parse_args()

    try:
        grid = read_grid(args.grid)
        base = ekf.read_params_file(args.params) if args.params else {}
        result = tune(args.tune, args.validate, grid, base, args.bin)
    except MissingBinary as error:
        print(error)
        return 2
    except (KeyError, ValueError, OSError) as error:
        print(error)
        return 2
    for line in format_tuning(result):
        print(line)
    if args.out:
        Path(args.out).write_text(json.dumps(
            {"params": result["best"]["values"],
             "tuned_on": result["tune_runs"]}, indent=2) + "\n",
            encoding="utf-8")
        print(f"\nEcrit : {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
