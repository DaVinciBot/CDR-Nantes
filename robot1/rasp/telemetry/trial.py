"""A trial: a recorded run made to measure something, with its ground truth.

A match is recorded to be looked at afterwards. A trial is recorded to
calibrate or to validate: it has a type (what was done), a note (how), and a
ground truth (what really happened, measured on the table). Without the truth
a calibration trial is worth nothing, and the session it came from has to be
done again. So the type decides two things up front:

  * whether the trial needs the per-tick log of the Teensy (LOG_TICK): the raw
    sensors and the input of the estimator are what a calibration is made of;
  * which ground truth must be given before the trial can be closed: a final
    pose read on the table marks, a number of turns, or none.

truth.json, next to meta.json in the run folder:

    {"kind": "pose", "method": "marques_table",
     "final_pose_table": [1250.0, 1000.0, 0.0],
     "uncertainty_mm": 2.0, "uncertainty_rad": 0.01, "note": ""}

    {"kind": "turns", "method": "marque_au_sol", "turns": 10.0,
     "uncertainty_rad": 0.02, "note": ""}

    {"kind": "none", "reason": "robot immobile"}

Poses are in the table frame (mm, rad), like everything else in the log.
Pure module: no thread, no link, no prompt.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass
from pathlib import Path

from . import log_format, log_reader

TRUTH_FILE = "truth.json"

TRUTH_NONE = "none"
TRUTH_POSE = "pose"
TRUTH_TURNS = "turns"


@dataclass(frozen=True)
class TrialType:
    """One kind of trial.

    Attributes:
        name: identifier, used on the command line and in the folder name.
        session: the group it belongs to in a robot session.
        description: what the operator does, in a line.
        calibration: needs LOG_TICK; refuses to start without it.
        truth: ground truth to give before closing (TRUTH_*).
        truth_reason: why none is needed, when truth is TRUTH_NONE.
    """

    name: str
    session: str
    description: str
    calibration: bool = True
    truth: str = TRUTH_POSE
    truth_reason: str = ""


_TYPES = (
    TrialType("check", "avant seance",
              "essai factice de 10 s : le dossier doit sortir complet",
              calibration=False, truth=TRUTH_NONE,
              truth_reason="controle du log, rien n'est mesure"),
    TrialType("static_off", "statique",
              "60 s immobile, moteurs coupes : bruit et biais des capteurs",
              truth=TRUTH_NONE, truth_reason="robot immobile"),
    TrialType("static_on", "statique",
              "60 s immobile, moteurs alimentes a vitesse nulle",
              truth=TRUTH_NONE, truth_reason="robot immobile"),
    TrialType("otos_axes", "capteur optique",
              "deplacement a la main le long de x puis de y robot : signes "
              "et axes",
              truth=TRUTH_NONE,
              truth_reason="controle de signe, decrit dans la note"),
    TrialType("otos_ang", "capteur optique",
              "N tours sur place : facteur angulaire",
              truth=TRUTH_TURNS),
    TrialType("otos_lin", "capteur optique",
              "translation de longueur connue : facteur lineaire, angle de "
              "montage"),
    TrialType("kin_wheels_up", "cinematique",
              "roues levees, consignes vx, vy, w : signes, ordre des roues",
              truth=TRUTH_NONE, truth_reason="roues levees, le robot ne bouge pas"),
    TrialType("kin_ground", "cinematique",
              "au sol, consignes lentes : diametres de roue"),
    TrialType("kin_spin", "cinematique",
              "N tours sur place : rayon du robot",
              truth=TRUTH_TURNS),
    TrialType("sequence", "deplacement",
              "sequence de deplacement : suivi, fluidite, arret"),
    TrialType("drift_100s", "deplacement",
              "100 s de trajets enchaines : derive de la pose"),
    TrialType("free", "autre",
              "essai libre, decrit dans la note",
              calibration=False, truth=TRUTH_NONE,
              truth_reason="essai libre"),
)

TRIAL_TYPES: dict[str, TrialType] = {trial.name: trial for trial in _TYPES}


# ── Ground truth ──────────────────────────────────────────────────────────

def parse_pose(text: str) -> list[float]:
    """Read "x, y, theta_deg" as typed by an operator.

    Comma or space separated, angle in DEGREES: that is what is read off a
    table. The result is in mm and radians, as stored.

    Args:
        text: three numbers.

    Returns:
        list[float]: [x_mm, y_mm, theta_rad].

    Raises:
        ValueError: if the text does not hold exactly three finite numbers.
    """
    parts = text.replace(",", " ").replace(";", " ").split()
    if len(parts) != 3:
        raise ValueError("attendu : x y theta (mm, mm, degres)")
    x, y, theta_deg = (float(part) for part in parts)
    if not all(math.isfinite(value) for value in (x, y, theta_deg)):
        raise ValueError("valeurs non finies")
    return [x, y, math.radians(theta_deg)]


def truth_pose(final_pose, *, method: str = "marques_table",
               uncertainty_mm: float = 2.0, uncertainty_rad: float = 0.01,
               note: str = "") -> dict:
    """Build the truth of a trial measured by its final pose.

    Args:
        final_pose: [x_mm, y_mm, theta_rad], table frame.
        method: how it was measured.
        uncertainty_mm: how well the position is known.
        uncertainty_rad: how well the heading is known.
        note: anything worth knowing later.

    Returns:
        dict: content of truth.json.

    Raises:
        ValueError: on a pose that is not three finite numbers, or on a
            negative uncertainty.
    """
    pose = [float(value) for value in final_pose]
    if len(pose) != 3 or not all(math.isfinite(value) for value in pose):
        raise ValueError("final_pose : trois nombres finis attendus")
    if uncertainty_mm < 0 or uncertainty_rad < 0:
        raise ValueError("incertitude negative")
    return {
        "kind": TRUTH_POSE, "method": method, "final_pose_table": pose,
        "uncertainty_mm": float(uncertainty_mm),
        "uncertainty_rad": float(uncertainty_rad), "note": note,
    }


def truth_turns(turns: float, *, method: str = "marque_au_sol",
                uncertainty_rad: float = 0.02, note: str = "") -> dict:
    """Build the truth of a trial measured by a number of turns in place.

    Args:
        turns: turns made, counter-clockwise positive (the sign of theta).
        method: how they were counted.
        uncertainty_rad: how well the final alignment is known.
        note: anything worth knowing later.

    Returns:
        dict: content of truth.json.

    Raises:
        ValueError: on zero or non-finite turns: a trial that did not turn
            measures no angular scale.
    """
    turns = float(turns)
    if not math.isfinite(turns) or turns == 0.0:
        raise ValueError("nombre de tours nul ou non fini")
    if uncertainty_rad < 0:
        raise ValueError("incertitude negative")
    return {
        "kind": TRUTH_TURNS, "method": method, "turns": turns,
        "uncertainty_rad": float(uncertainty_rad), "note": note,
    }


def truth_none(reason: str) -> dict:
    """Build the explicit "no ground truth" of a trial.

    Args:
        reason: why there is none. Never empty: a trial without truth must
            say it was a choice, not an oversight.

    Returns:
        dict: content of truth.json.

    Raises:
        ValueError: on an empty reason.
    """
    reason = reason.strip()
    if not reason:
        raise ValueError("raison obligatoire pour un essai sans verite terrain")
    return {"kind": TRUTH_NONE, "reason": reason}


def write_truth(folder, truth: dict) -> Path:
    """Write truth.json in a run folder, through a temporary file.

    Args:
        folder: the run folder.
        truth: what truth_pose(), truth_turns() or truth_none() built.

    Returns:
        Path: the file written.
    """
    target = Path(folder) / TRUTH_FILE
    temporary = target.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(truth, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, target)
    return target


def read_truth(folder) -> dict | None:
    """Return truth.json of a run, or None when it has none."""
    path = Path(folder) / TRUTH_FILE
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


# ── Before and after ──────────────────────────────────────────────────────

def start_problems(trial: TrialType, *, log_tick: bool) -> list[str]:
    """Tell what forbids starting a trial.

    Args:
        trial: the type about to be run.
        log_tick: whether LOG_TICK is asked from the Teensy.

    Returns:
        list[str]: reasons, empty when the trial may start.
    """
    problems = []
    if trial.calibration and not log_tick:
        problems.append(
            f"l'essai '{trial.name}' sert a calibrer : il lui faut LOG_TICK "
            f"(capteurs bruts, entree de l'estimateur)"
        )
    return problems


def check_trial(folder) -> log_reader.RunCheck:
    """Verify a trial folder: a whole log, and what its type demands.

    On top of log_reader.check_run(): the ground truth of the right kind, and
    for a calibration type, LOG_TICK lines actually received. A calibration
    trial that asked for LOG_TICK and got none recorded nothing it can be
    calibrated with, however clean the rest looks.

    Args:
        folder: the run folder.

    Returns:
        RunCheck: counters and problems, as check_run() gives them.
    """
    check = log_reader.check_run(folder)
    name = check.meta.get("test_type")
    trial = TRIAL_TYPES.get(name)
    if trial is None:
        check.problems.append(f"type d'essai inconnu ({name!r})")
        return check

    truth = read_truth(folder)
    if truth is None:
        check.problems.append("pas de verite terrain (truth.json absent)")
    elif trial.truth != TRUTH_NONE and truth.get("kind") != trial.truth:
        if truth.get("kind") == TRUTH_NONE:
            check.problems.append(
                f"essai '{trial.name}' sans verite terrain "
                f"({truth.get('reason', '')}) : inutilisable pour calibrer"
            )
        else:
            check.problems.append(
                f"verite terrain de type '{truth.get('kind')}', "
                f"'{trial.truth}' attendu"
            )

    if trial.calibration:
        received = min(
            check.streams.get(stream, 0) for stream in log_format.LOG_TICK_STREAMS
        )
        if received == 0:
            check.problems.append(
                "aucun LOG_TICK recu : rien a calibrer dans cet essai"
            )
    return check
