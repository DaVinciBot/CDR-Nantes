#!/usr/bin/env python3
"""Run one trial on the robot: record it, then ask for its ground truth.

    cd robot1/rasp
    python -m tools.run_test --list
    python -m tools.run_test --type check --duration 10 --yes
    python -m tools.run_test --type otos_lin --note "1 m le long de x, 200 mm/s" \
        --start-pose "250 1000 0" --target "1250 1000 0"
    python -m tools.run_test --type otos_ang --note "10 tours a gauche, lent"

A trial is a run made to calibrate or to validate, so this tool is strict
where a match is not:

  * everything is recorded: the log is always on here, whatever config.json
    says, and both per-tick messages of the Teensy are asked for;
  * a calibration trial does not start without LOG_TICK (override with
    --allow-partial, the folder will say so);
  * the trial is not closed without its ground truth, or an explicit reason
    for having none;
  * at the end the folder is read back, and the verdict says whether it can be
    used: whole log, right kind of truth, LOG_TICK actually received.

The robot is moved either by hand (no --target: record, then stop with Enter)
or through plain pose targets sent one after the other. The link is the one of
ROBOT_MODE, so the tool also runs in dummy mode, to rehearse a session.

Trial types, ground truth and verdict: robot1/rasp/telemetry/trial.py.
"""

import argparse
import logging
import math
import sys
import threading
import time

from usb_com import Messages, TraceMask, pack_message, payload_size, unpack_message

from robot1.rasp.comm import get_mode, init_robot
from robot1.rasp.telemetry import trial as trials
from robot1.rasp.telemetry.logger import RunLogger, load_log_config

logger = logging.getLogger("RUN_TEST")

ARRIVED_MM = 20.0
ARRIVED_RAD = 0.05
POLL_S = 0.05


class PoseTracker:
    """Last pose the Teensy reported, for the arrival test and the summary."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.pose = None
        self.frames = 0

    def on_odometry(self, data: bytes) -> None:
        """Callback of the link for ODOMETRY."""
        if len(data) != payload_size(Messages.ODOMETRY):
            return
        x, y, theta = unpack_message(Messages.ODOMETRY, data)[:3]
        with self._lock:
            self.pose = (x, y, theta)
            self.frames += 1

    def latest(self):
        """Return (pose or None, frames received)."""
        with self._lock:
            return self.pose, self.frames


def build_parser() -> argparse.ArgumentParser:
    """Build the command line of the tool."""
    parser = argparse.ArgumentParser(
        description="Lance un essai enregistre, puis demande sa verite terrain.",
    )
    parser.add_argument("--list", action="store_true",
                        help="afficher les types d'essai et sortir")
    parser.add_argument("--type", choices=sorted(trials.TRIAL_TYPES),
                        help="type d'essai (voir --list)")
    parser.add_argument("--note", default="",
                        help="ce qui a ete fait : distance, vitesse, sens...")
    parser.add_argument("--operator", default="", help="qui fait l'essai")
    parser.add_argument("--start-pose", metavar="'X Y DEG'",
                        help="pose imposee a la Teensy avant l'essai (table)")
    parser.add_argument("--target", metavar="'X Y DEG'", action="append",
                        default=[], help="cible de pose, repetable ; sans "
                                         "cible, le robot est bouge a la main")
    parser.add_argument("--target-timeout", type=float, default=20.0,
                        help="attente maximale par cible, en secondes")
    parser.add_argument("--duration", type=float,
                        help="duree d'enregistrement apres les cibles, en "
                             "secondes ; sans elle, Entree arrete l'essai")
    parser.add_argument("--yes", action="store_true",
                        help="demarrer sans attendre Entree")
    parser.add_argument("--truth", metavar="'X Y DEG'",
                        help="pose finale mesuree sur la table")
    parser.add_argument("--turns", type=float,
                        help="tours faits sur place, sens trigonometrique positif")
    parser.add_argument("--no-truth", metavar="RAISON",
                        help="clore sans verite terrain, avec sa raison")
    parser.add_argument("--uncertainty-mm", type=float, default=2.0)
    parser.add_argument("--uncertainty-deg", type=float, default=0.5)
    parser.add_argument("--no-log-tick", action="store_true",
                        help="ne pas demander LOG_TICK a la Teensy")
    parser.add_argument("--no-pid-trace", action="store_true",
                        help="ne pas demander PID_TRACE a la Teensy")
    parser.add_argument("--allow-partial", action="store_true",
                        help="lancer un essai de calibration sans LOG_TICK")
    parser.add_argument("--log-dir", help="dossier des runs (defaut : log_config)")
    return parser


def print_types(out=print) -> None:
    """Print the catalogue of trial types."""
    truth_names = {trials.TRUTH_NONE: "aucune", trials.TRUTH_POSE: "pose finale",
                   trials.TRUTH_TURNS: "nombre de tours"}
    out(f"{'type':<14} {'seance':<16} {'verite terrain':<16} description")
    for trial in trials.TRIAL_TYPES.values():
        out(f"{trial.name:<14} {trial.session:<16} "
            f"{truth_names[trial.truth]:<16} {trial.description}")


def wait_for_target(tracker: PoseTracker, target, timeout_s: float,
                    sleep=time.sleep) -> bool:
    """Wait until the reported pose is on the target, or the time is up.

    Args:
        tracker: where the ODOMETRY frames land.
        target: [x_mm, y_mm, theta_rad].
        timeout_s: longest wait.
        sleep: waiting function, replaceable in a test.

    Returns:
        bool: True when the target was reached.
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pose, _ = tracker.latest()
        if pose is not None:
            distance = math.hypot(pose[0] - target[0], pose[1] - target[1])
            turn = math.remainder(pose[2] - target[2], 2.0 * math.pi)
            if distance < ARRIVED_MM and abs(turn) < ARRIVED_RAD:
                return True
        sleep(POLL_S)
    return False


def ask_truth(trial: trials.TrialType, args, input_fn=input, out=print) -> dict:
    """Get the ground truth of the trial: command line first, prompt otherwise.

    A type that needs no truth gets its "none" with the reason of the type.
    Otherwise the operator is asked until the answer can be read; typing
    "aucune" closes the trial without truth, and its reason is then mandatory.

    Args:
        trial: the type that was run.
        args: parsed command line.
        input_fn: prompt function, replaceable in a test.
        out: print function.

    Returns:
        dict: content of truth.json.
    """
    uncertainty_rad = math.radians(args.uncertainty_deg)
    if args.no_truth is not None:
        return trials.truth_none(args.no_truth)
    if args.truth is not None:
        return trials.truth_pose(trials.parse_pose(args.truth),
                                 uncertainty_mm=args.uncertainty_mm,
                                 uncertainty_rad=uncertainty_rad)
    if args.turns is not None:
        return trials.truth_turns(args.turns, uncertainty_rad=uncertainty_rad)
    if trial.truth == trials.TRUTH_NONE:
        return trials.truth_none(trial.truth_reason)

    if trial.truth == trials.TRUTH_TURNS:
        question = ("Tours faits sur place (sens trigonometrique positif), "
                    "ou 'aucune' : ")
    else:
        question = ("Pose finale mesuree sur la table : x y theta "
                    "(mm mm degres), ou 'aucune' : ")
    while True:
        answer = input_fn(question).strip()
        try:
            if answer.lower() in ("aucune", "none"):
                reason = input_fn("Raison (obligatoire) : ")
                return trials.truth_none(reason)
            if trial.truth == trials.TRUTH_TURNS:
                return trials.truth_turns(float(answer.replace(",", ".")),
                                          uncertainty_rad=uncertainty_rad)
            return trials.truth_pose(trials.parse_pose(answer),
                                     uncertainty_mm=args.uncertainty_mm,
                                     uncertainty_rad=uncertainty_rad)
        except ValueError as error:
            out(f"  Non compris ({error}). Recommencer.")


def run_trial(args, *, com=None, input_fn=input, sleep=time.sleep,
              out=print) -> int:
    """Run one trial from start to verdict.

    Args:
        args: parsed command line (see build_parser()).
        com: an open link; opened from ROBOT_MODE when omitted.
        input_fn: prompt function, replaceable in a test.
        sleep: waiting function, replaceable in a test.
        out: print function.

    Returns:
        int: 0 when the trial is usable, 1 when its folder has a problem,
        2 when it was not started.
    """
    trial = trials.TRIAL_TYPES[args.type]
    mask = TraceMask.NONE
    if not args.no_pid_trace:
        mask |= TraceMask.PID_TRACE
    if not args.no_log_tick:
        mask |= TraceMask.LOG_TICK

    problems = trials.start_problems(trial,
                                     log_tick=bool(mask & TraceMask.LOG_TICK))
    if problems and not args.allow_partial:
        for problem in problems:
            out(f"REFUS : {problem}.")
        out("Relancer sans --no-log-tick, ou avec --allow-partial.")
        return 2

    # Read before anything is opened: a typo in a pose must not cost a run.
    try:
        start_pose = (trials.parse_pose(args.start_pose)
                      if args.start_pose else None)
        targets = [trials.parse_pose(text) for text in args.target]
        if args.truth is not None:
            trials.parse_pose(args.truth)
        if args.turns is not None:
            trials.truth_turns(args.turns)
        if args.no_truth is not None:
            trials.truth_none(args.no_truth)
    except ValueError as error:
        out(f"REFUS : argument illisible ({error}).")
        return 2

    own_link = com is None
    if own_link:
        com, _ = init_robot(logger)
    log = RunLogger(
        args.log_dir or load_log_config()["dir"], trial.name, mode=get_mode(),
        clock_ns=time.monotonic_ns, clock_name="monotonic", trace_mask=mask,
        meta={
            "test_type": trial.name, "session": trial.session,
            "note": args.note, "operator": args.operator,
            "truth_kind": trial.truth, "partial": bool(problems),
        },
    )
    out(f"Essai '{trial.name}' : {trial.description}")
    out(f"Dossier : {log.folder}")

    tracker = PoseTracker()
    com.add_callback(tracker.on_odometry, Messages.ODOMETRY.value)
    log.attach(com)
    if hasattr(com, "start_heartbeat"):
        com.start_heartbeat()

    try:
        if start_pose is not None:
            com.send_bytes(pack_message(Messages.SET_ODOMETRIE, *start_pose))
            log.set_meta(start_pose_table=start_pose)
            sleep(0.3)
        if not args.yes:
            input_fn("Robot en place. Entree pour demarrer l'essai... ")
        log.emit("trial", event="start")

        for index, target in enumerate(targets, start=1):
            com.send_bytes(pack_message(Messages.SET_TARGET, *target,
                                        0.0, 0.0, 0.0))
            reached = wait_for_target(tracker, target, args.target_timeout,
                                      sleep=sleep)
            log.emit("trial", event="target", index=index, target=target,
                     reached=reached)
            out(f"Cible {index}/{len(targets)} : "
                f"{'atteinte' if reached else 'NON atteinte'}.")

        if args.duration is not None:
            sleep(args.duration)
        else:
            input_fn("Enregistrement en cours. Entree pour arreter... ")
    except (KeyboardInterrupt, EOFError):
        out("Essai interrompu.")
        log.emit("trial", event="abort")
    finally:
        # Whatever happened, the base is stopped before anything is asked.
        com.send_bytes(pack_message(Messages.STOP))
        log.emit("trial", event="end")

    pose, frames = tracker.latest()
    if pose is None:
        out("Aucune pose recue de la Teensy.")
    else:
        out(f"Pose estimee a l'arret : x = {pose[0]:.1f} mm, y = {pose[1]:.1f} "
            f"mm, theta = {math.degrees(pose[2]):.2f} deg ({frames} trames).")

    # The run is closed whatever the operator answers: a truth that was not
    # given leaves a readable folder, flagged as unusable by the verdict.
    try:
        truth = ask_truth(trial, args, input_fn=input_fn, out=out)
        trials.write_truth(log.folder, truth)
    except (KeyboardInterrupt, EOFError):
        truth = None
        out("Verite terrain non saisie.")
    finally:
        if pose is not None:
            log.set_meta(final_pose_estimated=list(pose))
        log.close()
        if own_link and hasattr(com, "close"):
            com.close()

    if truth is not None and truth["kind"] == trials.TRUTH_POSE and pose is not None:
        true_x, true_y, true_theta = truth["final_pose_table"]
        turn = math.remainder(pose[2] - true_theta, 2.0 * math.pi)
        out(f"Ecart estimation - verite : dx = {pose[0] - true_x:+.1f} mm, "
            f"dy = {pose[1] - true_y:+.1f} mm, "
            f"dtheta = {math.degrees(turn):+.2f} deg.")

    check = trials.check_trial(log.folder)
    for stream, count in sorted(check.streams.items()):
        out(f"  {stream:<12} {count}")
    if check.ok:
        out(f"ESSAI : OK — {check.messages} messages, {check.lines} lignes, "
            f"verite terrain '{truth['kind']}' ({check.folder.name}).")
        return 0
    for problem in check.problems:
        out(f"PROBLEME : {problem}.")
    out(f"ESSAI : INUTILISABLE tel quel ({check.folder.name}).")
    return 1


def main() -> int:
    """Parse the command line and run the trial."""
    logging.basicConfig(level=logging.WARNING,
                        format="%(levelname)s | %(name)s | %(message)s")
    parser = build_parser()
    args = parser.parse_args()
    if args.list:
        print_types()
        return 0
    if args.type is None:
        parser.error("--type est obligatoire (voir --list)")
    return run_trial(args)


if __name__ == "__main__":
    sys.exit(main())
