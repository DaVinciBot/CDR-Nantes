"""Tests of replay/estimators: the complementary filter replayed on the PC.

    python -m pytest tests/test_replay_estimators.py

The "board" here is est_replay itself, fed with synthetic inputs and with the
resets of the scenario: the replay must give its estimate back, place each
reset where it happened, stay out of the verdict where its state is not
known, and catch a board that ran with other constants. The comparison with
the real board (its maths library, its compiler) is made on a Teensy.

Needs est_replay: `pio run -e replay` in robot1/teensy_moteur, or its path in
EST_REPLAY_BIN. Skipped otherwise.
"""

import json
import math

import pytest
from usb_com import Messages, TraceMask
from usb_com.python.messages import Estimator, LogEstFlag, LogEvent

from robot1.replay import report
from robot1.replay.estimators import complementary, native
from robot1.replay.load import load_run

from log_helpers import (
    TICK_US,
    SyntheticRun,
    as_float32,
    log_event,
    log_params,
    log_tick_of,
    odometry,
    omni_inputs,
)

try:
    native.find_binary()
except native.MissingBinary as missing:
    pytest.skip(str(missing), allow_module_level=True)

FIRST_TICK = 1000
TICKS = 400
RADIUS_MM = 160.0
COUNTS_TO_MM = 60.0 * math.pi / 16384.0
LOG_PARAMS_ROW = {
    "robot_radius_mm": RADIUS_MM, "wheel_diameter_mm": [60.0] * 3,
    "counts_per_rev": 16384.0, "alpha_encoders": as_float32(0.2),
    "encoder_noise_counts": 16.0, "imu_max_age_us": 50_000,
    "optical_outlier_mm_s": 1500.0, "optical_noise_mm_s": 5.0,
    "pure_rotation_min_rad": as_float32(0.005),
    "pure_rotation_min_counts": 50.0,
    "pure_rotation_tolerance": as_float32(0.2), "use_encoders": 1,
}


def inputs():
    return omni_inputs(TICKS, radius_mm=RADIUS_MM, counts_to_mm=COUNTS_TO_MM)


def play_board(steps, *, before=None, ekf=(), alpha=None):
    """What the board would log: one output per tick.

    Args:
        steps: the inputs, the first one at FIRST_TICK.
        before: {tick: [("pose", x, y, theta) | ("last",)]}, resets done
            just before that tick ("last": from the last estimate, as an
            estimator switch does).
        ekf: ticks run by the EKF (its estimate is faked).
        alpha: alpha_encoders of the board, when it is not the logged one.
    """
    params = complementary.params_from_log(LOG_PARAMS_ROW)
    if alpha is not None:
        params = complementary.apply_overrides(params, {"alpha_encoders": alpha})
    before = before or {}
    outputs = {}
    last = (0.0, 0.0, 0.0)
    with native.EstimatorProcess() as est:
        est.params(params)
        est.reset(0.0, 0.0, 0.0)
        for k, step in enumerate(steps):
            tick = FIRST_TICK + k
            for action in before.get(tick, []):
                if action[0] == "pose":
                    est.reset(*action[1:])
                else:
                    est.reset(*last)
            if tick in ekf:
                out = native.StepOutput(tick, as_float32(last[0] + 1.0),
                                        last[1], last[2], 0.0, 0.0, 0.0, 1, 0)
            else:
                out = est.step(tick, step)
            outputs[tick] = out
            last = (out.x, out.y, out.theta)
    return outputs


def write_run(root, steps, outputs, events=(), drop=()):
    """A run folder with LOG_PARAMS, the LOG_TICK of each tick and events.

    Args:
        events: (tick, LogEvent, value, arg) as the board dates them.
        drop: ticks whose LOG_TICK is lost.
    """
    run = SyntheticRun(root, trace_mask=TraceMask.LOG_TICK)
    run.receive(0.0005, log_params())
    seq = 0
    for k, step in enumerate(steps):
        tick = FIRST_TICK + k
        if tick in drop:
            continue
        run.receive(tick * TICK_US * 1e-6 + 0.002,
                    log_tick_of(seq, tick, step, outputs[tick]))
        seq += 1
    for index, (tick, event, value, arg) in enumerate(events):
        run.receive(tick * TICK_US * 1e-6 + 0.004,
                    log_event(index, tick, tick * TICK_US + 4000, event.value,
                              value=list(value), arg=arg))
    return run.close()


def replay(folder, **kwargs):
    return complementary.replay_run(load_run(folder, verify=False), **kwargs)


START = (500.0, 300.0, 0.0)


# ── Transport ─────────────────────────────────────────────────────────────

def test_a_float_crosses_the_tool_unchanged():
    value = as_float32(0.1)
    with native.EstimatorProcess() as est:
        est.params(complementary.params_from_log(LOG_PARAMS_ROW))
        est.reset(value, -value, 0.0)
        out = est.step(1, native.StepInput(first_frame=True))
    assert (out.x, out.y) == (value, -value)


def test_the_tool_refuses_a_malformed_line():
    with native.EstimatorProcess() as est:
        with pytest.raises(native.ReplayToolError):
            est.step(1, native.StepInput())        # no parameters yet


# ── Non-regression ────────────────────────────────────────────────────────

def test_the_replay_gives_the_board_estimate_back(tmp_path):
    steps = inputs()
    reset_tick = FIRST_TICK + 20
    outputs = play_board(steps, before={reset_tick: [("pose", *START)]})
    folder = write_run(tmp_path, steps, outputs,
                       events=[(reset_tick - 1, LogEvent.POSE_SET, START, 0)])

    result = replay(folder)

    assert result["verdict"] == complementary.OK
    segment = result["segments"][0]
    assert segment["mismatches"] == 0
    assert segment["anchored"] == TICKS - 20
    assert segment["anchors"] == [{"tick": reset_tick, "event": "POSE_SET",
                                   "placement": "apres le tick date"}]
    worst = segment["max_abs"]["anchored"]
    assert worst["x"] == 0.0 and worst["y"] == 0.0
    # Every branch of the filter was reached.
    seen = 0
    for out in outputs.values():
        seen |= out.flags
    assert seen == (LogEstFlag.OPTICAL_USED | LogEstFlag.OPTICAL_REJECTED
                    | LogEstFlag.PURE_ROTATION)


def test_a_reset_dated_one_tick_late_is_placed_before_its_tick(tmp_path):
    steps = inputs()
    reset_tick = FIRST_TICK + 150
    outputs = play_board(steps, before={reset_tick: [("pose", *START)]})
    # A tick came between the reset and the event: dated the next tick.
    folder = write_run(tmp_path, steps, outputs,
                       events=[(reset_tick, LogEvent.POSE_SET, START, 0)])

    result = replay(folder)

    assert result["verdict"] == complementary.OK
    anchors = result["segments"][0]["anchors"]
    assert anchors == [{"tick": reset_tick, "event": "POSE_SET",
                        "placement": "avant le tick date"}]


def test_a_board_with_other_constants_is_caught(tmp_path):
    steps = inputs()
    reset_tick = FIRST_TICK + 20
    outputs = play_board(steps, before={reset_tick: [("pose", *START)]},
                         alpha=0.3)
    folder = write_run(tmp_path, steps, outputs,
                       events=[(reset_tick - 1, LogEvent.POSE_SET, START, 0)])

    result = replay(folder)

    assert result["verdict"] == complementary.MISMATCH
    first = result["segments"][0]["first_mismatch"]
    assert first["tick"] >= reset_tick
    assert {"x", "y"} & set(first["fields"])


def test_a_hole_in_the_ticks_restarts_warm_out_of_the_verdict(tmp_path):
    steps = inputs()
    first_reset, second_reset = FIRST_TICK + 20, FIRST_TICK + 250
    outputs = play_board(steps, before={first_reset: [("pose", *START)],
                                        second_reset: [("pose", 0.0, 0.0, 0.0)]})
    folder = write_run(
        tmp_path, steps, outputs,
        events=[(first_reset - 1, LogEvent.POSE_SET, START, 0),
                (second_reset - 1, LogEvent.POSE_SET, (0.0, 0.0, 0.0), 0)],
        drop={FIRST_TICK + 100, FIRST_TICK + 101, FIRST_TICK + 102})

    result = replay(folder)

    segment = result["segments"][0]
    assert segment["gaps"] == 1
    assert segment["warm_starts"] == 2              # start of log, the hole
    assert len(segment["anchors"]) == 2
    assert segment["approximate"] > 0
    assert result["verdict"] == complementary.OK


def test_ticks_under_the_ekf_are_skipped_and_the_switch_back_anchors(tmp_path):
    steps = inputs()
    to_ekf, back = FIRST_TICK + 60, FIRST_TICK + 90
    ekf = set(range(to_ekf, back))
    outputs = play_board(steps, before={back: [("last",)]}, ekf=ekf)
    folder = write_run(
        tmp_path, steps, outputs,
        events=[(to_ekf - 1, LogEvent.ESTIMATOR_SELECTED, (0, 0, 0),
                 Estimator.EKF6.value),
                (back - 1, LogEvent.ESTIMATOR_SELECTED, (0, 0, 0),
                 Estimator.COMPLEMENTARY.value)])

    result = replay(folder)

    segment = result["segments"][0]
    assert segment["skipped_ekf"] == len(ekf)
    assert segment["anchors"][0]["event"] == "ESTIMATOR_SELECTED"
    assert segment["anchored"] == TICKS - (back - FIRST_TICK)
    assert result["verdict"] == complementary.OK


# ── What cannot be judged ─────────────────────────────────────────────────

def test_without_any_reset_there_is_nothing_to_judge(tmp_path):
    steps = inputs()
    folder = write_run(tmp_path, steps, play_board(steps))

    result = replay(folder)

    assert result["verdict"] == complementary.INCONCLUSIVE
    assert result["segments"][0]["approximate"] == TICKS - 1


def test_a_run_without_log_tick_is_unavailable(tmp_path):
    run = SyntheticRun(tmp_path, trace_mask=TraceMask.NONE)
    for k in range(10):
        run.receive(k * 0.03, odometry(k * 30_000))
    folder = run.close()

    result = replay(folder)

    assert result == {"available": False,
                      "reason": "flux est_in / est absents (LOG_TICK)"}


# ── Tuning, command line, report ──────────────────────────────────────────

def test_changed_constants_give_a_replay_without_verdict(tmp_path):
    steps = inputs()
    reset_tick = FIRST_TICK + 20
    outputs = play_board(steps, before={reset_tick: [("pose", *START)]},
                         alpha=0.6)
    folder = write_run(tmp_path, steps, outputs,
                       events=[(reset_tick - 1, LogEvent.POSE_SET, START, 0)])

    result = replay(folder, overrides={"alpha_encoders": "0.6"})

    assert result["verdict"] == complementary.NO_VERDICT
    assert result["params"]["alpha_encoders"] == 0.6
    final = result["segments"][0]["final"]
    # The board ran with 0.6 too: the replay ends where it ended.
    assert final["replay"][0] == pytest.approx(final["board"][0], abs=1e-3)


def test_an_unknown_constant_is_refused():
    params = complementary.params_from_log(LOG_PARAMS_ROW)
    with pytest.raises(KeyError):
        complementary.apply_overrides(params, {"alpha": 0.3})


def test_command_line_writes_the_result_and_the_report_shows_it(
        tmp_path, monkeypatch, capsys):
    steps = inputs()
    reset_tick = FIRST_TICK + 20
    outputs = play_board(steps, before={reset_tick: [("pose", *START)]})
    folder = write_run(tmp_path, steps, outputs,
                       events=[(reset_tick - 1, LogEvent.POSE_SET, START, 0)])

    monkeypatch.setattr("sys.argv", ["complementary", str(folder)])
    assert complementary.main() == 0
    assert "REJEU : OK" in capsys.readouterr().out
    written = json.loads((folder / complementary.RESULT_FILE).read_text("utf-8"))
    assert written["verdict"] == complementary.OK

    section = report.section_replay(load_run(folder, verify=False))
    assert section["available"] and section["verdict"] == complementary.OK
    lines = report._lines_replay(section)
    assert any("REJEU : OK" in line for line in lines)


def test_the_report_says_when_the_replay_was_not_run(tmp_path):
    steps = inputs()
    folder = write_run(tmp_path, steps, play_board(steps))
    section = report.section_replay(load_run(folder, verify=False))
    assert not section["available"]
    assert "estimators.complementary" in section["reason"]


def test_messages_used_by_the_replay_exist():
    # The replay reads these names: a rename in the protocol must fail here.
    assert Messages.LOG_PARAMS and LogEvent.POSE_SET and LogEvent.ESTIMATOR_SELECTED
