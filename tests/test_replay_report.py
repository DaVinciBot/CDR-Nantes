"""Tests of replay/report.py on runs whose answers are known.

    python -m pytest tests/test_replay_report.py

The wheels follow a first order with a pure delay, the pose a first order:
the report must find the delay, the time constant and the step metrics they
were built with.
"""

import json
import math

import numpy as np
import pytest
from usb_com.python.messages import LogHealthFlag, Rs485Fail, TraceFlag

from robot1.rasp.telemetry import trial
from robot1.replay import report
from robot1.replay.load import load_run

from log_helpers import (
    TICK_US,
    SyntheticRun,
    log_tick,
    odometry,
    pid_trace,
    set_target,
)

TICK_S = TICK_US * 1e-6


def first_order(command, tau_s, delay_ticks, gain=1.0, noise=0.0, seed=0):
    """measured[k+1] = a measured[k] + (1 - a) gain command[k - delay]."""
    rng = np.random.default_rng(seed)
    a = math.exp(-TICK_S / tau_s)
    measured = np.zeros(len(command))
    for k in range(len(command) - 1):
        source = command[k - delay_ticks] if k >= delay_ticks else 0.0
        measured[k + 1] = a * measured[k] + (1.0 - a) * gain * source
    return measured + rng.normal(0.0, noise, len(command)) if noise else measured


def wheel_command(ticks, hold=40, seed=1):
    """Speed levels held for `hold` ticks, rpm."""
    rng = np.random.default_rng(seed)
    levels = rng.uniform(-200.0, 200.0, ticks // hold + 1)
    return np.repeat(levels, hold)[:ticks]


# ── Pieces ────────────────────────────────────────────────────────────────

def test_delay_time_constant_and_gain_of_a_wheel_are_found():
    command = wheel_command(2000)
    measured = first_order(command, tau_s=0.08, delay_ticks=2, gain=0.95,
                           noise=0.3)
    fit = report.fit_first_order(command, measured, TICK_S)
    assert fit["delay_ticks"] == 2
    assert fit["tau_s"] == pytest.approx(0.08, rel=0.10)
    assert fit["gain"] == pytest.approx(0.95, rel=0.02)
    assert fit["r2"] > 0.99


def test_a_wheel_that_never_moves_tells_nothing():
    still = np.zeros(500)
    assert report.fit_first_order(still, still, TICK_S) is None


def test_step_metrics_of_a_first_order():
    t = np.arange(1, 600) * 0.005
    progress = 1.0 - np.exp(-t / 0.1)
    metrics = report.step_metrics(t, progress, 0.0, 500.0, 10.0)
    assert metrics["rise_s"] == pytest.approx(0.1 * math.log(9.0), abs=0.01)
    assert metrics["overshoot_pct"] == 0.0
    assert metrics["static_error"] == pytest.approx(0.0, abs=1e-3)
    assert metrics["settling_s"] == pytest.approx(0.1 * math.log(50.0), abs=0.01)


def test_overshoot_and_a_step_that_never_settles():
    t = np.arange(1, 400) * 0.005
    progress = 1.0 - np.exp(-3.0 * t) * np.cos(8.0 * t)
    metrics = report.step_metrics(t, progress, 0.0, 100.0, 2.0)
    assert metrics["overshoot_pct"] == pytest.approx((progress.max() - 1) * 100)
    assert metrics["overshoot_pct"] > 10.0

    stuck = np.minimum(t, 0.5)                     # stops at half the step
    metrics = report.step_metrics(t, stuck, 0.0, 100.0, 2.0)
    assert metrics["settling_s"] is None
    assert metrics["static_error"] == pytest.approx(50.0)
    assert metrics["rise_s"] is None


# ── A whole run ───────────────────────────────────────────────────────────

BOOT_S = 1.0
OFFSET_S = 5.0
STEP_TICK = 100
STEP_MM = 500.0
POSE_TAU_S = 0.3
WHEEL_TAU_S = 0.05
COMMAND_TICKS = (100, 200, 300, 400, 500)
COMMAND_LATENCY_S = 0.003


def control_run(root, ticks=600):
    """A trial with everything the report reads, and known answers."""
    rng = np.random.default_rng(7)
    command = wheel_command(ticks)
    measured = first_order(command, WHEEL_TAU_S, delay_ticks=1)
    run = SyntheticRun(root, mode="hardware", meta={"test_type": "sequence"})
    cmd_count = 0
    cmd_t_us = 0
    for tick in range(ticks):
        t_teensy = BOOT_S + tick * TICK_S
        t_us = int(round(t_teensy * 1e6))
        t_rasp = OFFSET_S + t_teensy + rng.exponential(5e-4)

        if tick + 1 in COMMAND_TICKS:
            # Sent by the Pi 5 ms into this tick, taken 3 ms later.
            run.send(OFFSET_S + t_teensy + 0.005, set_target(STEP_MM, 0.0, 0.0))
        if tick in COMMAND_TICKS:
            cmd_count += 1
            cmd_t_us = int(round((t_teensy - TICK_S + 0.005
                                  + COMMAND_LATENCY_S) * 1e6))

        stepped = tick >= STEP_TICK
        t_step = BOOT_S + STEP_TICK * TICK_S
        pose_x = (STEP_MM * (1.0 - math.exp(-(t_teensy - t_step) / POSE_TAU_S))
                  if stepped else 0.0)
        overrun = tick in (200, 201)
        run.receive(t_rasp, log_tick(
            tick, tick, t_us,
            work_us=16_000 if overrun else 5_000,
            health_flags=LogHealthFlag.OVERRUN if overrun else 0,
            rs485_fail=(0, Rs485Fail.NO_ANSWER.value if tick in (50, 51, 52)
                        else 0, 0),
            cmd_count=cmd_count, cmd_t_us=cmd_t_us, otos_read_us=900,
            est_pose=(pose_x, 0.0, 0.0)))
        run.receive(t_rasp + 1e-5, pid_trace(
            t_us,
            target=(STEP_MM if stepped else 0.0, 0.0, 0.0),
            pose=(pose_x, 0.0, 0.0),
            velocity_cmd=(100.0 if stepped else 0.0, 0.0, 0.0),
            wheel_cmd=[command[tick]] * 3,
            wheel_measured=[measured[tick]] * 3,
            flags=TraceFlag.ACCEL_LIMITED if 120 <= tick < 130 else 0))
        if tick % 2 == 0:
            run.receive(t_rasp + 2e-5, odometry(t_us, x=pose_x))
    folder = run.close()
    trial.write_truth(folder, trial.truth_pose([STEP_MM - 5.0, 0.0, 0.0]))
    return folder


@pytest.fixture(scope="module")
def full_report(tmp_path_factory):
    folder = control_run(tmp_path_factory.mktemp("runs"))
    return folder, report.build_report(load_run(folder))


def test_link_and_cycles(full_report):
    _, result = full_report
    assert result["warnings"] == []
    assert result["lien"]["ok"] is True
    cycles = result["cycles"]
    assert cycles["work_us"]["p50"] == 5000.0
    assert cycles["overruns"] == 2
    assert cycles["rs485_fail"]["2"] == {"NO_ANSWER": 3}
    assert cycles["rs485_fail"]["1"] == {}
    assert cycles["otos_read_us"]["max"] == 900.0


def test_command_latency(full_report):
    _, result = full_report
    commands = result["commandes"]
    assert commands["paired"] == len(COMMAND_TICKS)
    assert commands["latency_ms"]["p50"] == pytest.approx(
        COMMAND_LATENCY_S * 1e3, abs=0.2)
    assert not commands["pairing_doubtful"]


def test_step_response_of_the_pose(full_report):
    _, result = full_report
    steps = result["echelons"]
    assert steps["source"] == "ctrl"
    (step,) = steps["steps"]
    assert step["kind"] == "translation"
    assert step["size"] == pytest.approx(STEP_MM)
    assert step["rise_s"] == pytest.approx(POSE_TAU_S * math.log(9.0), abs=0.03)
    assert step["overshoot_pct"] == 0.0
    assert step["settling_s"] == pytest.approx(POSE_TAU_S * math.log(50.0),
                                               abs=0.03)


def test_wheels_and_tracking(full_report):
    _, result = full_report
    for fit in result["roues"]["wheels"].values():
        assert fit["delay_ticks"] == 1
        assert fit["tau_s"] == pytest.approx(WHEEL_TAU_S, rel=0.10)
        assert fit["gain"] == pytest.approx(1.0, rel=0.02)
    tracking = result["suivi"]
    assert tracking["saturations"]["ACCEL_LIMITED"] == {"ticks": 10,
                                                        "episodes": 1}
    assert tracking["error_mm"]["max"] == pytest.approx(STEP_MM)
    assert tracking["lost_steps"] == {"1": 0, "2": 0, "3": 0}


def test_ground_truth_and_written_report(full_report):
    folder, result = full_report
    truth = result["verite"]
    assert truth["source"] == "est"
    assert truth["error_mm"] == pytest.approx(5.0, abs=0.1)

    text = "\n".join(report.format_report(result))
    # Every section but the replays of the estimators, which are run apart.
    assert not result["rejeu"]["available"]
    assert not result["ekf"]["available"]
    assert text.count("non disponible") == 2
    written = json.loads(report.write_report(result, folder).read_text("utf-8"))
    assert written["roues"]["wheels"]["1"]["delay_ticks"] == 1


def test_a_run_with_little_in_it_still_reports(tmp_path):
    run = SyntheticRun(tmp_path)
    for tick in range(100):
        run.receive(1.0 + tick * 0.03, odometry(tick * 30_000, x=float(tick)))
    folder = run.close()

    result = report.build_report(load_run(folder))
    for section in ("cycles", "commandes", "suivi", "echelons", "roues",
                    "verite"):
        assert result[section]["available"] is False, section
        assert result[section]["reason"]
    text = "\n".join(report.format_report(result))
    assert "non disponible : flux health absent (LOG_TICK)" in text
    json.loads(report.write_report(result, folder).read_text("utf-8"))
