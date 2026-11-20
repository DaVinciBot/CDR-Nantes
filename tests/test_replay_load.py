"""Tests of replay/load.py: one table per stream, and the checks of a run.

    python -m pytest tests/test_replay_load.py
"""

from usb_com import TraceMask

from robot1.replay.load import flatten, load_run

from log_helpers import TICK_US, SyntheticRun, log_tick, odometry, pid_trace


def tick_run(root, ticks=600, lost_ticks=(), lone_ticks=()):
    """A session of LOG_TICK and PID_TRACE.

    Args:
        lost_ticks: ticks whose LOG_TICK the Teensy dropped (seq skipped).
        lone_ticks: ticks without their PID_TRACE.
    """
    run = SyntheticRun(root)
    for tick in range(ticks):
        t_us = 1_000_000 + tick * TICK_US
        t_rasp = 5.0 + t_us * 1e-6 + 0.001
        if tick not in lost_ticks:
            run.receive(t_rasp, log_tick(
                tick, tick, t_us, enc_counts=(tick, -tick, 2 * tick),
                est_pose=(10.0, 20.0, 0.5), work_us=4000))
        if tick not in lone_ticks:
            run.receive(t_rasp + 1e-5, pid_trace(
                t_us, target=(100.0, 200.0, 0.5), wheel_cmd=(1.0, 2.0, 3.0)))
    return run.close()


def test_array_fields_get_named_columns():
    row = flatten({"stream": "ctrl", "t": 1.0, "target": [1, 2, 3],
                   "velocity_cmd": [4, 5, 6], "wheel_cmd": [7, 8, 9],
                   "otos_acc": [0, 0, 1], "points": [[0, 0], [1, 1]]})
    assert row == {
        "t": 1.0, "target_x": 1, "target_y": 2, "target_theta": 3,
        "velocity_cmd_vx": 4, "velocity_cmd_vy": 5, "velocity_cmd_omega": 6,
        "wheel_cmd_1": 7, "wheel_cmd_2": 8, "wheel_cmd_3": 9,
        "otos_acc_x": 0, "otos_acc_y": 0, "otos_acc_h": 1,
        "points": [[0, 0], [1, 1]],
    }


def test_a_whole_session_loads_without_warning(tmp_path):
    run = load_run(tick_run(tmp_path))
    assert run.warnings == []
    assert run.counters["incomplete_ticks"] == 0

    ctrl = run.stream("ctrl")
    assert len(ctrl) == 600
    assert (ctrl["target_x"] == 100.0).all() and (ctrl["wheel_cmd_3"] == 3.0).all()
    assert ctrl["t"].iloc[0] == 1.0

    ticks = run.tick_table()
    assert len(ticks) == 600
    assert list(ticks["enc_counts_3"][:3]) == [0, 2, 4]
    assert (ticks["est_pose_theta"] == 0.5).all()
    assert (ticks["work_us"] == 4000).all()
    # The header of the tick appears once, not once per block.
    assert "t_us" in ticks and "t_us_x" not in ticks and "seq_y" not in ticks


def test_lost_frames_and_incomplete_ticks_are_reported(tmp_path):
    run = load_run(tick_run(tmp_path, lost_ticks=(100, 101), lone_ticks=(300,)))
    assert run.counters["seq_gaps"]["LOG_TICK"] == 2
    assert run.counters["incomplete_ticks"] == 3
    text = " | ".join(run.warnings)
    assert "2 LOG_TICK manquants" in text
    assert "3 ticks incomplets" in text
    assert len(run.tick_table()) == 598


def test_streams_asked_and_never_received_are_named(tmp_path):
    run = SyntheticRun(tmp_path)
    for tick in range(100):
        run.receive(1.0 + tick * 0.03, odometry(tick * 30_000))
    loaded = load_run(run.close())
    text = " | ".join(loaded.warnings)
    assert "PID_TRACE demandee" in text and "LOG_TICK demande" in text
    # Absent streams are empty tables, never a KeyError.
    assert loaded.stream("ctrl").empty and not loaded.has("est_in")
    assert loaded.tick_table().empty


def test_nothing_asked_nothing_missing(tmp_path):
    run = SyntheticRun(tmp_path, trace_mask=TraceMask.NONE)
    run.receive(1.0, odometry(500_000))
    assert load_run(run.close()).warnings == []
