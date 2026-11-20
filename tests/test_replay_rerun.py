"""Tests of replay/to_rerun.py: a run becomes a .rrd with the expected entities.

    python -m pytest tests/test_replay_rerun.py

Skipped when the Rerun SDK is not installed (pip install -e ".[analysis]").
"""

import pytest

pytest.importorskip("rerun")

from robot1.replay import to_rerun                                  # noqa: E402
from robot1.replay.load import load_run                             # noqa: E402

from log_helpers import (                                           # noqa: E402
    TICK_US,
    SyntheticRun,
    log_tick,
    odometry,
    pid_trace,
)


def test_a_run_is_exported_with_the_entities_of_the_bridge(tmp_path):
    run = SyntheticRun(tmp_path)
    for tick in range(200):
        t_us = 1_000_000 + tick * TICK_US
        t_rasp = 3.0 + t_us * 1e-6
        run.receive(t_rasp, log_tick(tick, tick, t_us, work_us=4000))
        run.receive(t_rasp, pid_trace(t_us, target=(100.0 + tick, 0.0, 0.0),
                                      pose=(float(tick), 0.0, 0.0),
                                      wheel_cmd=(1.0, 2.0, 3.0)))
        if tick % 2 == 0:
            run.receive(t_rasp, odometry(t_us, x=float(tick)))
        if tick % 4 == 0:
            run.emit(t_rasp, "state", target_x=500.0, target_y=500.0,
                     trajectory=[[0.0, 0.0], [500.0, 500.0]],
                     obstacles=[{"x": 1500.0, "y": 1000.0, "radius": 200.0}],
                     fused_x=float(tick), fused_y=0.0, fused_theta=0.0)
    run.emit(3.5, "strat", step=1, action="DEPLACEMENT")
    folder = run.close()

    output = tmp_path / "out.rrd"
    written = to_rerun.export(load_run(folder, verify=False), output)

    assert output.stat().st_size > 0
    for entity in ("world/robot/odom", "world/robot/fused",
                   "world/robot/reference", "world/pathfinding/target",
                   "world/pathfinding/trajectory",
                   "world/obstacles/adversaries", "data/teensy/x_mm",
                   "data/ctrl/x/target", "data/ctrl/x/pose",
                   "data/ctrl/roue_1/wheel_cmd", "data/health/work_us",
                   "logs/strat"):
        assert entity in written, entity


def test_column_entities_group_the_parts_of_an_array():
    assert to_rerun._column_entity("ctrl", "target_theta") == "data/ctrl/theta/target"
    assert to_rerun._column_entity("ctrl", "wheel_measured_2") == \
        "data/ctrl/roue_2/wheel_measured"
    assert to_rerun._column_entity("odometry", "vx") == "data/odometry/vx"
    assert to_rerun._column_entity("health", "work_us") == "data/health/work_us"
