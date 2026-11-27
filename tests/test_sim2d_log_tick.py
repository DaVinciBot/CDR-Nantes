"""Tests of the LOG_TICK of sim2d: sensors rebuilt from the true motion.

    python -m pytest tests/test_sim2d_log_tick.py
"""

import math
import sys

import pytest
from usb_com import TAP_RX, Messages, TraceMask, pack_message
from usb_com.python.messages import LogInputFlag, unpack_fields

from robot1.sim2d import run, transport
from robot1.sim2d.kinematics import WHEEL_DIAMETER_MM, forward_kinematics
from robot1.sim2d.robot import SimulatedRobot
from robot1.sim2d.sensors import COUNTS_PER_REV

COUNTS_TO_MM = math.pi * WHEEL_DIAMETER_MM / COUNTS_PER_REV


def simulated_link(mask=TraceMask.LOG_TICK):
    """A robot at rest, its link, and every received frame recorded."""
    robot = SimulatedRobot(x=1000.0, y=1000.0, theta=0.0)
    transport.set_robot(robot)
    com = transport.Sim2dCom()
    received = []
    com.set_tap(lambda direction, payload: received.append(
        (direction, bytes(payload))))
    if mask:
        com.send_bytes(pack_message(Messages.SET_TRACE, int(mask)))
    return robot, com, received


def frames(received, message):
    return [unpack_fields(message, payload[1:]) for direction, payload in
            received if direction == TAP_RX and payload[0] == message.value]


def drive(robot, com, x, y, theta=0.0, seconds=2.0):
    """Send one SET_TARGET and run the loop for a while, 50 ms steps."""
    com.send_bytes(pack_message(Messages.SET_TARGET, x, y, theta,
                                0.0, 0.0, 0.0))
    for _ in range(int(round(seconds / 0.05))):
        robot.step(0.05)
        com.publish_odometry(robot.time)


def test_no_log_tick_until_set_trace_asks():
    robot, com, received = simulated_link(mask=TraceMask.NONE)
    drive(robot, com, 1200.0, 1000.0, seconds=0.5)
    assert frames(received, Messages.ODOMETRY)
    assert frames(received, Messages.LOG_TICK) == []


def test_one_log_tick_per_tick_numbered_like_the_firmware():
    robot, com, received = simulated_link()
    drive(robot, com, 1200.0, 1000.0, seconds=1.0)

    ticks = frames(received, Messages.LOG_TICK)
    assert frames(received, Messages.PID_TRACE) == []      # LOG_TICK alone
    assert len(ticks) == pytest.approx(1.0 / 0.015, abs=1)
    assert [t["seq"] for t in ticks] == list(range(len(ticks)))
    assert [t["tick"] for t in ticks] == list(range(1, len(ticks) + 1))
    assert all(b["t_us"] - a["t_us"] == 15_000 for a, b in zip(ticks, ticks[1:]))

    assert ticks[0]["input_flags"] & LogInputFlag.FIRST_FRAME
    assert not ticks[0]["input_flags"] & LogInputFlag.OPTICAL_STEP_VALID
    for tick in ticks[1:]:
        flags = tick["input_flags"]
        assert not flags & LogInputFlag.FIRST_FRAME
        assert flags & LogInputFlag.ENCODERS_FRESH
        assert flags & LogInputFlag.IMU_VALID
        assert flags & LogInputFlag.OPTICAL_STEP_VALID
        assert tick["dt_us"] == 15_000


def test_with_pid_trace_every_tick_has_both_in_firmware_order():
    robot, com, received = simulated_link(
        TraceMask.LOG_TICK | TraceMask.PID_TRACE)
    drive(robot, com, 1200.0, 1000.0, seconds=0.5)
    ticks = frames(received, Messages.LOG_TICK)
    traces = frames(received, Messages.PID_TRACE)
    assert [t["t_us"] for t in ticks] == [t["t_us"] for t in traces]

    # By tick: ODOMETRY, then LOG_TICK, then PID_TRACE.
    rank = {Messages.ODOMETRY.value: 0, Messages.LOG_TICK.value: 1,
            Messages.PID_TRACE.value: 2}
    order = [(unpack_fields(Messages(p[0]), p[1:])["t_us"], rank[p[0]])
             for d, p in received if d == TAP_RX]
    assert order == sorted(order)


def test_encoders_count_the_true_motion():
    robot, com, received = simulated_link()
    drive(robot, com, 1300.0, 1150.0, theta=0.5, seconds=4.0)
    ticks = frames(received, Messages.LOG_TICK)

    # Raw counts run opposite to the forward deltas, and add up exactly.
    for i in range(3):
        total = sum(t["d"][i] for t in ticks)
        assert ticks[-1]["enc_counts"][i] - ticks[0]["enc_counts"][i] == -total

    # Integrating the deltas through the kinematics finds the true pose,
    # within the rounding of one count per tick.
    x, y, theta = 1000.0, 1000.0, 0.0
    for tick in ticks[1:]:
        rx, ry, dtheta = forward_kinematics(
            *(d * COUNTS_TO_MM for d in tick["d"]))
        mid = theta + 0.5 * dtheta
        x += rx * math.cos(mid) - ry * math.sin(mid)
        y += rx * math.sin(mid) + ry * math.cos(mid)
        theta += dtheta
    # The last LOG_TICK reads the motion up to its own tick: compare there.
    assert ticks[-1]["t_us"] * 1e-6 <= robot.time
    true_x, true_y, true_theta = robot.true_pose()
    assert math.hypot(x - true_x, y - true_y) < 1.0
    assert abs(theta - true_theta) < 1e-3


def test_optical_steps_add_up_to_the_true_displacement():
    robot, com, received = simulated_link()
    drive(robot, com, 1400.0, 1000.0, seconds=8.0)       # straight, settled
    ticks = frames(received, Messages.LOG_TICK)
    dx = sum(t["optical_dx_mm"] for t in ticks)
    dy = sum(t["optical_dy_mm"] for t in ticks)
    last = ticks[-1]
    # Steps follow the raw tracking position, which started at zero.
    assert dx == pytest.approx(last["otos_pos"][0], abs=1e-2)
    assert dy == pytest.approx(last["otos_pos"][1], abs=1e-2)
    assert last["otos_pos"][0] == pytest.approx(400.0, abs=15.0)
    assert last["heading_rad"] == pytest.approx(robot.true_theta, abs=1e-3)


def test_set_odometrie_moves_no_sensor():
    robot, com, received = simulated_link()
    drive(robot, com, 1200.0, 1000.0, seconds=8.0)       # settled
    before = frames(received, Messages.LOG_TICK)[-1]
    com.send_bytes(pack_message(Messages.SET_ODOMETRIE, 500.0, 500.0, 1.0))
    com.send_bytes(pack_message(Messages.SET_TARGET, 500.0, 500.0, 1.0,
                                0.0, 0.0, 0.0))
    robot.step(0.05)
    com.publish_odometry(robot.time)
    after = frames(received, Messages.LOG_TICK)[-1]
    # The robot was placed: the chip and the wheels saw no motion.
    assert after["otos_pos"] == pytest.approx(before["otos_pos"], abs=0.5)
    assert after["enc_counts"] == before["enc_counts"]
    assert after["est_pose"][:2] == pytest.approx([500.0, 500.0])


def test_estimator_output_is_the_odometry_and_commands_are_counted():
    robot, com, received = simulated_link()
    drive(robot, com, 1200.0, 1000.0, seconds=0.6)
    drive(robot, com, 1200.0, 1200.0, seconds=0.6)
    ticks = {t["t_us"]: t for t in frames(received, Messages.LOG_TICK)}
    odometry = frames(received, Messages.ODOMETRY)
    matched = [o for o in odometry if o["t_us"] in ticks]
    assert matched
    for odo in matched:
        tick = ticks[odo["t_us"]]
        assert tick["est_pose"] == pytest.approx(
            [odo["x"], odo["y"], odo["theta"]])
        assert tick["est_vel"] == pytest.approx(
            [odo["vx"], odo["vy"], odo["omega"]])
    assert max(t["cmd_count"] for t in ticks.values()) == 2


def test_carre_logged_with_both_streams_feeds_the_report(tmp_path, monkeypatch):
    pytest.importorskip("pandas")
    from robot1.replay import report
    from robot1.replay.load import load_run

    monkeypatch.setattr(sys, "argv", [
        "run", "--scenario", "carre", "--log", "--trace", "--log-tick",
        "--log-dir", str(tmp_path)])
    assert run.main() == 0
    (folder,) = tmp_path.iterdir()

    loaded = load_run(folder)
    assert not [w for w in loaded.warnings if "incomplet" in w]
    built = report.build_report(loaded)
    assert built["cycles"]["available"]
    assert built["cycles"]["ticks"] > 1000
    commands = built["commandes"]
    assert commands["available"]
    assert not commands["pairing_doubtful"]
    # Taken at the tick that follows reception: never negative, never more
    # than one control period.
    assert 0.0 <= commands["latency_ms"]["max"] <= 15.0
