"""Tests of the PID_TRACE of sim2d: sent only when asked, in firmware order.

    python -m pytest tests/test_sim2d_trace.py
"""

import pytest
from usb_com import TAP_RX, Messages, TraceMask, pack_message
from usb_com.python.messages import unpack_fields

from robot1.sim2d import transport
from robot1.sim2d.robot import SimulatedRobot
from robot1.sim2d.teensy import CONTROL_PERIOD_S


def simulated_link():
    """A robot at rest and its link, every received frame recorded."""
    robot = SimulatedRobot(x=1000.0, y=1000.0, theta=0.0)
    transport.set_robot(robot)
    com = transport.Sim2dCom()
    received = []
    com.set_tap(lambda direction, payload: received.append(
        (direction, bytes(payload))))
    return robot, com, received


def frames(received, message):
    return [unpack_fields(message, payload[1:]) for direction, payload in
            received if direction == TAP_RX and payload[0] == message.value]


def test_no_trace_until_set_trace_asks():
    robot, com, received = simulated_link()
    com.send_bytes(pack_message(Messages.SET_TARGET, 1500.0, 1000.0, 0.0,
                                0.0, 0.0, 0.0))
    robot.step(0.3)
    com.publish_odometry(robot.time)
    assert frames(received, Messages.ODOMETRY)
    assert frames(received, Messages.PID_TRACE) == []


def test_trace_of_every_tick_in_firmware_order():
    robot, com, received = simulated_link()
    com.send_bytes(pack_message(Messages.SET_TRACE, int(TraceMask.PID_TRACE)))
    com.send_bytes(pack_message(Messages.SET_TARGET, 1500.0, 1000.0, 0.0,
                                0.0, 0.0, 0.0))
    for _ in range(20):
        robot.step(0.05)
        com.publish_odometry(robot.time)

    traces = frames(received, Messages.PID_TRACE)
    assert len(traces) == pytest.approx(1.0 / CONTROL_PERIOD_S, abs=1)
    times = [trace["t_us"] for trace in traces]
    assert times == sorted(times)
    assert all(b - a == 15_000 for a, b in zip(times, times[1:]))

    # By tick, ODOMETRY before PID_TRACE, as the firmware sends them.
    order = [(unpack_fields(Messages(p[0]), p[1:])["t_us"], p[0])
             for d, p in received if d == TAP_RX]
    assert order == sorted(order)

    # The reference is the target; the measured speed is the command of the
    # tick before (ideal wheels, sampled one period later).
    assert traces[-1]["target"][0] == pytest.approx(1500.0)
    for before, after in zip(traces, traces[1:]):
        assert after["wheel_measured"] == pytest.approx(before["wheel_cmd"])
    assert any(trace["flags"] for trace in traces)       # it ramped up


def test_set_trace_off_stops_it():
    robot, com, received = simulated_link()
    com.send_bytes(pack_message(Messages.SET_TRACE, int(TraceMask.PID_TRACE)))
    robot.step(0.1)
    com.send_bytes(pack_message(Messages.SET_TRACE, 0))
    com.publish_odometry(robot.time)
    count = len(frames(received, Messages.PID_TRACE))
    robot.step(0.5)
    com.publish_odometry(robot.time)
    assert len(frames(received, Messages.PID_TRACE)) == count
