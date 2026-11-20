"""Tests of replay/align.py: offset, drift, wrap and restarts of the Teensy.

    python -m pytest tests/test_replay_align.py

The lines are built as the decoder writes them: a Teensy frame carries t_us,
its time of reception t_rasp_ns, src and stream. The latency of the link is
drawn at random, never negative, with bursts where a whole second is late.
"""

import numpy as np
import pytest
from usb_com.python.messages import LogEvent

from robot1.replay import align
from robot1.replay.load import load_run

from log_helpers import SyntheticRun, odometry, pid_trace

TICK_S = 0.015
WRAP_S = 2 ** 32 * 1e-6


def teensy_line(t_teensy_s, t_rasp_s, stream="ctrl", **fields):
    """A decoded Teensy line: t_us is the wrapped micros() of t_teensy_s."""
    return {
        "t_rasp_ns": int(round(t_rasp_s * 1e9)), "src": "teensy",
        "stream": stream, "t_us": int(round(t_teensy_s * 1e6)) & 0xFFFFFFFF,
        **fields,
    }


def rasp_line(t_rasp_s, stream="state", **fields):
    return {"t_rasp_ns": int(round(t_rasp_s * 1e9)), "src": "rasp",
            "stream": stream, **fields}


def session(duration_s, offset_s, drift_ppm, *, boot_s=2.0, seed=0,
            stream="ctrl"):
    """Ticks of the Teensy seen by the Pi through a jittery link.

    Returns:
        tuple: (lines, true Teensy times).
    """
    rng = np.random.default_rng(seed)
    t_teensy = boot_s + np.arange(int(duration_s / TICK_S)) * TICK_S
    latency = rng.exponential(1e-3, len(t_teensy))
    # A busy Pi: one second out of ten, every frame is 5 to 25 ms late.
    late = (np.floor(t_teensy) % 10) == 3
    latency[late] += rng.uniform(5e-3, 25e-3, late.sum())
    t_rasp = offset_s + (1.0 + drift_ppm * 1e-6) * t_teensy + latency
    lines = [teensy_line(tt, tr, stream) for tt, tr in zip(t_teensy, t_rasp)]
    return lines, t_teensy


def test_offset_and_drift_injected_are_found():
    lines, t_teensy = session(120.0, offset_s=3.217, drift_ppm=40.0)
    alignment = align.align_lines(lines)

    assert alignment.method == align.METHOD_ENVELOPE
    (segment,) = alignment.segments
    assert segment.offset_s == pytest.approx(3.217, abs=0.2e-3)
    assert segment.drift_ppm == pytest.approx(40.0, abs=5.0)
    assert segment.latency_ms["p50"] < 2.0

    # A Teensy line keeps its own measurement time.
    assert [line["t"] for line in lines] == pytest.approx(list(t_teensy),
                                                         abs=1e-6)


def test_a_line_of_the_pi_is_moved_into_the_teensy_time():
    lines, _ = session(60.0, offset_s=-1.5, drift_ppm=-25.0)
    # The Pi logs something at the instant the Teensy read t = 30 s.
    t_rasp = -1.5 + (1.0 - 25e-6) * 30.0
    lines.append(rasp_line(t_rasp))
    lines.sort(key=lambda line: line["t_rasp_ns"])
    align.align_lines(lines)
    (state,) = [line for line in lines if line["src"] == "rasp"]
    assert state["t"] == pytest.approx(30.0, abs=0.3e-3)


def test_wrap_of_micros_is_unrolled():
    # 20 s that straddle the wrap of t_us (71.6 min after boot).
    lines, t_teensy = session(20.0, offset_s=0.5, drift_ppm=0.0,
                              boot_s=WRAP_S - 5.0)
    assert any(line["t_us"] < 10_000_000 for line in lines)

    alignment = align.align_lines(lines)
    assert alignment.restarts == 0
    times = [line["t"] for line in lines]
    assert times == pytest.approx(list(t_teensy), abs=1e-6)
    assert all(b > a for a, b in zip(times, times[1:]))


def test_frames_slightly_out_of_order_are_not_a_restart():
    lines, _ = session(10.0, offset_s=0.0, drift_ppm=0.0)
    # STATUS is sent from loop(): its t_us can be a little behind the last
    # per-tick frame received.
    lines.insert(300, teensy_line(2.0 + 299 * TICK_S - 0.004,
                                  lines[299]["t_rasp_ns"] * 1e-9 + 1e-4,
                                  stream="status"))
    alignment = align.align_lines(lines)
    assert alignment.restarts == 0


@pytest.mark.parametrize("with_boot_event", [True, False])
def test_a_restart_of_the_teensy_starts_a_segment(with_boot_event):
    first, _ = session(30.0, offset_s=10.0, drift_ppm=0.0)
    # Restart: the board boots again 2 s after the last frame, t_us from 0.
    restart_rasp = first[-1]["t_rasp_ns"] * 1e-9 + 2.0
    second, t_second = session(30.0, offset_s=restart_rasp - 0.5,
                               drift_ppm=0.0, boot_s=0.5, seed=1)
    if with_boot_event:
        second.insert(0, teensy_line(0.5, restart_rasp + 1e-4, "event",
                                     event=LogEvent.BOOT.value))
    lines = first + second

    alignment = align.align_lines(lines)
    assert alignment.restarts == 1
    assert alignment.segments[1].offset_s == pytest.approx(restart_rasp - 0.5,
                                                           abs=0.2e-3)
    assert {line["segment"] for line in lines[len(first):]} == {1}
    times = [line["t"] for line in lines if line["stream"] == "ctrl"]
    assert all(b > a for a, b in zip(times, times[1:]))
    # The second segment lands where the Pi saw it: 2 s after the first.
    gap = times[len(first)] - times[len(first) - 1]
    assert gap == pytest.approx(2.0, abs=1e-3)


def test_a_frame_dated_on_another_clock_does_not_bend_the_line():
    # The first frame of a sim2d match is dated before the clock jumped.
    lines, _ = session(60.0, offset_s=0.28, drift_ppm=0.0)
    lines.insert(0, teensy_line(1.9, 1.9, "odometry"))
    (segment,) = align.align_lines(lines).segments
    assert segment.offset_s == pytest.approx(0.28, abs=0.2e-3)
    assert segment.drift_ppm == pytest.approx(0.0, abs=5.0)


def test_a_run_without_teensy_keeps_the_time_of_the_pi():
    lines = [rasp_line(100.0 + 0.05 * i, "state") for i in range(10)]
    alignment = align.align_lines(lines, {"t_start_rasp_ns": 99_000_000_000})
    assert alignment.method == align.METHOD_RASP_ONLY
    assert lines[0]["t"] == pytest.approx(1.0)
    assert lines[-1]["t"] == pytest.approx(1.45)


def test_alignment_through_a_real_run_folder(tmp_path):
    run = SyntheticRun(tmp_path)
    rng = np.random.default_rng(3)
    for tick in range(4000):                     # 60 s
        t_teensy = 1.0 + tick * TICK_S
        t_us = int(round(t_teensy * 1e6))
        t_rasp = 7.5 + (1.0 + 30e-6) * t_teensy + rng.exponential(1e-3)
        run.receive(t_rasp, pid_trace(t_us))
        if tick % 2 == 0:
            run.receive(t_rasp + 1e-5, odometry(t_us))
    folder = run.close()

    lines, alignment = align.align_run(folder)
    (segment,) = alignment.segments
    assert segment.offset_s == pytest.approx(7.5, abs=0.2e-3)
    assert segment.drift_ppm == pytest.approx(30.0, abs=5.0)
    assert all("t" in line for line in lines)

    loaded = load_run(folder)
    assert loaded.stream("ctrl")["t"].iloc[0] == pytest.approx(1.0, abs=1e-6)
