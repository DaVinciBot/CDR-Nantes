"""Tests of the trials: types, ground truth, verdict, and the run_test tool.

    pip install -e ".[dev]"
    python -m pytest tests

No robot: a fake link plays the Teensy. It answers a pose target with the
odometry of a robot that got there, and with LOG_TICK lines when asked to.
"""

import math
import struct

import pytest

from usb_com import TAP_RX, TAP_TX, Messages, TraceMask, pack_message
from usb_com.python.messages import PAYLOAD_FORMATS

from robot1.rasp.telemetry import log_reader
from robot1.rasp.telemetry import trial as trials
from robot1.rasp.tools import run_test


class FakeTeensy:
    """A link whose far end behaves like a base that reaches its targets."""

    def __init__(self, sends_log_tick=True, error_mm=0.0):
        self.tap = None
        self.callbacks = {}
        self.sent = []
        self.sends_log_tick = sends_log_tick
        self.error_mm = error_mm
        self.mask = 0
        self.seq = 0

    def add_callback(self, func, iid):
        self.callbacks[iid] = func

    def read_bytes(self):
        return b""

    def set_tap(self, tap):
        self.tap = tap

    def send_bytes(self, data):
        self.sent.append(bytes(data))
        if self.tap is not None:
            self.tap(TAP_TX, data)
        message_id = data[0]
        if message_id == Messages.SET_TRACE.value:
            self.mask = data[1]
        elif message_id == Messages.SET_TARGET.value:
            x, y, theta = struct.unpack("<3f", data[1:13])
            self._deliver(pack_message(
                Messages.ODOMETRY, x + self.error_mm, y, theta, 0, 0, 0, 1000))
            if self.sends_log_tick and self.mask & TraceMask.LOG_TICK:
                for _ in range(5):
                    self._deliver(self._log_tick())

    def _log_tick(self):
        fmt = PAYLOAD_FORMATS[Messages.LOG_TICK]
        count = len(struct.unpack(fmt, bytes(struct.calcsize(fmt))))
        values = [0] * count
        values[0] = 1
        values[1] = self.seq
        values[2] = self.seq
        self.seq += 1
        return pack_message(Messages.LOG_TICK, *values)

    def _deliver(self, payload):
        if self.tap is not None:
            self.tap(TAP_RX, payload)
        callback = self.callbacks.get(payload[0])
        if callback is not None:
            callback(payload[1:])

    def messages(self, message):
        return [data for data in self.sent if data[0] == message.value]


def parse(tmp_path, *argv):
    return run_test.build_parser().parse_args(
        ["--log-dir", str(tmp_path), "--yes", "--duration", "0", *argv])


def run(tmp_path, com, *argv, answers=()):
    """Run the tool against a fake link; return (code, printed, folder)."""
    printed = []
    replies = iter(answers)
    code = run_test.run_trial(
        parse(tmp_path, *argv), com=com,
        input_fn=lambda _prompt: next(replies),
        sleep=lambda _seconds: None, out=printed.append,
    )
    folders = sorted(path for path in tmp_path.iterdir() if path.is_dir())
    return code, "\n".join(printed), folders[-1] if folders else None


# ── Types and ground truth ────────────────────────────────────────────────

def test_every_type_says_what_truth_it_needs():
    for trial in trials.TRIAL_TYPES.values():
        assert trial.truth in (trials.TRUTH_NONE, trials.TRUTH_POSE,
                               trials.TRUTH_TURNS)
        # A type without truth must say why: it is a choice, not an oversight.
        assert (trial.truth != trials.TRUTH_NONE) or trial.truth_reason
        assert trial.description and trial.session
    assert trials.TRIAL_TYPES["otos_lin"].truth == trials.TRUTH_POSE
    assert trials.TRIAL_TYPES["otos_ang"].truth == trials.TRUTH_TURNS
    assert not trials.TRIAL_TYPES["check"].calibration


def test_a_pose_is_typed_in_degrees_and_stored_in_radians():
    assert trials.parse_pose("1250 1000 90") == [1250.0, 1000.0,
                                                 pytest.approx(math.pi / 2)]
    assert trials.parse_pose("1250, 1000.5, -45") == [
        1250.0, 1000.5, pytest.approx(-math.pi / 4)]
    for bad in ("1250 1000", "a b c", "1 2 3 4", "", "1 2 nan"):
        with pytest.raises(ValueError):
            trials.parse_pose(bad)


def test_truth_is_validated_before_it_is_written(tmp_path):
    pose = trials.truth_pose([1250.0, 1000.0, 0.0], uncertainty_mm=3.0)
    assert pose["kind"] == "pose" and pose["uncertainty_mm"] == 3.0
    trials.write_truth(tmp_path, pose)
    assert trials.read_truth(tmp_path) == pose

    assert trials.truth_turns(-10)["turns"] == -10.0
    assert trials.truth_none(" robot immobile ")["reason"] == "robot immobile"

    with pytest.raises(ValueError):
        trials.truth_pose([1.0, 2.0])
    with pytest.raises(ValueError):
        trials.truth_pose([1.0, 2.0, float("inf")])
    with pytest.raises(ValueError):
        trials.truth_pose([1.0, 2.0, 0.0], uncertainty_mm=-1.0)
    with pytest.raises(ValueError):
        trials.truth_turns(0)
    with pytest.raises(ValueError):
        trials.truth_none("   ")
    assert trials.read_truth(tmp_path / "nowhere") is None


def test_a_calibration_trial_needs_log_tick():
    otos = trials.TRIAL_TYPES["otos_lin"]
    assert trials.start_problems(otos, log_tick=True) == []
    assert len(trials.start_problems(otos, log_tick=False)) == 1
    assert trials.start_problems(trials.TRIAL_TYPES["check"], log_tick=False) == []


# ── The tool, from start to verdict ───────────────────────────────────────

def test_linear_trial_with_its_truth_is_usable(tmp_path):
    com = FakeTeensy(error_mm=4.0)
    code, printed, folder = run(
        tmp_path, com, "--type", "otos_lin", "--note", "1 m le long de x",
        "--operator", "Antoine", "--start-pose", "250 1000 0",
        "--target", "1250 1000 0", "--truth", "1250 1000 0",
    )
    assert code == 0, printed
    assert "ESSAI : OK" in printed
    # Estimate against truth, on the spot: the 4 mm the fake base is off by.
    assert "dx = +4.0 mm" in printed

    # What the Teensy was told, in order: everything recorded, pose, target,
    # then a real STOP whatever happened.
    assert com.messages(Messages.SET_TRACE) == [
        pack_message(Messages.SET_TRACE, TraceMask.PID_TRACE | TraceMask.LOG_TICK)]
    assert [data[0] for data in com.sent] == [
        Messages.SET_TRACE.value, Messages.SET_ODOMETRIE.value,
        Messages.SET_TARGET.value, Messages.STOP.value]

    meta = log_reader.read_meta(folder)
    assert meta["test_type"] == "otos_lin" and meta["note"] == "1 m le long de x"
    assert meta["operator"] == "Antoine" and meta["partial"] is False
    assert meta["start_pose_table"] == [250.0, 1000.0, 0.0]
    assert meta["final_pose_estimated"] == [1254.0, 1000.0, 0.0]
    assert meta["streams"]["log_tick"] is True
    assert "otos_lin" in folder.name

    truth = trials.read_truth(folder)
    assert truth["final_pose_table"] == [1250.0, 1000.0, 0.0]

    lines = log_reader.read_lines(folder)
    events = [line["event"] for line in log_reader.stream_lines(lines, "trial")]
    assert events == ["start", "target", "end"]
    (target,) = [line for line in lines if line.get("event") == "target"]
    assert target["reached"] is True


def test_truth_is_asked_until_it_can_be_read(tmp_path):
    com = FakeTeensy()
    code, printed, folder = run(
        tmp_path, com, "--type", "otos_lin", "--target", "1250 1000 0",
        answers=["1250 mille 0", "1250 1000", "1250 1000 0"],
    )
    assert code == 0, printed
    assert printed.count("Non compris") == 2
    assert trials.read_truth(folder)["kind"] == "pose"


def test_turns_trial_takes_a_number_of_turns(tmp_path):
    com = FakeTeensy()
    code, printed, folder = run(
        tmp_path, com, "--type", "otos_ang", "--target", "1500 1000 0",
        answers=["zero", "0", "-10"],
    )
    assert code == 0, printed
    assert trials.read_truth(folder) == {
        "kind": "turns", "method": "marque_au_sol", "turns": -10.0,
        "uncertainty_rad": pytest.approx(math.radians(0.5)), "note": "",
    }


def test_a_static_trial_needs_no_question(tmp_path):
    com = FakeTeensy()
    com_target = pack_message(Messages.SET_TARGET, 0.0, 0.0, 0.0, 0, 0, 0)
    # The base is not commanded, but the Teensy still logs while it stands.
    code, printed, folder = run(tmp_path, com, "--type", "check")
    assert code == 0, printed
    assert trials.read_truth(folder)["kind"] == "none"
    assert com_target not in com.sent


def test_closing_without_truth_must_be_said_and_makes_the_trial_unusable(tmp_path):
    com = FakeTeensy()
    # An empty reason is refused and the question comes back: "no truth" has
    # to be a decision with its reason, not a key pressed too fast.
    code, printed, folder = run(
        tmp_path, com, "--type", "otos_lin", "--target", "1250 1000 0",
        answers=["aucune", "", "aucune", "marques effacees"],
    )
    assert printed.count("Non compris") == 1
    assert code == 1
    assert "inutilisable pour calibrer" in printed
    assert trials.read_truth(folder) == {"kind": "none",
                                         "reason": "marques effacees"}
    # The log itself is whole: only the truth is missing.
    assert log_reader.check_run(folder).ok


def test_a_calibration_trial_refuses_to_start_without_log_tick(tmp_path):
    com = FakeTeensy()
    code, printed, folder = run(tmp_path, com, "--type", "otos_lin",
                                "--no-log-tick", "--truth", "1250 1000 0")
    assert code == 2 and "REFUS" in printed
    assert folder is None and com.sent == []       # nothing opened, nothing sent

    # Forced: it runs, says it is partial, and the verdict says what it lacks.
    code, printed, folder = run(
        tmp_path, com, "--type", "otos_lin", "--no-log-tick",
        "--allow-partial", "--target", "1250 1000 0", "--truth", "1250 1000 0")
    assert code == 1
    assert "aucun LOG_TICK recu" in printed
    meta = log_reader.read_meta(folder)
    assert meta["partial"] is True and meta["streams"]["log_tick"] is False


def test_a_teensy_that_sends_no_log_tick_is_caught_at_the_end(tmp_path):
    com = FakeTeensy(sends_log_tick=False)         # older firmware, or a bug
    code, printed, folder = run(
        tmp_path, com, "--type", "otos_lin", "--target", "1250 1000 0",
        "--truth", "1250 1000 0")
    assert code == 1
    assert "aucun LOG_TICK recu" in printed
    assert log_reader.check_run(folder).ok         # the log is whole, but empty


def test_a_bad_argument_is_refused_before_anything_is_opened(tmp_path):
    com = FakeTeensy()
    for argv in (["--target", "1250 1000"], ["--truth", "x y z"],
                 ["--turns", "0"], ["--no-truth", "  "],
                 ["--start-pose", "1 2 3 4"]):
        code, printed, folder = run(tmp_path, com, "--type", "otos_lin", *argv)
        assert code == 2 and "REFUS" in printed, argv
        assert folder is None and com.sent == []


def test_an_unreached_target_is_recorded_and_the_base_is_stopped(tmp_path):
    class Stuck(FakeTeensy):
        def _deliver(self, payload):
            if payload[0] != Messages.ODOMETRY.value:
                super()._deliver(payload)

    com = Stuck()
    code, printed, folder = run(
        tmp_path, com, "--type", "sequence", "--target", "1250 1000 0",
        "--target-timeout", "0.05", "--truth", "1250 1000 0")
    assert "NON atteinte" in printed
    assert "Aucune pose recue" in printed
    assert com.sent[-1] == pack_message(Messages.STOP)
    lines = log_reader.read_lines(folder)
    (target,) = [line for line in lines if line.get("event") == "target"]
    assert target["reached"] is False
    assert code == 0, printed                      # usable: truth and LOG_TICK


def test_interrupting_the_truth_question_still_closes_the_run(tmp_path):
    def interrupted(_prompt):
        raise KeyboardInterrupt

    com = FakeTeensy()
    printed = []
    code = run_test.run_trial(
        parse(tmp_path, "--type", "otos_lin", "--target", "1250 1000 0"),
        com=com, input_fn=interrupted, sleep=lambda _s: None,
        out=printed.append)
    (folder,) = [path for path in tmp_path.iterdir() if path.is_dir()]
    assert code == 1
    assert log_reader.check_run(folder).ok         # closed, readable
    assert trials.read_truth(folder) is None
    assert any("truth.json absent" in line for line in printed)


def test_types_are_listed():
    printed = []
    run_test.print_types(printed.append)
    text = "\n".join(printed)
    for name in trials.TRIAL_TYPES:
        assert name in text
