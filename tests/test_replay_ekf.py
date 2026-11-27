"""Tests of the offline EKF: replay, consistency indicators, faults, tuning.

    python -m pytest tests/test_replay_ekf.py

The runs are noisy trials whose truth and sensors follow the very model of
the EKF (log_helpers.noisy_trial): given the generator's constants, the EKF
must come out consistent; given wrong ones, or faulty sensors, the
indicators must say so.

Needs est_replay: `pio run -e replay` in robot1/teensy_moteur, or its path in
EST_REPLAY_BIN. Skipped otherwise.
"""

import json
import math
import sys

import pytest

from robot1.replay import report
from robot1.replay.estimators import ekf, ekf_tune, faults, native
from robot1.replay.load import load_run

from log_helpers import noisy_trial, write_trial

try:
    native.find_binary()
except native.MissingBinary as missing:
    pytest.skip(str(missing), allow_module_level=True)

TICKS = 667                     # 10 s at 15 ms
GENERATOR = dict(accel_noise=100.0, alpha_noise=1.0, optical_sigma_mm_s=10.0)
# The EKF given the generator's constants.
MATCHED = dict(GENERATOR)


def make_run(root, seed, **kwargs):
    inputs, states = noisy_trial(TICKS, seed=seed, **GENERATOR)
    return write_trial(root, inputs, states, name=f"seq{seed}", **kwargs)


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    """Twenty noisy trials, written once for the module."""
    root = tmp_path_factory.mktemp("ekf_runs")
    return [make_run(root, seed) for seed in range(20)]


@pytest.fixture(scope="module")
def matched(runs):
    return [ekf.evaluate_run(load_run(f, verify=False), MATCHED) for f in runs]


def evaluate(folder, values=MATCHED, fault_list=()):
    return ekf.evaluate_run(load_run(folder, verify=False), values,
                            [faults.parse_fault(f) for f in fault_list])


# ── The tool ──────────────────────────────────────────────────────────────

def test_the_tool_reports_every_update_of_a_tick():
    params = native.EkfParams.for_period(160.0, 60.0 * math.pi / 16384.0)
    inputs, _ = noisy_trial(3, seed=0)
    lines = [native.params_line(params), native.reset_line(500.0, 300.0, 0.0)]
    lines += [native.step_line(k, step) for k, step in enumerate(inputs)]
    outputs = native.run_batch(lines, "ekf6")
    # First frame: no prediction nor wheels, but heading then optical.
    assert [u[0] for u in outputs[0].updates] == ["heading", "optical_x",
                                                  "optical_y"]
    assert [u[0] for u in outputs[1].updates] == [
        "wheel_1", "wheel_2", "wheel_3", "heading", "optical_x", "optical_y"]
    for _, innovation, variance, nis, _ in outputs[1].updates:
        assert nis == pytest.approx(innovation ** 2 / variance, rel=1e-4)
    assert outputs[1].pxx > 0.0 and outputs[1].ptt > 0.0


def test_the_tool_refuses_a_short_ekf_parameter_line():
    with pytest.raises(native.ReplayToolError):
        native.run_batch(["P 160 0.01 200", "R 0 0 0"], "ekf6")


def test_expected_nis_of_the_gated_chi_square():
    mean, variance = ekf.truncated_nis_moments(1e4)   # no gate
    assert mean == pytest.approx(1.0)
    assert variance == pytest.approx(2.0)
    assert ekf.NIS_EXPECTED == pytest.approx(0.925, abs=1e-3)


# ── Consistency ───────────────────────────────────────────────────────────

def test_given_the_generator_constants_the_filter_is_consistent(matched):
    for result in matched:
        assert result["anchors"] and result["warm_starts"] == 0
        for name, group in result["nis"].items():
            assert group["verdict"] == "coherent", (result["run"], name, group)
        for source, ch in result["channels"].items():
            assert not ch["biased"], (result["run"], source, ch)
    # Pooled over the runs, the NIS sits on its expected value.
    for name in ekf.GROUPS:
        means = [r["nis"][name]["mean_nis"] for r in matched]
        assert sum(means) / len(means) == pytest.approx(ekf.NIS_EXPECTED,
                                                        abs=0.06)
    # White innovations: one channel in a while may fail by chance.
    coloured = sum(not ch["white"] for r in matched
                   for ch in r["channels"].values())
    assert coloured <= 3


def test_coverage_over_twenty_trials_is_what_the_chi_square_says(matched):
    summary = ekf.summarize(matched)
    assert summary["with_truth"] == 20
    # Binomial spread over 20 trials: about 8 % on xy, 5 % on theta.
    assert summary["coverage_xy"] >= 0.65
    assert summary["coverage_theta"] >= 0.8


def test_too_little_process_noise_reads_overconfident(runs):
    result = evaluate(runs[0], dict(MATCHED, accel_noise=10.0, alpha_noise=0.1))
    verdicts = {g["verdict"] for g in result["nis"].values()}
    assert any(v.startswith("trop confiant") for v in verdicts)


def test_too_much_process_noise_reads_pessimistic(runs):
    result = evaluate(runs[0], dict(MATCHED, accel_noise=1000.0,
                                    alpha_noise=10.0))
    assert result["nis"]["roues"]["verdict"].startswith("trop pessimiste")


# ── Faults ────────────────────────────────────────────────────────────────

def test_an_under_counting_optical_sensor_is_caught_by_the_ratio(runs):
    clean = evaluate(runs[1])["diagnostics"]
    faulty = evaluate(runs[1], fault_list=["optical_scale:0.8"])["diagnostics"]
    assert clean["optical_ratio_windows_out"] == 0
    assert faulty["optical_ratio_windows_out"] >= faulty["optical_ratio_windows"] - 1
    assert faulty["optical_ratio_min"] == pytest.approx(0.8, abs=0.03)


def test_a_lost_optical_sensor_is_rejected_and_flagged(runs):
    clean = evaluate(runs[2])["diagnostics"]
    faulty = evaluate(runs[2], fault_list=["optical_dropout:4,0.5"])
    d = faulty["diagnostics"]
    assert clean["optical_not_ok_ticks"] == 0
    assert d["optical_max_consecutive_rejections"] >= 7
    assert d["optical_not_ok_ticks"] > 0
    assert faulty["faults"] == ["optical_dropout:4,0.5"]


def test_a_drifting_gyro_biases_the_heading_innovations(runs):
    result = evaluate(runs[3], fault_list=["gyro_drift:0.01"])
    assert result["channels"]["heading"]["biased"]
    assert not evaluate(runs[3])["channels"]["heading"]["biased"]


@pytest.mark.parametrize("text", ["optical_scale", "optical_scale:a",
                                  "optical_dropout:1", "spin:2",
                                  "optical_scale:0"])
def test_a_malformed_fault_is_refused(text):
    with pytest.raises(ValueError):
        faults.parse_fault(text)


def test_faults_leave_the_logged_values_float32(runs):
    ticks = load_run(runs[0], verify=False).tick_table()
    out = faults.apply_faults(ticks, [faults.parse_fault("gyro_drift:0.003")])
    heading = out["heading_rad"].to_numpy()
    assert (heading.astype("float32").astype("float64") == heading).all()
    assert out["heading_rad"].iloc[0] == ticks["heading_rad"].iloc[0]


# ── Where the replay starts ───────────────────────────────────────────────

def test_without_pose_set_the_replay_starts_warm_and_says_so(tmp_path):
    folder = make_run(tmp_path, 40, pose_set=False)
    result = evaluate(folder)
    assert result["warm_starts"] == 1 and not result["anchors"]
    assert any("chaud" in w for w in result["warnings"])


def test_a_hole_in_the_ticks_restarts_warm(tmp_path):
    folder = make_run(tmp_path, 41, drop={1300})
    result = evaluate(folder)
    assert result["gaps"] == 1 and result["warm_starts"] == 1
    assert result["diagnostics"]["ticks"] == TICKS - 2


def test_unknown_constant_is_refused():
    with pytest.raises(KeyError):
        ekf.coerce_values({"accel": 1.0})


def test_a_run_without_log_tick_is_unavailable(tmp_path):
    from log_helpers import SyntheticRun, log_params
    run = SyntheticRun(tmp_path)
    run.receive(0.001, log_params())
    result = ekf.evaluate_run(load_run(run.close(), verify=False))
    assert not result["available"]


# ── Command line and report ───────────────────────────────────────────────

def test_command_line_writes_the_result_and_the_report_shows_it(
        runs, monkeypatch, capsys):
    folder = runs[4]
    monkeypatch.setattr(sys, "argv", ["ekf", str(folder), "--set",
                                      "accel_noise=100", "--set",
                                      "alpha_noise=1"])
    assert ekf.main() == 0
    written = json.loads((folder / ekf.RESULT_FILE).read_text("utf-8"))
    assert written["params"]["accel_noise"] == 100.0
    text = "\n".join(report.format_report(
        report.build_report(load_run(folder, verify=False))))
    assert "EKF hors ligne : " in text

    # A fault is a what-if: nothing written over the clean replay.
    (folder / ekf.RESULT_FILE).unlink()
    monkeypatch.setattr(sys, "argv", ["ekf", str(folder), "--fault",
                                      "optical_scale:0.8"])
    assert ekf.main() == 0
    assert not (folder / ekf.RESULT_FILE).exists()
    assert "defauts injectes : optical_scale:0.8" in capsys.readouterr().out


# ── Tuning ────────────────────────────────────────────────────────────────

GRID = {"accel_noise": [50, 100, 200], "alpha_noise": [0.5, 1, 2],
        "optical_sigma_mm_s": [5, 10, 20]}


def test_the_sweep_finds_the_generator_constants(runs):
    result = ekf_tune.tune(runs[5:8], runs[8:10], GRID)
    best = result["best"]["values"]
    assert best["accel_noise"] == GENERATOR["accel_noise"]
    assert best["alpha_noise"] == GENERATOR["alpha_noise"]
    assert best["optical_sigma_mm_s"] == GENERATOR["optical_sigma_mm_s"]
    assert len(result["ranking"]) == 27
    assert all(r["available"] for r in result["validation"])
    assert result["validation_summary"]["with_truth"] == 2


def test_a_run_in_both_sets_is_refused(runs):
    with pytest.raises(ValueError, match="reglage et de validation"):
        ekf_tune.tune(runs[:2], [runs[1]], GRID)


def test_tuning_command_line_writes_constants_ekf_reads(runs, tmp_path,
                                                       monkeypatch):
    grid = tmp_path / "grid.json"
    grid.write_text(json.dumps({"accel_noise": [100, 400]}), encoding="utf-8")
    out = tmp_path / "ekf.json"
    monkeypatch.setattr(sys, "argv", [
        "ekf_tune", "--tune", str(runs[11]), "--validate", str(runs[12]),
        "--grid", str(grid), "--out", str(out)])
    assert ekf_tune.main() == 0
    values = ekf.read_params_file(out)
    assert values["accel_noise"] == 100.0


def test_a_grid_with_an_unknown_constant_is_refused(tmp_path):
    grid = tmp_path / "grid.json"
    grid.write_text(json.dumps({"accel": [1, 2]}), encoding="utf-8")
    with pytest.raises(KeyError):
        ekf_tune.read_grid(grid)
