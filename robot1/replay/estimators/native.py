"""Drive est_replay, the estimators of the board built for the PC.

Two ways to run it: EstimatorProcess, in lock step (each step waits for its
answer; needed to try a reset at two places, as the complementary replay
does), and run_batch(), every line at once (the EKF replay and the tuning
sweep, which never go back).

The tool is built by PlatformIO, from robot1/teensy_moteur:

    pio run -e replay        ->  .pio/build/replay/program(.exe)

EST_REPLAY_BIN points to another build when set. The protocol (one command
per line on stdin, one answer per step on stdout) is described at the top of
tools/est_replay/main.cpp. Floats are sent in hexadecimal (float.hex()), so a
logged float reaches the C++ without any rounding.
"""

from __future__ import annotations

import os
import struct
import subprocess
from dataclasses import dataclass, fields
from pathlib import Path

BIN_ENV = "EST_REPLAY_BIN"
_PROGRAM = "program.exe" if os.name == "nt" else "program"
DEFAULT_BIN = (Path(__file__).resolve().parents[2] / "teensy_moteur" / ".pio"
               / "build" / "replay" / _PROGRAM)

BUILD_HINT = ("outil de rejeu absent : le compiler depuis robot1/teensy_moteur "
              "avec `pio run -e replay` (ou donner son chemin dans "
              f"{BIN_ENV})")


class MissingBinary(RuntimeError):
    """est_replay is not built."""


class ReplayToolError(RuntimeError):
    """est_replay refused a line or stopped."""


@dataclass
class ComplementaryParams:
    """Constants of the complementary filter, as pose::ComplementaryParams."""

    robot_radius_mm: float
    counts_to_mm: float
    encoder_noise_counts: float = 16.0
    imu_max_age_us: int = 50_000
    optical_outlier_mm_s: float = 1500.0
    optical_noise_mm_s: float = 5.0
    pure_rotation_min_rad: float = 0.005
    pure_rotation_min_counts: float = 50.0
    pure_rotation_tolerance: float = 0.2
    alpha_encoders: float = 0.20
    use_encoders: bool = True

    def to_dict(self) -> dict:
        return {f.name: getattr(self, f.name) for f in fields(self)}


@dataclass
class EkfParams:
    """Constants of the 6-state EKF: geometry, kalman::Ekf6Params and
    pose::EkfInitialSigmas, in the order of the est_replay P line.

    Defaults: provisional_ekf6_params() of lib/kalman for a 15 ms loop
    (copied here: a change there is to be reported here). Not tuned values:
    tuning them is what this replay is for.
    """

    robot_radius_mm: float
    counts_to_mm: float
    accel_noise: float = 200.0
    alpha_noise: float = 2.0
    wheel_sigma_counts: float = 10.0
    wheel_slip_ratio: float = 0.02
    optical_sigma_mm_s: float = 10.0
    heading_sigma_rad: float = 1.0e-3
    optical_lever_x_mm: float = 0.0
    optical_lever_y_mm: float = 0.0
    max_dt_s: float = 0.05
    max_sigma_theta_rad: float = 0.2
    max_sigma_v_mm_s: float = 200.0
    max_sigma_omega_rad_s: float = 1.0
    optical_max_consecutive_rejections: int = 7
    optical_ratio_min: float = 0.93
    optical_ratio_max: float = 1.07
    optical_ratio_min_speed_mm_s: float = 50.0
    optical_ratio_window: int = 67
    sigma_xy_mm: float = 10.0
    sigma_theta_rad: float = 0.02
    sigma_v_mm_s: float = 1.0
    sigma_omega_rad_s: float = 0.01

    def to_dict(self) -> dict:
        return {f.name: getattr(self, f.name) for f in fields(self)}

    @classmethod
    def for_period(cls, robot_radius_mm: float, counts_to_mm: float,
                   period_s: float = 0.015, **values) -> "EkfParams":
        """Defaults for a loop of period_s, as provisional_ekf6_params()
        sizes the optical diagnostics (100 ms of rejections, 1 s window)."""

        def samples_in(duration_s):
            if not period_s > 0.0:
                return 1
            n = duration_s / period_s + 0.5
            return 1 if n < 1.0 else int(n)

        base = dict(optical_max_consecutive_rejections=samples_in(0.1),
                    optical_ratio_window=samples_in(1.0))
        base.update(values)
        return cls(robot_radius_mm=robot_radius_mm, counts_to_mm=counts_to_mm,
                   **base)


@dataclass
class StepInput:
    """One pose::EstimatorInput."""

    d1: int = 0
    d2: int = 0
    d3: int = 0
    dt_us: int = 0
    encoder_age_us: int = 0
    first_frame: bool = False
    encoders_fresh: bool = False
    imu_valid: bool = False
    heading_rad: float = 0.0
    imu_timestamp_us: int = 0
    imu_age_us: int = 0
    optical_timestamp_us: int = 0
    optical_step_valid: bool = False
    optical_dx_mm: float = 0.0
    optical_dy_mm: float = 0.0
    optical_step_dt_us: int = 0
    optical_velocity_valid: bool = False
    optical_vx_mm_s: float = 0.0
    optical_vy_mm_s: float = 0.0


@dataclass
class StepOutput:
    """What the estimator gives back after one step."""

    tick: int
    x: float
    y: float
    theta: float
    vx: float
    vy: float
    omega: float
    health: int
    flags: int


# kalman::UpdateSource
UPDATE_SOURCES = ("wheel_1", "wheel_2", "wheel_3", "optical_x", "optical_y",
                  "heading")


@dataclass
class EkfStepOutput(StepOutput):
    """An EKF step: the estimate, its covariance, diagnostics and updates."""

    pxx: float = 0.0
    pyy: float = 0.0
    ptt: float = 0.0
    pxy: float = 0.0
    pxt: float = 0.0
    pyt: float = 0.0
    covariance_healthy: bool = True
    optical_ok: bool = True
    optical_consecutive_rejections: int = 0
    optical_ratio: float = 0.0
    optical_ratio_windows: int = 0
    optical_ratio_windows_out: int = 0
    # (source name, innovation, variance, nis, accepted), in update order
    updates: list = None


def parse_step(answer: str):
    """One S answer of est_replay: StepOutput, or EkfStepOutput if longer.

    Raises:
        ReplayToolError: the tool reported an error or the line is malformed.
    """
    parts = answer.split()
    if not parts or parts[0] == "E" or len(parts) < 9:
        raise ReplayToolError(f"est_replay : {answer.strip() or 'sans reponse'}")
    base = (int(parts[0]), *(_float32(part) for part in parts[1:7]),
            int(parts[7]), int(parts[8]))
    if len(parts) == 9:
        return StepOutput(*base)
    try:
        count = int(parts[21])
        updates = []
        for i in range(count):
            u = parts[22 + 5 * i: 27 + 5 * i]
            updates.append((UPDATE_SOURCES[int(u[0])], _float32(u[1]),
                            _float32(u[2]), _float32(u[3]), u[4] == "1"))
        if len(parts) != 22 + 5 * count:
            raise ValueError("length")
        return EkfStepOutput(
            *base, *(_float32(v) for v in parts[9:15]),
            parts[15] == "1", parts[16] == "1", int(parts[17]),
            _float32(parts[18]), int(parts[19]), int(parts[20]), updates)
    except (ValueError, IndexError) as error:
        raise ReplayToolError(f"est_replay : reponse illisible : "
                              f"{answer.strip()[:120]}") from error


def params_line(params) -> str:
    """The P line of a ComplementaryParams or an EkfParams."""
    return " ".join(["P", *(_number(v) for v in params.to_dict().values())])


def reset_line(x_mm: float, y_mm: float, theta_rad: float) -> str:
    return " ".join(["R", _number(float(x_mm)), _number(float(y_mm)),
                     _number(float(theta_rad))])


def step_line(tick: int, step_input: StepInput) -> str:
    values = [_number(getattr(step_input, f.name)) for f in fields(StepInput)]
    return " ".join(["S", str(int(tick)), *values])


def run_batch(lines: list[str], estimator: str = "complementary",
              binary=None) -> list:
    """Run est_replay on every line at once; one parsed output per S line.

    Raises:
        MissingBinary: est_replay is not built.
        ReplayToolError: the tool refused a line.
    """
    tool = find_binary(binary)
    done = subprocess.run([str(tool), "--estimator", estimator],
                          input="\n".join(lines) + "\n", capture_output=True,
                          text=True, check=False)
    answers = done.stdout.splitlines()
    if done.returncode != 0:
        last = answers[-1] if answers else "sans message"
        raise ReplayToolError(f"est_replay arrete ({done.returncode}) : {last}")
    return [parse_step(answer) for answer in answers]


def find_binary(path=None) -> Path:
    """The est_replay executable: path, else EST_REPLAY_BIN, else the build.

    Raises:
        MissingBinary: nothing there.
    """
    candidate = Path(path or os.environ.get(BIN_ENV) or DEFAULT_BIN)
    if not candidate.is_file():
        raise MissingBinary(f"{BUILD_HINT} ; cherche : {candidate}")
    return candidate


def _float32(text: str) -> float:
    """A float printed with 9 digits, back to the exact float32 it came from."""
    return struct.unpack("<f", struct.pack("<f", float(text)))[0]


def _number(value) -> str:
    """A token: integers as they are, floats in exact hexadecimal."""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return str(value)
    return float(value).hex()


class EstimatorProcess:
    """One running est_replay, in lock step: a step waits for its answer.

        with EstimatorProcess() as est:
            est.params(params)
            est.reset(x, y, theta)
            out = est.step(tick, step_input)
    """

    def __init__(self, binary=None, estimator: str = "complementary"):
        self.binary = find_binary(binary)
        self._process = subprocess.Popen(
            [str(self.binary), "--estimator", estimator],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
            bufsize=1,
        )

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def close(self) -> None:
        """End the tool; it must leave cleanly."""
        process = self._process
        if process.poll() is None:
            process.stdin.close()
            process.wait(timeout=10)
        process.stdout.close()

    def _send(self, *tokens) -> None:
        line = " ".join(str(token) for token in tokens)
        try:
            self._process.stdin.write(line + "\n")
            self._process.stdin.flush()
        except OSError as error:
            raise ReplayToolError(f"est_replay arrete : {self._tail()}") from error

    def _tail(self) -> str:
        """What the tool said before stopping."""
        try:
            return self._process.stdout.read().strip() or "sans message"
        except (OSError, ValueError):
            return "sans message"

    def params(self, params) -> None:
        """A new estimator with these constants (state and copy forgotten)."""
        self._send(params_line(params))

    def reset(self, x_mm: float, y_mm: float, theta_rad: float) -> None:
        """reset(), as SET_ODOMETRIE or an estimator switch on the board."""
        self._send("R", _number(float(x_mm)), _number(float(y_mm)),
                   _number(float(theta_rad)))

    def keep(self) -> None:
        """Keep a copy of the state (one copy, the last one)."""
        self._send("K")

    def back(self) -> None:
        """Back to the kept copy."""
        self._send("B")

    def step(self, tick: int, step_input: StepInput) -> StepOutput:
        """One step(); returns the estimate after it."""
        self._send(step_line(tick, step_input))
        answer = self._process.stdout.readline()
        parts = answer.split()
        if len(parts) < 9 or parts[0] == "E":
            raise ReplayToolError(
                f"est_replay : {answer.strip() or 'sans reponse'} "
                f"{self._tail()}".strip())
        return parse_step(answer)
