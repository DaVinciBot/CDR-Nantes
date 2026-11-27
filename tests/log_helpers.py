"""Synthetic runs for the tests of the run log and of the offline tools.

No robot and no serial port: the Teensy is played by messages built with the
protocol itself, pushed through the tap a real link uses, into a real
RunLogger. The folder that comes out is a run folder like any other.
"""

import json
import math
import random
import struct

from usb_com import TAP_RX, TAP_TX, Messages, TraceMask, pack_message
from usb_com.python.messages import (
    LOG_SCHEMA_VERSION,
    PAYLOAD_FIELDS,
    PAYLOAD_FORMATS,
)

from robot1.rasp.telemetry.logger import RunLogger

TICK_US = 15_000


class FakeClock:
    """Time that moves only when told to, in nanoseconds."""

    def __init__(self):
        self.t = 0

    def __call__(self):
        return self.t


class FakeCom:
    """The least a transport is: a tap, and a record of what was sent."""

    def __init__(self):
        self.tap = None
        self.sent = []

    def add_callback(self, func, iid):
        pass

    def read_bytes(self):
        return b""

    def set_tap(self, tap):
        self.tap = tap

    def send_bytes(self, data):
        self.sent.append(bytes(data))
        if self.tap is not None:
            self.tap(TAP_TX, data)

    def receive(self, payload):
        """Deliver one message as if the Teensy had sent it."""
        self.tap(TAP_RX, payload)


# ── Messages, by field name ───────────────────────────────────────────────

def pack_named(message: Messages, **fields) -> bytes:
    """Pack a message from its field names; fields not given are zero.

    An array field takes a sequence of its length.
    """
    values = []
    for name, count in PAYLOAD_FIELDS[message]:
        value = fields.pop(name, None)
        if count == 1:
            values.append(0 if value is None else value)
        else:
            values.extend([0] * count if value is None else list(value))
    if fields:
        raise KeyError(f"unknown fields for {message.name}: {sorted(fields)}")
    # Integer fields must get integers, struct refuses a float.
    codes = [code for code in _codes(PAYLOAD_FORMATS[message])]
    values = [int(value) if code in "bBhHiIqQ" else value
              for code, value in zip(codes, values)]
    return pack_message(message, *values)


def _codes(fmt: str) -> list[str]:
    """One struct code per value of a format ("<3fI" -> f, f, f, I)."""
    codes = []
    count = ""
    for char in fmt.lstrip("<>=!@"):
        if char.isdigit():
            count += char
            continue
        if char == "s":
            codes.append("s")
        else:
            codes.extend(char * int(count or 1))
        count = ""
    return codes


def log_tick(seq: int, tick: int, t_us: int, **fields) -> bytes:
    """A LOG_TICK; fields not given are zero."""
    return pack_named(Messages.LOG_TICK, schema_version=LOG_SCHEMA_VERSION,
                      seq=seq & 0xFFFF, tick=tick, t_us=t_us & 0xFFFFFFFF,
                      **fields)


def log_event(seq: int, tick: int, t_us: int, event: int, **fields) -> bytes:
    """A LOG_EVENT."""
    return pack_named(Messages.LOG_EVENT, schema_version=LOG_SCHEMA_VERSION,
                      seq=seq & 0xFFFF, tick=tick, t_us=t_us & 0xFFFFFFFF,
                      event=event, **fields)


def pid_trace(t_us: int, **fields) -> bytes:
    """A PID_TRACE; fields not given are zero."""
    return pack_named(Messages.PID_TRACE, t_us=t_us & 0xFFFFFFFF, **fields)


def odometry(t_us: int, x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0,
             omega=0.0) -> bytes:
    """An ODOMETRY."""
    return pack_message(Messages.ODOMETRY, x, y, theta, vx, vy, omega,
                        t_us & 0xFFFFFFFF)


def set_target(x, y, theta, vx=0.0, vy=0.0, omega=0.0) -> bytes:
    """A SET_TARGET."""
    return pack_message(Messages.SET_TARGET, x, y, theta, vx, vy, omega)


# ── A whole run ───────────────────────────────────────────────────────────

class SyntheticRun:
    """Messages and lines dated on the clock of the Pi, then written.

    Everything is queued with its time of the Pi and played in that order at
    close(), so a test can add the streams one after the other.
    """

    def __init__(self, root, *, name="test", mode="dummy",
                 trace_mask=TraceMask.PID_TRACE | TraceMask.LOG_TICK,
                 meta=None):
        self.clock = FakeClock()
        self.com = FakeCom()
        self.log = RunLogger(root, name, mode=mode, clock_ns=self.clock,
                             clock_name="fake", lossless=True,
                             trace_mask=trace_mask, meta=meta)
        self.log.attach(self.com)
        self._events = []

    def receive(self, t_rasp_s: float, payload: bytes) -> None:
        """A message of the Teensy, received at t_rasp_s."""
        self._events.append((t_rasp_s, len(self._events), "rx", payload))

    def send(self, t_rasp_s: float, payload: bytes) -> None:
        """A message to the Teensy, sent at t_rasp_s."""
        self._events.append((t_rasp_s, len(self._events), "tx", payload))

    def emit(self, t_rasp_s: float, stream: str, **fields) -> None:
        """A line of the Pi, at t_rasp_s."""
        self._events.append((t_rasp_s, len(self._events), "emit",
                             (stream, fields)))

    def close(self):
        """Play everything in time order, close the log, return the folder."""
        for t_rasp_s, _, kind, item in sorted(self._events,
                                              key=lambda e: e[:2]):
            self.clock.t = int(round(t_rasp_s * 1e9))
            if kind == "rx":
                self.com.receive(item)
            elif kind == "tx":
                self.com.send_bytes(item)
            else:
                self.log.emit(item[0], **item[1])
        self.clock.t += 1
        self.log.close()
        return self.log.folder


# ── Estimator inputs ──────────────────────────────────────────────────────

def as_float32(value: float) -> float:
    """The float32 a value becomes once in a LOG_TICK."""
    return struct.unpack("<f", struct.pack("<f", value))[0]


def omni_inputs(count: int, *, radius_mm: float, counts_to_mm: float,
                seed: int = 0) -> list:
    """Inputs of the estimator for a base that moves in every way it can.

    Translation with rotation, rotation in place, crawl, outlier optical
    samples, missed encoder reads and missing optical samples: every branch
    of the complementary filter is reached. Floats are float32 already, as
    they would come out of a LOG_TICK. The first input is a first frame.

    Returns:
        list[StepInput]
    """
    from robot1.replay.estimators.native import StepInput

    rng = random.Random(seed)
    dt_s = TICK_US * 1e-6
    abs_counts = [0.0, 0.0, 0.0]
    last_counts = [0, 0, 0]
    heading = 0.0
    previous_fresh = True
    inputs = []
    for k in range(count):
        phase = (k // 80) % 4
        if phase == 0:
            vx, vy, omega = 300.0, 100.0, 0.3       # mm/s, mm/s, rad/s
        elif phase == 1:
            vx, vy, omega = 0.0, 0.0, 2.0           # in place
        elif phase == 2:
            vx, vy, omega = 12.0, -6.0, 0.0         # crawl
        else:
            vx, vy, omega = -450.0, 250.0, -0.8
        # Wheel travel over the tick (inverse kinematics, 120 / 240 / 0 deg).
        s3 = math.sqrt(3.0) / 2.0
        travel = (-0.5 * vx + s3 * vy - radius_mm * omega,
                  -0.5 * vx - s3 * vy - radius_mm * omega,
                  vx - radius_mm * omega)
        for i in range(3):
            abs_counts[i] += travel[i] * dt_s / counts_to_mm
        heading = math.atan2(math.sin(heading + omega * dt_s),
                             math.cos(heading + omega * dt_s))

        read_ok = rng.random() > 0.04
        if read_ok:
            now = [int(round(c)) for c in abs_counts]
            d = [now[i] - last_counts[i] for i in range(3)]
            last_counts = now
        else:
            d = [0, 0, 0]
        fresh = read_ok and previous_fresh and k > 0
        previous_fresh = read_ok

        optical_ok = k % 7 != 3
        dx = vx * dt_s + rng.gauss(0.0, 0.05)
        dy = vy * dt_s + rng.gauss(0.0, 0.05)
        if k % 53 == 20:
            dx += 40.0                               # outlier
        inputs.append(StepInput(
            d1=d[0], d2=d[1], d3=d[2], dt_us=TICK_US,
            encoder_age_us=0 if read_ok else TICK_US,
            first_frame=k == 0, encoders_fresh=fresh,
            imu_valid=True, heading_rad=as_float32(heading),
            imu_timestamp_us=k * TICK_US, imu_age_us=1200,
            optical_timestamp_us=k * TICK_US,
            optical_step_valid=optical_ok,
            optical_dx_mm=as_float32(dx), optical_dy_mm=as_float32(dy),
            optical_step_dt_us=TICK_US if optical_ok else 0,
            optical_velocity_valid=optical_ok,
            optical_vx_mm_s=as_float32(dx / dt_s),
            optical_vy_mm_s=as_float32(dy / dt_s),
        ))
    return inputs


def log_params(**fields) -> bytes:
    """A LOG_PARAMS: the constants of the 2027 base, unless given."""
    base = dict(
        schema_version=LOG_SCHEMA_VERSION, estimator=0, period_us=TICK_US,
        robot_radius_mm=160.0, wheel_diameter_mm=[60.0] * 3,
        wheel_angle_rad=[2.0943951, 4.1887902, 0.0], counts_per_rev=16384.0,
        alpha_encoders=0.2, encoder_noise_counts=16.0, imu_max_age_us=50_000,
        optical_outlier_mm_s=1500.0, optical_noise_mm_s=5.0,
        pure_rotation_min_rad=0.005, pure_rotation_min_counts=50.0,
        pure_rotation_tolerance=0.2, use_encoders=1,
        otos_linear_scalar=1.0, otos_angular_scalar=1.0,
        otos_linear_scale_soft=1.0, otos_angular_scale_soft=1.0,
    )
    base.update(fields)
    return pack_named(Messages.LOG_PARAMS, **base)


def log_tick_of(seq: int, tick: int, step_input, output) -> bytes:
    """The LOG_TICK of one tick: its estimator input and output."""
    flags = (int(step_input.first_frame) | int(step_input.encoders_fresh) << 1
             | int(step_input.imu_valid) << 2
             | int(step_input.optical_step_valid) << 3
             | int(step_input.optical_velocity_valid) << 4)
    return log_tick(
        seq, tick, tick * TICK_US,
        d=[step_input.d1, step_input.d2, step_input.d3],
        dt_us=step_input.dt_us, encoder_age_us=step_input.encoder_age_us,
        heading_rad=step_input.heading_rad,
        imu_timestamp_us=step_input.imu_timestamp_us,
        imu_age_us=step_input.imu_age_us,
        optical_timestamp_us=step_input.optical_timestamp_us,
        optical_dx_mm=step_input.optical_dx_mm,
        optical_dy_mm=step_input.optical_dy_mm,
        optical_step_dt_us=step_input.optical_step_dt_us,
        optical_vx_mm_s=step_input.optical_vx_mm_s,
        optical_vy_mm_s=step_input.optical_vy_mm_s,
        input_flags=flags,
        est_pose=[output.x, output.y, output.theta],
        est_vel=[output.vx, output.vy, output.omega],
        est_health=output.health, est_flags=output.flags,
    )


# ── Noisy trials, for the EKF ─────────────────────────────────────────────

def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _white_accel_pair(rng, q: float, dt: float) -> tuple[float, float]:
    """(position, velocity) increments of a white acceleration of density q
    over dt: covariance q^2 [dt^3/3, dt^2/2; dt^2/2, dt], as Ekf6 models it."""
    l11 = math.sqrt(dt ** 3 / 3.0)
    l21 = (dt ** 2 / 2.0) / l11
    l22 = math.sqrt(dt - l21 * l21)
    z1, z2 = rng.gauss(0.0, 1.0), rng.gauss(0.0, 1.0)
    return q * l11 * z1, q * (l21 * z1 + l22 * z2)


def noisy_trial(count: int, *, radius_mm: float = 160.0,
                counts_to_mm: float = 60.0 * math.pi / 16384.0,
                start=(500.0, 300.0, 0.0), start_sigma=(10.0, 0.02),
                seed: int = 0, accel_noise: float = 100.0, alpha_noise: float = 1.0,
                wheel_sigma_counts: float = 10.0,
                wheel_slip_ratio: float = 0.02,
                optical_sigma_mm_s: float = 10.0,
                heading_sigma_rad: float = 1.0e-3,
                lever_mm=(0.0, 0.0)):
    """A trial whose truth and sensors follow the model of the EKF exactly.

    The truth moves as Ekf6 predicts it (constant velocity over a tick, at the
    mid-tick heading) plus white acceleration in the robot frame; each sensor
    measures it with the noise Ekf6 assumes. An EKF given these very constants
    must therefore be consistent: mean NIS near its expected value, white
    innovations, 2-sigma coverage as the chi-square says. It starts at rest,
    at start plus a draw of start_sigma (mm, rad): the uncertainty the EKF
    assumes at a POSE_SET (EkfInitialSigmas), so that the coverage is tested
    from the first tick.

    Returns:
        (inputs, states): the list of StepInput (the first a first frame),
        and the true state (x, y, theta, vx, vy, omega) at each tick, vx and
        vy in the robot frame.
    """
    from robot1.replay.estimators.native import StepInput

    rng = random.Random(seed)
    dt = TICK_US * 1e-6
    x = start[0] + rng.gauss(0.0, start_sigma[0])
    y = start[1] + rng.gauss(0.0, start_sigma[0])
    theta = _wrap(start[2] + rng.gauss(0.0, start_sigma[1]))
    vx = vy = omega = 0.0
    floor_mm_s = wheel_sigma_counts * counts_to_mm / dt
    s3 = math.sqrt(3.0) / 2.0
    inputs, states = [], []
    for k in range(count):
        if k > 0:
            mid = theta + 0.5 * omega * dt
            c, s = math.cos(mid), math.sin(mid)
            ex, nvx = _white_accel_pair(rng, accel_noise, dt)
            ey, nvy = _white_accel_pair(rng, accel_noise, dt)
            et, nw = _white_accel_pair(rng, alpha_noise, dt)
            x += (c * vx - s * vy) * dt + c * ex - s * ey
            y += (s * vx + c * vy) * dt + s * ex + c * ey
            theta = _wrap(theta + omega * dt + et)
            vx += nvx
            vy += nvy
            omega += nw
        states.append((x, y, theta, vx, vy, omega))

        # Wheel speeds (inverse kinematics), noisy, as whole counts.
        speeds = (-0.5 * vx + s3 * vy - radius_mm * omega,
                  -0.5 * vx - s3 * vy - radius_mm * omega,
                  vx - radius_mm * omega)
        counts = []
        for z in speeds:
            sigma = math.sqrt(floor_mm_s ** 2 + (wheel_slip_ratio * z) ** 2)
            counts.append(int(round((z + rng.gauss(0.0, sigma)) * dt
                                    / counts_to_mm)))
        ovx = vx - omega * lever_mm[1] + rng.gauss(0.0, optical_sigma_mm_s)
        ovy = vy + omega * lever_mm[0] + rng.gauss(0.0, optical_sigma_mm_s)
        heading = _wrap(theta + rng.gauss(0.0, heading_sigma_rad))
        stamp = (k + 1) * TICK_US
        inputs.append(StepInput(
            d1=counts[0], d2=counts[1], d3=counts[2], dt_us=TICK_US,
            encoder_age_us=0, first_frame=k == 0, encoders_fresh=k > 0,
            imu_valid=True, heading_rad=as_float32(heading),
            imu_timestamp_us=stamp, imu_age_us=0,
            optical_timestamp_us=stamp,
            optical_step_valid=True,
            optical_dx_mm=as_float32(ovx * dt), optical_dy_mm=as_float32(ovy * dt),
            optical_step_dt_us=TICK_US,
            optical_velocity_valid=True,
            optical_vx_mm_s=as_float32(ovx), optical_vy_mm_s=as_float32(ovy),
        ))
    return inputs, states


def write_trial(root, inputs, states, *, first_tick: int = 1000,
                start=(500.0, 300.0, 0.0), name: str = "essai",
                truth: bool = True, uncertainty=(0.0, 0.0),
                pose_set: bool = True, drop=()):
    """A run folder for a noisy trial, as run_test.py would leave it.

    LOG_PARAMS, a POSE_SET at the start pose before the first tick (unless
    pose_set is False), one LOG_TICK per input (but the ticks in drop) whose
    estimate is the complementary filter of est_replay, and truth.json at
    the true end pose.

    Returns:
        Path: the run folder.
    """
    from robot1.replay.estimators import native

    params = native.ComplementaryParams(robot_radius_mm=160.0,
                                        counts_to_mm=60.0 * math.pi / 16384.0)
    lines = [native.params_line(params), native.reset_line(*start)]
    lines += [native.step_line(first_tick + k, step)
              for k, step in enumerate(inputs)]
    outputs = native.run_batch(lines, "complementary")

    run = SyntheticRun(root, name=name, trace_mask=TraceMask.LOG_TICK,
                       meta={"test_type": "sequence"})
    run.receive(0.0005, log_params())
    if pose_set:
        event_tick = first_tick - 1
        run.receive(event_tick * TICK_US * 1e-6 + 0.004,
                    log_event(0, event_tick, event_tick * TICK_US + 4000, 2,
                              value=list(start), arg=0))
    seq = 0
    for k, (step, out) in enumerate(zip(inputs, outputs)):
        tick = first_tick + k
        if tick in drop:
            continue
        run.receive(tick * TICK_US * 1e-6 + 0.002, log_tick_of(seq, tick, step, out))
        seq += 1
    folder = run.close()
    if truth:
        x, y, theta = states[-1][:3]
        (folder / "truth.json").write_text(json.dumps({
            "kind": "pose", "method": "synthetique",
            "final_pose_table": [x, y, theta],
            "uncertainty_mm": uncertainty[0], "uncertainty_rad": uncertainty[1],
            "note": "",
        }), encoding="utf-8")
    return folder
