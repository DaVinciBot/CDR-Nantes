"""The sensors of the base, as LOG_TICK reports them: encoders, OTOS, IMU.

The simulator has no sensors of its own: they are rebuilt here from the TRUE
motion of the base. Three Python copies of firmware pieces, kept line for line
so a LOG_TICK of sim2d reads like one of the robot:

    holonomic_basis.cpp, build of pose::EstimatorInput
        encoder deltas signed "forward", first_frame then encoders_fresh,
        IMU heading and its age, one NEW optical sample per tick;
    OtosSensor::poll()                  robot1/teensy_moteur/lib/sensors
        raw position, velocity and acceleration in the tracking axes of the
        chip; the first read after a reset only sets the reference of the
        steps; a step is the difference of two tracking positions turned by
        the mount angle, and nothing else;
    Telemetry::fill_log_tick()          robot1/teensy_moteur/lib/telemetry
        health flags of the tick.

The sensors are EXACT: no slip, no optical under-count, no gyro drift. The
estimator output of LOG_TICK is the drifting odometry of robot.py. A replay of
the complementary filter on these inputs therefore finds the true pose, not
est_pose; the gap between the two is the drift the simulator injected.

The OTOS tracking pose is integrated from the displacements of the base, never
read from the true pose: SET_ODOMETRIE moves the true pose without any wheel
turning, and the chip must not see that jump.

The work of the tick is not simulated: work_us and otos_read_us stay at 0.

Constants come from robot1/teensy_moteur/include/config.h. Any change to the
firmware pieces above must be carried over here.
"""

import math

from usb_com.python.messages import LogHealthFlag, LogInputFlag

from .kinematics import MM_S_TO_RPM, forward_kinematics

# config.h: MKS_COUNTS_PER_REV, encoder counts per wheel revolution.
COUNTS_PER_REV = 16384.0

# config.h: OTOS_MOUNT_ANGLE_RAD, rotation from the chip axes to the robot axes.
OTOS_MOUNT_ANGLE_RAD = 0.0

# BNO085 accuracy field: 3 = high. The simulated heading is exact.
IMU_ACCURACY_HIGH = 3

U32 = 0xFFFFFFFF


def t_us(sim_time: float) -> int:
    """micros() of the Teensy at a simulated time: a uint32 that wraps."""
    return int(round(sim_time * 1e6)) & U32


def _wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class SimSensors:
    """Encoders, optical sensor and IMU of the simulated base.

    integrate() follows the motion between ticks; sample() reads everything at
    a control tick and returns the sensor fields of LOG_TICK.
    """

    def __init__(self) -> None:
        """Sensors at boot: encoders at zero, OTOS tracking reset."""
        # Wheel rotation since boot, in counts, signed "forward" (lib/kinematics).
        # Kept as floats; rounded at each read so the deltas add up exactly.
        self._counts = [0.0, 0.0, 0.0]
        self._last_counts = [0, 0, 0]
        self._last_tick_us = 0
        self._started = False

        # OTOS tracking pose, mm and rad, in the axes of the chip at its reset.
        self._otos_x = 0.0
        self._otos_y = 0.0
        self._otos_h = 0.0
        self._otos_last_vel = None
        self._otos_last_t = 0.0
        # OtosSensor::poll(): reference of the steps, taken by the first read.
        self._has_reference = False
        self._ref_x = 0.0
        self._ref_y = 0.0
        self._last_read_us = 0

        mount = OTOS_MOUNT_ANGLE_RAD
        self._mount_cos = math.cos(mount)
        self._mount_sin = math.sin(mount)

    # ── Between ticks ─────────────────────────────────────────────────────

    def integrate(self, wheel_rpm, dt: float) -> None:
        """Follow dt seconds of motion at the held wheel speeds.

        Args:
            wheel_rpm: speeds of wheels 1, 2 and 3, rpm, held over dt.
            dt: duration, s.
        """
        if dt <= 0.0:
            return
        for i, rpm in enumerate(wheel_rpm):
            self._counts[i] += rpm / 60.0 * dt * COUNTS_PER_REV

        # The chip integrates the robot-frame motion in its own axes, at the
        # midpoint of its heading, like robot.py does for the true pose.
        rx, ry, omega = forward_kinematics(
            *(rpm / MM_S_TO_RPM for rpm in wheel_rpm))
        dtheta = omega * dt
        mid = self._otos_h + 0.5 * dtheta
        c, s = math.cos(mid), math.sin(mid)
        self._otos_x += (c * rx - s * ry) * dt
        self._otos_y += (s * rx + c * ry) * dt
        self._otos_h = _wrap_angle(self._otos_h + dtheta)

    # ── At a control tick ─────────────────────────────────────────────────

    def sample(self, sim_time: float, heading_rad: float, wheel_rpm) -> dict:
        """Read every sensor at a control tick.

        Args:
            sim_time: simulated time of the tick, s.
            heading_rad: true heading, table frame, rad (the IMU is exact and
                aligned on the table, as init_holonomic_basis() leaves it).
            wheel_rpm: wheel speeds held over the period that just ended, rpm.

        Returns:
            dict: the encoder, estimator-input, raw OTOS, raw IMU and health
            fields of LOG_TICK, named as in usb_com PAYLOAD_FIELDS.
        """
        now_us = t_us(sim_time)
        fields = {}
        input_flags = 0

        # Encoders: the MKS counts run opposite to the commanded rpm; the
        # firmware negates them to get the "forward" deltas.
        counts = [int(round(c)) for c in self._counts]
        fields["enc_counts"] = [-c for c in counts]
        fields["enc_t_us_mid"] = now_us
        fields["enc_read_ok"] = 1
        if not self._started:
            input_flags |= LogInputFlag.FIRST_FRAME
            self._started = True
            fields["d"] = [0, 0, 0]
            fields["dt_us"] = 0
        else:
            input_flags |= LogInputFlag.ENCODERS_FRESH
            fields["d"] = [c - last for c, last in zip(counts, self._last_counts)]
            fields["dt_us"] = (now_us - self._last_tick_us) & U32
        self._last_counts = counts
        self._last_tick_us = now_us
        fields["encoder_age_us"] = 0

        # IMU: a fresh, exact heading every tick.
        input_flags |= LogInputFlag.IMU_VALID
        fields["heading_rad"] = heading_rad
        fields["imu_timestamp_us"] = now_us
        fields["imu_age_us"] = 0
        fields["imu_raw_yaw_rad"] = heading_rad
        fields["imu_yaw_offset_rad"] = 0.0
        fields["imu_accuracy"] = IMU_ACCURACY_HIGH

        # OTOS: raw values in the tracking axes, then the step of poll().
        rx, ry, omega = forward_kinematics(
            *(rpm / MM_S_TO_RPM for rpm in wheel_rpm))
        c, s = math.cos(self._otos_h), math.sin(self._otos_h)
        vel = (c * rx - s * ry, s * rx + c * ry, omega)
        if self._otos_last_vel is None or sim_time <= self._otos_last_t:
            acc = (0.0, 0.0, 0.0)
        else:
            span = sim_time - self._otos_last_t
            acc = tuple((v - last) / span
                        for v, last in zip(vel, self._otos_last_vel))
        self._otos_last_vel = vel
        self._otos_last_t = sim_time
        fields["otos_pos"] = [self._otos_x, self._otos_y, self._otos_h]
        fields["otos_vel"] = list(vel)
        fields["otos_acc"] = list(acc)
        fields["otos_read_us"] = 0

        step = self._poll(now_us, vel)
        if step is None:
            fields.update(optical_timestamp_us=0, optical_dx_mm=0.0,
                          optical_dy_mm=0.0, optical_step_dt_us=0,
                          optical_vx_mm_s=0.0, optical_vy_mm_s=0.0)
        else:
            input_flags |= (LogInputFlag.OPTICAL_STEP_VALID
                            | LogInputFlag.OPTICAL_VELOCITY_VALID)
            fields.update(step)
        fields["input_flags"] = int(input_flags)

        # Telemetry::fill_log_tick(): no overrun, no failed write.
        health = LogHealthFlag.IMU_READ
        if step is not None:
            health |= LogHealthFlag.OPTICAL_READ
        fields["health_flags"] = int(health)
        fields["rs485_fail"] = [0, 0, 0]
        fields["work_us"] = 0
        return fields

    def skip(self, sim_time: float) -> None:
        """A tick whose LOG_TICK is off: move the references, build nothing.

        The firmware reads its sensors at every tick, logged or not. Keeping
        the references current means a LOG_TICK turned on mid-run carries
        deltas over one tick, and first_frame only on the very first tick.

        Args:
            sim_time: simulated time of the tick, s.
        """
        now_us = t_us(sim_time)
        self._started = True
        self._last_counts = [int(round(c)) for c in self._counts]
        self._last_tick_us = now_us
        self._has_reference = True
        self._ref_x = self._otos_x
        self._ref_y = self._otos_y
        self._last_read_us = now_us
        self._otos_last_vel = None

    def _poll(self, now_us: int, vel: tuple):
        """OtosSensor::poll(): the step since the previous read, or None."""
        if not self._has_reference:
            self._has_reference = True
            self._ref_x = self._otos_x
            self._ref_y = self._otos_y
            self._last_read_us = now_us
            return None
        dx, dy = self._to_robot(self._otos_x - self._ref_x,
                                self._otos_y - self._ref_y)
        vx, vy = self._to_robot(vel[0], vel[1])
        self._ref_x = self._otos_x
        self._ref_y = self._otos_y
        step_dt_us = (now_us - self._last_read_us) & U32
        self._last_read_us = now_us
        return {
            "optical_timestamp_us": now_us,
            "optical_dx_mm": dx,
            "optical_dy_mm": dy,
            "optical_step_dt_us": step_dt_us,
            "optical_vx_mm_s": vx,
            "optical_vy_mm_s": vy,
        }

    def _to_robot(self, sx: float, sy: float) -> tuple:
        """OtosSensor::to_robot(): chip axes to robot axes, mount angle only."""
        return (self._mount_cos * sx - self._mount_sin * sy,
                self._mount_sin * sx + self._mount_cos * sy)
