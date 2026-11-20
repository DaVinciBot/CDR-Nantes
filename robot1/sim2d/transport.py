"""The simulated link, in place of the USB link to the Teensy.

comm/context.py returns this class when ROBOT_MODE=sim2d. It satisfies
comm.link.ComLink, so app.py cannot tell it from the real Com: same three
methods, same message identifiers, same payload layouts
(usb_com.PAYLOAD_FORMATS).

Handing over the robot: create_com() builds every transport with the USB
signature and nothing else, so the simulated robot cannot be passed as an
argument. It is deposited here beforehand by run.py, through set_robot(). That
is the price of keeping a single build path for both transports, and the
alternative -- a second construction path in production code -- is worse.
"""

import logging

from usb_com import (
    TAP_RX,
    TAP_TX,
    Messages,
    TraceMask,
    pack_message,
    payload_size,
    unpack_message,
)

from .robot import (
    ODOMETRY_EVERY_TICKS,
    OdometrySample,
    SimulatedRobot,
    TraceSample,
)
from .teensy import CONTROL_PERIOD_S

logger = logging.getLogger("SIM2D_COM")

# ODOMETRY leaves the firmware every second control tick: 30 ms, 33.3 Hz.
# The samples are taken by the robot at those instants (robot.py); this rate is
# only for the log line.
ODOM_RATE_HZ = 1.0 / (ODOMETRY_EVERY_TICKS * CONTROL_PERIOD_S)

_pending_robot = None


def _t_us(sim_time: float) -> int:
    """micros() of the Teensy at a simulated time: a uint32 that wraps."""
    return int(round(sim_time * 1e6)) & 0xFFFFFFFF


def set_robot(robot: SimulatedRobot) -> None:
    """Deposit the robot the next Sim2dCom will drive.

    Args:
        robot: the simulated robot, built by the scenario.
    """
    global _pending_robot
    _pending_robot = robot


class Sim2dCom:
    """Transport backed by a simulated robot instead of a serial port."""

    def __init__(
        self,
        logger_=None,
        serial_number: int = 0,
        vid: int = 0,
        pid: int = 0,
        baudrate: int = 0,
        *,
        enable_crc: bool = True,
        enable_dummy: bool = False,
    ) -> None:
        """Build the simulated link.

        The USB arguments are accepted and ignored on purpose: create_com()
        passes them to every transport. See comm/link.py.

        Raises:
            RuntimeError: when no robot was deposited by set_robot(). Failing
                loudly here beats silently simulating a robot nobody can see.
        """
        if _pending_robot is None:
            raise RuntimeError(
                "Sim2dCom: aucun robot depose. Appeler "
                "robot1.sim2d.transport.set_robot() avant de construire "
                "le Robot."
            )
        self.robot = _pending_robot
        self._callbacks = {}
        self._tap = None
        self.frames_sent = 0
        self.unknown_messages = 0
        logger.info("Lien simule pret (odometrie a %.0f Hz).", ODOM_RATE_HZ)

    # ── ComLink ───────────────────────────────────────────────────────────

    def add_callback(self, func, iid: int) -> None:
        """Register func as the handler for messages of id iid.

        Args:
            func: callable taking the payload bytes.
            iid: message identifier, see usb_com.Messages.
        """
        self._callbacks[iid] = func

    def set_tap(self, tap) -> None:
        """Register a listener on every message, both ways (see comm/link.py).

        Args:
            tap: callable taking a direction (usb_com.TAP_RX or TAP_TX) and
                the message as [id][payload], or None to remove it.
        """
        self._tap = tap

    def send_bytes(self, data: bytes, *, record: bool = True) -> None:
        """Receive one message from the match loop and act on it.

        Args:
            data: [id][payload], exactly what app.py builds. The framing
                (CRC8, COBS, 0x00 delimiter) is added by the real Com
                below this level, so it never appears here.
            record: accepted for signature compatibility, unused.
        """
        if not data:
            return
        if self._tap is not None:
            self._tap(TAP_TX, data)
        message_id = data[0]
        payload = data[1:]

        if message_id == Messages.SET_TARGET.value:
            fields = self._unpack(Messages.SET_TARGET, payload)
            if fields is not None:
                # Pose and velocity of the reference, as the firmware takes
                # them: extrapolated target, feed-forward for 150 ms.
                self.robot.set_target(*fields)
        elif message_id == Messages.SET_ODOMETRIE.value:
            fields = self._unpack(Messages.SET_ODOMETRIE, payload)
            if fields is not None:
                x, y, theta = fields
                self.robot.reset_odometry(x, y, theta)
                logger.info(
                    "Odometrie forcee a (%.0f, %.0f, %.2f).", x, y, theta
                )
        elif message_id == Messages.SET_TRACE.value:
            fields = self._unpack(Messages.SET_TRACE, payload)
            if fields is not None:
                # LOG_TICK is accepted and not sent: the simulator has no
                # sensors to report.
                self.robot.trace_enabled = bool(fields[0] & TraceMask.PID_TRACE)
        else:
            self.unknown_messages += 1
            logger.debug("Message %d ignore par la simulation.", message_id)

    def read_bytes(self) -> bytes:
        """Return nothing: the simulated link pushes through callbacks."""
        return b""

    # ── Simulation side ───────────────────────────────────────────────────

    def publish_odometry(self, sim_time: float, *, force: bool = False) -> int:
        """Deliver the ODOMETRY frames the firmware produced since last call.

        The robot samples its odometry every 30 ms of simulated time, wherever
        the steps of the loop fall: a 50 ms loop tick receives one or two
        frames, like the real link, and the last one wins in app.py.

        The pose is the drifting odometry, as on the real link. The velocity
        is the true one, table frame: the simulator has no velocity estimator.

        The PID_TRACE frames of the same ticks go out too, when SET_TRACE asked
        for them, in the order the firmware sends them: by tick, ODOMETRY
        first.

        Args:
            sim_time: current simulated time, in seconds. Only used by force.
            force: also send the current pose now, used for the first frame,
                before any step.

        Returns:
            int: number of ODOMETRY frames delivered to the callback.
        """
        robot = self.robot
        if force:
            robot.odometry_samples.append(OdometrySample(
                sim_time, *robot.odometry_pose(),
                robot.vx, robot.vy, robot.omega,
            ))
        frames = [(sample.t, 0, self._odometry_message(sample))
                  for sample in robot.odometry_samples]
        frames += [(sample.t, 1, self._trace_message(sample))
                   for sample in robot.trace_samples]
        robot.odometry_samples.clear()
        robot.trace_samples.clear()
        frames.sort(key=lambda frame: frame[:2])

        sent = 0
        for _, _, message in frames:
            if self._tap is not None:
                self._tap(TAP_RX, message)
            callback = self._callbacks.get(message[0])
            if callback is None:
                continue
            # A callback receives the data without the ID byte, as with Com.
            callback(message[1:])
            if message[0] == Messages.ODOMETRY.value:
                sent += 1
        self.frames_sent += sent
        return sent

    @staticmethod
    def _odometry_message(sample: OdometrySample) -> bytes:
        """Pack one ODOMETRY frame."""
        return pack_message(
            Messages.ODOMETRY, sample.x, sample.y, sample.theta,
            sample.vx, sample.vy, sample.omega, _t_us(sample.t),
        )

    @staticmethod
    def _trace_message(sample: TraceSample) -> bytes:
        """Pack one PID_TRACE frame. No I nor D in the 2027 law: zeros."""
        return pack_message(
            Messages.PID_TRACE, _t_us(sample.t), *sample.target, *sample.pose,
            *sample.velocity_cmd, *sample.wheel_requested, *sample.wheel_cmd,
            *sample.wheel_measured, *sample.p_term, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, sample.flags,
        )

    @staticmethod
    def _unpack(message: Messages, payload: bytes):
        """Decode a payload of known layout, or complain and return None."""
        expected = payload_size(message)
        if len(payload) != expected:
            logger.warning(
                "%s : %d octets recus, %d attendus.",
                message.name, len(payload), expected,
            )
            return None
        return unpack_message(message, payload)
