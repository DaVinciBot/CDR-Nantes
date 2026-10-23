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

from usb_com import Messages, pack_message, payload_size, unpack_message

from .robot import SimulatedRobot

logger = logging.getLogger("SIM2D_COM")

# Rate at which the Teensy reports its odometry on the real robot (~10 Hz).
# Reporting faster here would make the simulated match loop better informed
# than the real one.
ODOM_RATE_HZ = 10.0

_pending_robot = None


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
        self._next_odom_time = None
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
        message_id = data[0]
        payload = data[1:]

        if message_id == Messages.SET_TARGET.value:
            fields = self._unpack(Messages.SET_TARGET, payload)
            if fields is not None:
                # The trajectory velocity (fields 3 to 5) is a feed-forward
                # of the firmware PID. The simulated base approaches its
                # target proportionally instead, so it has no use for it.
                x, y, theta = fields[:3]
                self.robot.set_target(x, y, theta)
        elif message_id == Messages.SET_ODOMETRIE.value:
            fields = self._unpack(Messages.SET_ODOMETRIE, payload)
            if fields is not None:
                x, y, theta = fields
                self.robot.reset_odometry(x, y, theta)
                logger.info(
                    "Odometrie forcee a (%.0f, %.0f, %.2f).", x, y, theta
                )
        else:
            self.unknown_messages += 1
            logger.debug("Message %d ignore par la simulation.", message_id)

    def read_bytes(self) -> bytes:
        """Return nothing: the simulated link pushes through callbacks."""
        return b""

    # ── Simulation side ───────────────────────────────────────────────────

    def publish_odometry(self, sim_time: float, *, force: bool = False) -> bool:
        """Send ODOMETRY if it is due at this simulated time.

        The pose is the drifting odometry, as on the real link. The velocity
        is the true one: the simulator has no velocity estimator, and the
        match loop only reads the pose today.

        Args:
            sim_time: current simulated time, in seconds.
            force: send now whatever the rate, used for the first frame.

        Returns:
            bool: True if a frame was delivered.
        """
        if self._next_odom_time is None:
            self._next_odom_time = sim_time
        if not force and sim_time < self._next_odom_time:
            return False

        self._next_odom_time = sim_time + 1.0 / ODOM_RATE_HZ
        callback = self._callbacks.get(Messages.ODOMETRY.value)
        if callback is None:
            return False

        x, y, theta = self.robot.odometry_pose()
        # t_us is the micros() of the Teensy: a uint32 that wraps.
        t_us = int(sim_time * 1e6) & 0xFFFFFFFF
        message = pack_message(
            Messages.ODOMETRY, x, y, theta,
            self.robot.vx, self.robot.vy, self.robot.omega, t_us,
        )
        # A callback receives the data without the ID byte, as with Com.
        callback(message[1:])
        self.frames_sent += 1
        return True

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
