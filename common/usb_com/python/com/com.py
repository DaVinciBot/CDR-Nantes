"""USB communication helper with optional CRC and dummy mode."""

from __future__ import annotations

import threading
import time
from functools import wraps
from typing import TYPE_CHECKING, Any

from serial import Serial
from serial.tools.list_ports import comports

from usb_com.python.com.dummy import DummySerial
from usb_com.python.com.exceptions import ComError
from usb_com.python.com.framing import decode_frame, encode_frame
from usb_com.python.messages import END_BYTES_SIGNATURE, Messages

NACK_ID = 127

if TYPE_CHECKING:
    from collections.abc import Callable

    from loggerplusplus import Logger


class Com:
    """Handle USB exchanges with a Teensy microcontroller.

    Supports message transmission, CRC8 verification, and callback mechanisms.
    """

    def __init__(
        self,
        logger: Logger,
        serial_number: int,
        vid: int,
        pid: int,
        baudrate: int,
        *,
        enable_crc: bool = True,
        enable_dummy: bool = False,
    ) -> None:
        """Initialize the USB communication instance.

        Args:
            logger (Logger): Logger instance for logging messages.
            serial_number (int): Serial number of the target device.
            vid (int): Vendor ID of the USB device.
            pid (int): Product ID of the USB device.
            baudrate (int): Baud rate for serial communication.
            enable_crc (bool, optional):
                Enables CRC8 checksum verification. Defaults to ``True``.
            enable_dummy (bool, optional):
                Enables dummy mode for testing. Defaults to ``False``.
        """
        # Initialize init variables
        self.logger: Logger = logger
        self.serial_number: int = serial_number
        self.vid: int = vid
        self.pid: int = pid
        self.baudrate: int = baudrate
        self.enable_crc: bool = enable_crc
        self.enable_dummy: bool = enable_dummy

        # Initialize usb com variables
        self._device: Serial | DummySerial = self._get_serial()

        # send_bytes() is called both by the receiver thread (to answer a NACK)
        # and by the caller thread, so every write to the port must be
        # serialized. The CRC state is no longer shared at all: framing.py
        # builds a fresh hasher per call.
        self._tx_lock = threading.Lock()

        self.last_message: bytes | None = None
        self.message_id_callback: dict[int, Callable[[bytes], None]] = {}

        # Start the receiver thread (if not in dummy mode),
        # it is responsible for handling the received data
        self._receiver_thread: threading.Thread | None = self._start_receiver()

    # region ======= Private methods =======
    def _get_serial(self) -> Serial | DummySerial:
        """Detect and initialize the serial device or dummy mode.

        Returns:
            Serial | DummySerial:
                Initialized serial connection or dummy instance.

        Raises:
            ComError: If no device is found and dummy mode is disabled.
        """
        device_found: Serial | DummySerial | None = None

        for port in comports():
            if (
                port.vid == self.vid
                and port.pid == self.pid
                and port.serial_number is not None
                and port.serial_number == str(self.serial_number)
            ):
                device_found = Serial(port.device, baudrate=self.baudrate)
                break

        if device_found is None:
            if self.enable_dummy:
                self.logger.info("Dummy mode")
                device_found = DummySerial()
            else:
                msg = "No Device found!"
                self.logger.critical(msg)
                raise ComError(msg)

        return device_found

    def _start_receiver(self) -> threading.Thread | None:
        """Starts the receiver thread unless in dummy mode.

        Returns:
            threading.Thread | None: Receiver thread or None if dummy mode is enabled.
        """
        # If in dummy mode, do not start the receiver thread
        if self.enable_dummy:
            return None

        receiver = threading.Thread(target=self.__receiver, name="USBComReceiver")
        receiver.start()
        return receiver

    def __receiver(self) -> None:
        """Run in a thread and dispatch messages based on the protocol format.

        Format: ``msg_type | msg_data | msg_length | CRC8 | MSG_END_BYTES``
        with sizes ``1 | msg_length | 1 | 1 | 4`` bytes.
        """
        while True:
            try:
                raw = self.read_bytes()

                # An empty or truncated read happens on disconnection or on a
                # read timeout. Without this guard the code below sliced an
                # empty buffer, always failed the CRC check and NACK-flooded
                # the link in a tight 100 % CPU loop.
                if not raw:
                    time.sleep(0.01)
                    continue

                frame = decode_frame(raw, enable_crc=self.enable_crc)

                if frame is None:
                    self.logger.warning(
                        f"Invalid frame, sending NACK ... [{raw.hex(sep=' ')}]",
                    )
                    # record=False: a NACK must not become the message we
                    # resend the next time the Teensy NACKs us.
                    self.send_bytes(Messages.NACK.to_bytes(), record=False)
                    continue

                message_id, data = frame

                try:
                    if message_id == NACK_ID:
                        self.logger.warning("Received a NACK")
                        if self.last_message is not None:
                            self.send_bytes(self.last_message, record=False)
                            self.logger.info(
                                f"Sending back message : {self.last_message[0]}",
                            )
                            self.last_message = None
                    else:
                        self.message_id_callback.get(
                            message_id,
                            lambda x: self.logger.error(
                                f"Unknown message type ! msg: {x}",
                            ),
                        )(data)

                except Exception as e:  # noqa: BLE001
                    self.logger.error(f"Received message handling crashed :\n{e}")
                    time.sleep(0.5)  # Wait to avoid spamming the logs

            except Exception as e:  # noqa: BLE001
                self.logger.critical(
                    f"Device connection seems to be closed, teensy crashed ? [{e}]",
                )
                time.sleep(0.5)  # Wait to avoid spamming the logs

    # endregion

    # region ======= Public methods =======
    @staticmethod
    def check_dummy(func: Callable[..., Any]) -> Callable[..., Any]:  # UNUSED
        """Decorator to cancel execution when in dummy mode.

        Args:
            func (Callable[..., Any]): Function to wrap.

        Returns:
            Callable[..., Any]:
                Wrapped function that returns ``None`` if dummy mode is enabled.
        """

        @wraps(func)
        def wrapper(  # UNUSED
            self: Com,
            *args: Any,  # noqa: ANN401
            **kwargs: Any,  # noqa: ANN401
        ) -> Any:  # noqa: ANN401
            # Check if the self.enable_dummy attribute is disabled (False)
            if self.enable_dummy:
                self.logger.debug(f"[DUMMY] {func.__name__} was called")
                return None  # Prevents the function from executing

            # Execute the function normally
            return func(self, *args, **kwargs)

        return wrapper

    def send_bytes(self, data: bytes, *, record: bool = True) -> None:
        """Sends bytes over the serial connection.

        Args:
            data (bytes): Data to be transmitted.
            record (bool, optional): Keep this message as the one to resend on
                the next NACK. Pass ``False`` for NACKs and retransmissions,
                otherwise a NACK overwrites the real payload and we end up
                resending a NACK. Defaults to ``True``.
        """
        frame = encode_frame(data, enable_crc=self.enable_crc)

        # The receiver thread also sends (NACKs, retransmissions), so the write
        # must be atomic. The former reset_output_buffer() call was dropped: it
        # discarded bytes the other thread was still sending.
        with self._tx_lock:
            if record:
                self.last_message = data

            self._device.write(frame)
            while self._device.out_waiting:
                pass

    def read_bytes(self) -> bytes:
        """Reads bytes from the serial connection until the END_BYTES_SIGNATURE.

        Returns:
            bytes: Received data.
        """
        return self._device.read_until(END_BYTES_SIGNATURE)

    def add_callback(self, func: Callable[[bytes], None], iid: int) -> None:
        """Registers a callback for a specific message ID.

        Args:
            func (Callable[[bytes], None]): Callback function.
            iid (int): Message ID to associate with the callback.
        """
        if self.message_id_callback.get(iid) is not None:
            self.logger.warning(f"Callback for message id {iid} already exists !")

        self.message_id_callback[iid] = func

    # endregion
