"""Utilities to list serial numbers of connected devices."""

from __future__ import annotations

import serial.tools.list_ports
from loggerplusplus import LoggerPlusPlus as Logger

logger = Logger()


def get_all_serial_number() -> None:
    """Print the serial numbers of all detected serial ports."""
    ports = serial.tools.list_ports.comports()
    logger.info(f"Number of ports: {len(ports)}")
    for port in ports:
        logger.info(f"Serial Number: {port.serial_number}")


get_all_serial_number()
