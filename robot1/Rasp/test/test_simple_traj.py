#!/usr/bin/env python3
"""Test simplifié avec moins de messages."""

import struct
import time
from loader import loader
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

Com = loader.load_class('usb_com', 'Com')
Messages = loader.load_class('usb_com', 'Messages')

def handle_position(data: bytes) -> None:
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f"📍 Position: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")

def send_position(x, y, theta, com, description=""):
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f"📤 {description}: X={x}mm, Y={y}mm, θ={theta}rad")

serial_config = loader.get_config('serial_config')

com = Com(
    logger=logger,
    serial_number=serial_config['serial_number'],
    vid=serial_config['vid'],
    pid=serial_config['pid'],
    baudrate=serial_config['baudrate'],
    enable_crc=serial_config['enable_crc'],
    enable_dummy=serial_config['enable_dummy']
)

com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

logger.info("✅ Connexion établie avec la Teensy!")
logger.info("=" * 70)

# Test avec quelques points seulement
positions = [
    (0, 0, 0, "Origine"),
    (100, 0, 0, "Avancer 100mm"),
    (100, 100, 0, "Diagonale"),
    (0, 100, 0, "Gauche"),
    (0, 0, 0, "Retour origine"),
]

for x, y, theta, desc in positions:
    send_position(x, y, theta, com, desc)
    time.sleep(2)  # Attendre 2 secondes entre chaque commande

logger.info("=" * 70)
logger.info("✅ Test terminé ! Attente des derniers messages...")
time.sleep(3)
