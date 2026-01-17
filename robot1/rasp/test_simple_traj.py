#!/usr/bin/env python3
"""Test simplifié avec moins de messages - compatible simulation et hardware."""

import struct
import time
import logging
import sys
from pathlib import Path
from loader import loader

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

Messages = loader.load_class('usb_com', 'Messages')

#A mettre partout dans les codes python
# Initialisation automatique (simulation ou hardware)
sys.path.insert(0, str(Path(__file__).parent))
from robot_context import init_robot

com, mode = init_robot(logger)


def handle_position(data: bytes) -> None:
    """Callback pour recevoir la position du robot."""
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f"📍 Position robot: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")
    else:
        logger.warning(f" Message trop court: {len(data)} bytes")

def send_position(x, y, theta, com, description=""):
    """Envoie une position cible au robot."""
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f"📤 {description}: X={x}mm, Y={y}mm, θ={theta}rad")

com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

# Test avec quelques points seulement
positions = [
    #(0, 0, 0, "Origine"),
    #(100,0,0, "Avancer 200mm"),
    (200,0,0, "Diagonale"),
    (200, 100, 0, "Gauche"),
    #(0, 0, 0, "Retour origine"),
]

for x, y, theta, desc in positions:
    send_position(x, y, theta, com, desc)
    time.sleep(10)  # Attendre 10 secondes entre chaque commande

logger.info("=" * 70)
logger.info("✅ Test terminé ! Attente des derniers messages...")
time.sleep(3)
