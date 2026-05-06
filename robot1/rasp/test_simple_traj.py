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
from utils import init_robot

com, mode = init_robot(logger)


def handle_position(data: bytes) -> None:
    """Callback pour recevoir la position du robot."""
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f" Position robot: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")
    else:
        logger.warning(f" Message trop court: {len(data)} bytes")

def send_position(x, y, theta, com, description=""):
    """Envoie une position cible au robot."""
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f" {description}: X={x}mm, Y={y}mm, θ={theta}rad")


def send_reset_odometry(com):
    """Remet l'odométrie à zéro et stoppe le robot proprement."""
    msg = Messages.SET_ODOMETRIE.to_bytes()
    msg += struct.pack('<ddd', 0.0, 0.0, 0.0)
    com.send_bytes(msg)
    logger.info(" Odométrie remise à zéro (0, 0, 0) → robot IDLE")


com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

# Test avec quelques points seulement
positions = [
    (200, 0, 0, "Origine"),
    #(0, 200, 1.57, "Déplacement diagonal"),
    #(0,200,0, "Retour origine"),    
]

try:
    for x, y, theta, desc in positions:
        send_position(x, y, theta, com, desc)
        time.sleep(5)

except KeyboardInterrupt:
    logger.warning("  Interruption clavier détectée")

finally:
    # Toujours exécuté : reset propre même si Ctrl+C ou exception
    logger.info("=" * 70)
    logger.info(" Fin du test — reset odométrie et arrêt robot")
    send_reset_odometry(com)
    time.sleep(0.5)  # Laisser le message partir avant fermeture du port
