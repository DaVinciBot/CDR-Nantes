#!/usr/bin/env python3
"""Test trajectoire simple + visualisation Foxglove intégrée."""

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
from robot1.rasp.Foxglove.foxglove_bridge import FoxgloveBridge

com, mode = init_robot(logger)
logger.info("Mode actif: %s", mode)

# Démarre Foxglove 3D sur le même process COM.
# auto_subscribe=False pour garder ce script maître du callback odométrie.
bridge = FoxgloveBridge(com, port=8765, auto_subscribe=False).start()


def handle_position(data: bytes) -> None:
    """Callback pour recevoir la position du robot."""
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f" Position robot: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")
        bridge.publish(x, y, theta)
    else:
        logger.warning(f" Message trop court: {len(data)} bytes")

def send_position(x, y, theta, com, description=""):
    """Envoie une position cible au robot."""
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f" {description}: X={x}mm, Y={y}mm, θ={theta}rad")


def main() -> None:
    com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

    positions = [
        (200, 0, 0, "Origine"),
        (0, 200, 1.57, "Déplacement diagonal"),
        #(0,200,0, "Retour origine"),
    ]

    time.sleep(2)  # Laisse le bridge Foxglove s'initialiser.
    for x, y, theta, desc in positions:
        send_position(x, y, theta, com, desc)
        time.sleep(120)

    logger.info("=" * 70)
    logger.info(" Test terminé ! Attente des derniers messages...")
    time.sleep(3)


if __name__ == "__main__":
    try:
        main()
    finally:
        bridge.stop()
        if hasattr(com, "close"):
            com.close()
