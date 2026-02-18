#!/usr/bin/env python3
"""Programme de test simplifié - compatible simulation et hardware."""
import struct
import math
import logging
import sys
import time
from pathlib import Path
from loader import loader

# Créer un logger simple
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


def main():
    logger.info("Démarrage du programme de test...")
    
    # Ligne droite horizontale
    send_position(200, 0, 0, com, "Ligne droite horizontale") 
    time.sleep(5)  # Attendre 5 secondes
    # Ligne droite verticale 
    send_position(0, 200, 0, com, "Ligne droite verticale")
    time.sleep(5)  # Attendre 5 secondes
    # Carré
    send_position(200, 0, 0, com, "Carré - Coin 1")
    send_position(200, -200, 0, com, "Carré - Coin 2")
    send_position(0, -200, 0, com, "Carré - Coin 3")
    send_position(0, 0, 0, com, "Carré - Retour origine")
    time.sleep(5)  # Attendre 5 secondes
    # Triangle rectangle
    send_position(200, 0, 0, com, "Triangle - Côté 1")
    send_position(200, 200, 0, com, "Triangle - Côté 2")
    send_position(0, 0, 0, com, "Triangle - Retour origine")
    time.sleep(5)  # Attendre 5 secondes
    # Cercle
    logger.info("Démarrage du cercle...")
    nb_pas = 1000
    rayon = 100

    """for i in range(nb_pas + 1):
        angle = i * 2 * math.pi / nb_pas
        x = rayon * math.cos(angle)
        y = rayon * math.sin(angle)
        theta = angle + math.pi / 2
        send_position(x, y, theta, com, f"Cercle - Point {i}/{nb_pas}")
        time.sleep(0.9)  # Petit délai pour ne pas saturer la communication"""
    send_position(0,0,90,com,"Orientation 90 degrés")
    logger.info("✅ Programme terminé !")

if __name__ == "__main__":
    main()
