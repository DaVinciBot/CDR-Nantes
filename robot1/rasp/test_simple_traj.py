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

# ============================================================
# DÉTECTION AUTOMATIQUE SIMULATION/HARDWARE - DÉBUT
# ============================================================
# Pour forcer le mode : $env:ROBOT_MODE="simulation" ou "hardware"
# En simulation : lancer depuis simulation/ ou définir ROBOT_MODE
# En hardware : lancer depuis robot1/rasp/ (détection automatique)

sys.path.insert(0, str(Path(__file__).parent))
from robot_context import is_simulation, create_com

def handle_position(data: bytes) -> None:
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f"📍 Position: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")

def send_position(x, y, theta, com, description=""):
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f"📤 {description}: X={x}mm, Y={y}mm, θ={theta}rad")

# Détection et affichage du mode
mode = "SIMULATION" if is_simulation() else "HARDWARE"
logger.info(f"🤖 Mode détecté : {mode}")
logger.info("=" * 70)

# Création automatique de Com selon le contexte
com = create_com(logger=logger)

com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

logger.info("✅ Connexion établie!")
logger.info("=" * 70)

# ============================================================
# DÉTECTION AUTOMATIQUE SIMULATION/HARDWARE - FIN
# ============================================================

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
    time.sleep(10)  # Attendre 10 secondes entre chaque commande

logger.info("=" * 70)
logger.info("✅ Test terminé ! Attente des derniers messages...")
time.sleep(3)
