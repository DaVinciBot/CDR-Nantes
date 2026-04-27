#!/usr/bin/env python3
"""Test de déplacement : entre x, y, theta en interactif ou via liste."""

import struct
import time
import logging
import sys
from pathlib import Path
from loader import loader

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

Messages = loader.load_class('usb_com', 'Messages')

sys.path.insert(0, str(Path(__file__).parent))
from utils import init_robot

com, mode = init_robot(logger)


def handle_position(data: bytes) -> None:
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f"  Position reçue : X={x:.1f}mm  Y={y:.1f}mm  θ={theta:.4f}rad ({theta*57.2958:.1f}°)")
    else:
        logger.warning(f"  Message trop court : {len(data)} bytes")


def send_position(x: float, y: float, theta: float, description: str = "") -> None:
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    label = f" [{description}]" if description else ""
    logger.info(f"→ Envoi{label} : X={x}mm  Y={y}mm  θ={theta}rad ({theta*57.2958:.1f}°)")


com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

# ── LISTE DE DÉPLACEMENTS ─────────────────────────────────────────────────────
# Remplis cette liste pour jouer une séquence automatique,
# ou laisse-la vide pour passer en mode interactif.
#
# Format : (x_mm, y_mm, theta_rad, "description", delai_secondes)
#
positions = [
    # (200,   0,    0.0,  "Avance 200mm",      5),
    # (200, 200,    1.57, "Tourne à gauche",   5),
    # (  0,   0,    0.0,  "Retour origine",    5),
]

# ── EXÉCUTION ─────────────────────────────────────────────────────────────────
if positions:
    logger.info(f"Mode séquence : {len(positions)} point(s) à envoyer.")
    for x, y, theta, desc, delai in positions:
        send_position(x, y, theta, desc)
        time.sleep(delai)

else:
    logger.info("Mode interactif — tape 'q' pour quitter.")
    logger.info("Format attendu :  x  y  theta_rad   (ex: 200 0 0  ou  0 200 1.57)")
    print()

    while True:
        try:
            raw = input("Direction > ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if raw.lower() in ("q", "quit", "exit"):
            break
        if not raw:
            continue

        parts = raw.split()
        if len(parts) != 3:
            print("  ✗ Format : x y theta_rad  (3 valeurs séparées par des espaces)")
            continue

        try:
            x, y, theta = float(parts[0]), float(parts[1]), float(parts[2])
        except ValueError:
            print("  ✗ Valeurs non numériques, réessaie.")
            continue

        send_position(x, y, theta)
        time.sleep(0.1)   # laisser le callback afficher la réponse proprement

logger.info("Test terminé. Attente des derniers messages...")
time.sleep(2)