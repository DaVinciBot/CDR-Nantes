#!/usr/bin/env python3
"""Test de debug : observation des vitesses calculées pour diagnostiquer le problème."""

import struct
import time
import logging
import sys
from pathlib import Path
from datetime import datetime

sys.path.insert(0, str(Path(__file__).parent))
from loader import loader
from robot_context import is_simulation, create_com

# Configuration du logging pour fichier + console
log_filename = f"debug_wheels_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(message)s',
    handlers=[
        logging.FileHandler(log_filename),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

Messages = loader.load_class('usb_com', 'Messages')

def handle_position(data: bytes) -> None:
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        logger.info(f" Position: X={x:.3f}mm, Y={y:.3f}mm, theta={theta:.4f}rad")

def send_position(x, y, theta, com, description=""):
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f" Commande: {description}")
    logger.info(f"   → X={x}mm, Y={y}mm, θ={theta}rad")

# Détection du mode
mode = "SIMULATION" if is_simulation() else "HARDWARE"
logger.info("=" * 70)
logger.info(f" MODE DEBUG - Diagnostic des équations et du proto")
logger.info(f" Mode: {mode}")
logger.info(f" Log sauvegardé dans: {log_filename}")
logger.info("=" * 70)

# Création de Com
com = create_com(logger=logger)
com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)
logger.info(" Connexion établie!\n")

# ========================================
# TESTS INDIVIDUELS
# ========================================

logger.info(" TESTS À EFFECTUER:")
logger.info("   1. Mouvement X pur (200, 0, 0)")
logger.info("   2. Mouvement Y pur (0, 200, 0)")
logger.info("   3. Rotation pure (0, 0, 1.57)")
logger.info("")
logger.info(" OBSERVEZ DANS LA CONSOLE WEBOTS:")
logger.info("   - Les vitesses des 3 roues (w1, w2, w3)")
logger.info("   - Les composantes (vx_steps, vy_steps, omega_steps)")
logger.info("")
logger.info("=" * 70)

tests = [
    (200, 0, 0, "Mouvement X pur", 
     "Attendu: w1 et w2 participent, w3 devrait être le principal moteur"),
    (0, 200, 0, "Mouvement Y pur", 
     "Attendu: w1 et w2 devraient dominer, w3 proche de 0"),
    (0, 0, 1.57, "Rotation pure (π/2)", 
     "Attendu: w1, w2, w3 tous négatifs (même sens)"),
]

for x, y, theta, desc, expected in tests:
    logger.info(f"\n TEST: {desc}")
    logger.info(f"   {expected}")
    logger.info("-" * 70)
    
    send_position(x, y, theta, com, desc)
    
    # Attendre pendant 8 secondes (comme test_simple_traj)
    logger.info(" Observation pendant 8 secondes...")
    time.sleep(8)
    
    logger.info(" Test terminé.\n")
    time.sleep(1)

logger.info("=" * 70)
logger.info("TOUS LES TESTS TERMINÉS")
logger.info("")
logger.info(" DIAGNOSTIC:")
logger.info("   - Si les vitesses calculées sont correctes MAIS le mouvement est faux")
logger.info("     → Problème dans le PROTO (axes de rotation)")
logger.info("   - Si les vitesses calculées sont incorrectes")
logger.info("     → Problème dans les ÉQUATIONS (matrice cinématique)")
logger.info("=" * 70)
logger.info(f"\nTous les résultats sont sauvegardés dans: {log_filename}")
logger.info("   Vous pouvez partager ce fichier pour analyse.")
