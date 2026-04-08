#!/usr/bin/env python3
"""Test Foxglove avec données synthétiques - Pour vérifier visualisation 4 points."""

import struct
import time
import logging
import math
import sys
from pathlib import Path
from loader import loader

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

Messages = loader.load_class('usb_com', 'Messages')

# Initialisation automatique (simulation ou hardware)
sys.path.insert(0, str(Path(__file__).parent))
from utils import init_robot
from foxglove.foxglove_bridge import FoxgloveBridge

com, mode = init_robot(logger)
logger.info("Mode actif: %s", mode)

# Démarre Foxglove 3D sur le même process COM.
bridge = FoxgloveBridge(com, port=8765, auto_subscribe=False).start()

logger.info("Bridge Foxglove démarré sur ws://localhost:8765")
logger.info("Lancez Foxglove et connectez-vous")
time.sleep(2)

# Boucle de test: positions synthétiques tournantes
logger.info("Démarrage boucle de visualisation...")

try:
    t = 0
    while True:
        # Génération synthétique: cercle tournant
        angle = (t % 360) * math.pi / 180.0
        radius = 500  # 500mm
        
        # Position odométrie: cercle
        odom_x = 1500 + radius * math.cos(angle)
        odom_y = 1000 + radius * math.sin(angle)
        
        # Position Lidar: décalée légèrement
        lidar_x = odom_x + 20
        lidar_y = odom_y - 10
        
        # Position fusionnée: moyenne simple
        fused_x = (odom_x + lidar_x) / 2
        fused_y = (odom_y + lidar_y) / 2
        theta = angle
        
        # Position cible: coin opposé
        target_x = 3000 - radius * math.cos(angle)
        target_y = 2000 - radius * math.sin(angle)
        
        # Construire les 4 positions
        p_target = {"x": target_x, "y": target_y, "theta": theta}
        p_odom = {"x": odom_x, "y": odom_y, "theta": theta}
        p_lidar = {"x": lidar_x, "y": lidar_y, "theta": theta + 0.05}
        p_fused = {"x": fused_x, "y": fused_y, "theta": theta}
        
        # Publier
        bridge.publish(p_target, p_odom, p_lidar, p_fused)
        
        logger.info(
            f"[{t:3d}°] Odom({odom_x:.0f},{odom_y:.0f}) → "
            f"Lidar({lidar_x:.0f},{lidar_y:.0f}) → "
            f"Fused({fused_x:.0f},{fused_y:.0f})"
        )
        
        time.sleep(0.1)
        t += 1
        
except KeyboardInterrupt:
    logger.info("\nArrêt demandé")
finally:
    bridge.stop()
    if hasattr(com, "close"):
        com.close()
    logger.info("Bridge fermé")
