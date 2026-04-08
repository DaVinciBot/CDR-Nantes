#!/usr/bin/env python3
"""Test Foxglove avec données synthétiques - Pour vérifier visualisation 4 points."""

import sys
import time
import logging
import math
import importlib
from pathlib import Path

# Setup loader
sys.path.insert(0, str(Path(__file__).parent))
from loader import loader

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Charger les classes via loader
FoxgloveBridge = loader.load_class("foxglove", "FoxgloveBridge")

# Charger les fonctions via importlib
utils_module = importlib.import_module("utils.robot_context")
init_robot = utils_module.init_robot

def main():
    """Test simple avec positions synthétiques tournantes."""
    
    # Initialiser COM (hardware ou simulation)
    com, mode = init_robot(logger)
    logger.info(f"Mode: {mode}")
    
    # Créer le bridge Foxglove
    # auto_subscribe=False pour garder le contrôle manuel
    bridge = FoxgloveBridge(com, port=8765, auto_subscribe=False).start()
    
    logger.info("Bridge Foxglove démarré sur ws://localhost:8765")
    logger.info("Lancez Foxglove et connectez-vous à ws://localhost:8765")
    time.sleep(2)
    
    # Boucle de test: positions synthétiques tournantes
    logger.info("Démarrage boucle de visualisation (5 secondes par position)...")
    
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
            lidar_x = odom_x + 20  # +20mm décalage
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
            p_lidar = {"x": lidar_x, "y": lidar_y, "theta": theta + 0.05}  # angle légèrement décalé
            p_fused = {"x": fused_x, "y": fused_y, "theta": theta}
            
            # Publier
            bridge.publish(p_target, p_odom, p_lidar, p_fused)
            
            logger.info(
                f"[{t:3d}°] Odom({odom_x:.0f},{odom_y:.0f}) → "
                f"Lidar({lidar_x:.0f},{lidar_y:.0f}) → "
                f"Fused({fused_x:.0f},{fused_y:.0f}) | "
                f"Target({target_x:.0f},{target_y:.0f})"
            )
            
            time.sleep(0.1)
            t += 1
            
    except KeyboardInterrupt:
        logger.info("\nArrêt demandé par l'utilisateur")
    finally:
        bridge.stop()
        if hasattr(com, "close"):
            com.close()
        logger.info("Bridge fermé")

if __name__ == "__main__":
    main()
