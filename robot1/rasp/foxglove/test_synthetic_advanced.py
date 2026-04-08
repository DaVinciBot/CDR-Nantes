#!/usr/bin/env python3
"""
Test synthétique FoxgloveBridgeAdvanced — Simule 4 sources capteurs.

Lance des positions tournantes pour vérifier que:
  ✓ Les 4 marqueurs (target, odom, lidar, fused) s'affichent correctement
  ✓ Le robot rouge suit la fusion
  ✓ L'angle vient de l'IMU (pas dérivé de l'odométrie)
"""

import sys
import time
import math
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from foxglove_bridge_advanced import FoxgloveBridgeAdvanced, SensorFusionManager

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)s | %(message)s"
)
logger = logging.getLogger("test_synthetic")

def main():
    logger.info("=== Test Foxglove Advanced (Synthétique) ===")
    logger.info("Lancez Foxglove et connectez à ws://localhost:8765")
    
    # Créer le bridge (pas de COM physique, juste visualization)
    bridge = FoxgloveBridgeAdvanced(port=8765).start()
    
    time.sleep(2)  # Laisser le serveur démarrer
    
    logger.info("Démarrage simulation (Ctrl+C pour arrêter)...")
    
    try:
        t = 0
        while True:
            # ─────────────────────────────────────────────────────
            # Simulation: 4 cercles concentriques + angle stationnaire
            # ─────────────────────────────────────────────────────
            
            angle = (t % 360) * math.pi / 180.0  # Tour complet en 360*dt
            center_x, center_y = 1500, 1000      # Centre du terrain
            
            # 1. TARGET: cercle externe (rayon 800mm)
            r_target = 800
            target_x = center_x + r_target * math.cos(angle)
            target_y = center_y + r_target * math.sin(angle)
            
            # 2. ODOM: cercle (rayon 600mm), légèrement décalé en phase
            r_odom = 600
            odom_x = center_x + r_odom * math.cos(angle + 0.3)
            odom_y = center_y + r_odom * math.sin(angle + 0.3)
            
            # 3. LIDAR: cercle (rayon 500mm), bien détecté
            r_lidar = 500
            lidar_x = center_x + r_lidar * math.cos(angle + 0.1)
            lidar_y = center_y + r_lidar * math.sin(angle + 0.1)
            lidar_confidence = 0.85  # Bien détecté
            
            # 4. IMU ANGLE: rotation lente + oscillation (pour voir la fusion!)
            imu_theta = 0.05 * math.sin(t * 0.01) + 0.1
            
            # Publier
            bridge.publish_data(
                target_x=target_x,
                target_y=target_y,
                odom_x=odom_x,
                odom_y=odom_y,
                lidar_x=lidar_x,
                lidar_y=lidar_y,
                lidar_confidence=lidar_confidence,
                imu_theta=imu_theta
            )
            
            if t % 20 == 0:
                fusion_x, fusion_y, fusion_theta = bridge._fusion.fuse_position()
                logger.info(
                    f"[{t:3d}] "
                    f"Target({target_x:.0f},{target_y:.0f}) | "
                    f"Odom({odom_x:.0f},{odom_y:.0f}) | "
                    f"Lidar({lidar_x:.0f},{lidar_y:.0f},conf={lidar_confidence:.2f}) | "
                    f"Fused({fusion_x:.0f},{fusion_y:.0f}) | "
                    f"θ_imu={imu_theta:.3f}rad"
                )
            
            time.sleep(0.05)  # 20 Hz
            t += 1
    
    except KeyboardInterrupt:
        logger.info("Arrêt utilisateur")
    finally:
        bridge.stop()
        logger.info("Bridge fermé")

if __name__ == "__main__":
    main()
