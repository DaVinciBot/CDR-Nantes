#!/usr/bin/env python3
"""
Exemple d'intégration FoxgloveBridgeAdvanced avec 4 sources réelles.

Architecture:
  1. UPDATE_ROLLING_BASIS → odométrie Teensy
  2. LIDAR_DATA → position calculée par balises
  3. IMU_ANGLE → angle θ (source unique fiable)
  4. SET_TARGET_POSITION → position cible du Path Finding
  
Fusion: θ vient TOUJOURS de l'IMU, positions fusionnées intelligemment.
"""

import struct
import sys
import time
import logging
from pathlib import Path

# Setup paths
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from loader import loader
from utils import init_robot
from foxglove_bridge_advanced import FoxgloveBridgeAdvanced

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

Messages = loader.load_class("usb_com", "Messages")


class RobotDataIntegrator:
    """Intègre les 4 sources capteurs → Foxglove."""
    
    def __init__(self, bridge: FoxgloveBridgeAdvanced):
        self.bridge = bridge
        self.target_x = 0.0
        self.target_y = 0.0
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.lidar_x = 0.0
        self.lidar_y = 0.0
        self.lidar_confidence = 0.0
        self.imu_theta = 0.0
    
    def on_rolling_basis(self, data: bytes) -> None:
        """Callback Teensy: odométrie brute."""
        if len(data) < 24:
            return
        x, y, theta_ignored = struct.unpack("<ddd", data[:24])
        self.odom_x = x
        self.odom_y = y
        # ← On ignore theta brut, on utilise IMU
        logger.debug(f"Odom: ({x:.1f}, {y:.1f})")
        self._publish()
    
    def on_lidar_position(self, data: bytes) -> None:
        """Callback Lidar: position calculée + confiance."""
        if len(data) < 32:
            return
        x, y = struct.unpack("<dd", data[:16])
        confidence = struct.unpack("<f", data[16:20])[0] if len(data) >= 20 else 0.0
        self.lidar_x = x
        self.lidar_y = y
        self.lidar_confidence = confidence
        logger.debug(f"Lidar: ({x:.1f}, {y:.1f}) conf={confidence:.2f}")
        self._publish()
    
    def on_imu_angle(self, data: bytes) -> None:
        """Callback IMU: angle θ (source unique)."""
        if len(data) < 8:
            return
        theta, = struct.unpack("<d", data[:8])
        self.imu_theta = theta
        logger.debug(f"IMU θ: {theta:.4f} rad ({theta * 180 / 3.14159:.1f}°)")
        self._publish()
    
    def on_target_position(self, data: bytes) -> None:
        """Callback Path Finding: position cible."""
        if len(data) < 16:
            return
        x, y = struct.unpack("<dd", data[:16])
        self.target_x = x
        self.target_y = y
        logger.debug(f"Target: ({x:.1f}, {y:.1f})")
        self._publish()
    
    def _publish(self) -> None:
        """Publie l'état fusionné vers Foxglove."""
        self.bridge.publish_data(
            target_x=self.target_x,
            target_y=self.target_y,
            odom_x=self.odom_x,
            odom_y=self.odom_y,
            lidar_x=self.lidar_x,
            lidar_y=self.lidar_y,
            lidar_confidence=self.lidar_confidence,
            imu_theta=self.imu_theta
        )


def main():
    # 1. Initialiser COM (hardware ou simulation)
    com, mode = init_robot(logger)
    logger.info(f"Mode: {mode}")
    
    # 2. Créer le bridge Foxglove
    bridge = FoxgloveBridgeAdvanced(port=8765).start()
    
    # 3. Créer l'intégrateur de données
    integrator = RobotDataIntegrator(bridge)
    
    # 4. Enregistrer les callbacks
    try:
        # Odométrie Teensy
        com.add_callback(
            integrator.on_rolling_basis,
            Messages.UPDATE_ROLLING_BASIS.value
        )
        logger.info("✓ Callback odométrie enregistré")
        
        # NOTE: Pour les autres sources (Lidar, IMU, Target),
        # vous devrez ajouter les messages correspondants dans messages.py
        # et les callbacks correspondants ici.
        # Exemple (à adapter selon vos messages):
        #
        # com.add_callback(
        #     integrator.on_lidar_position,
        #     Messages.LIDAR_POSITION.value  # À créer
        # )
        #
        # com.add_callback(
        #     integrator.on_imu_angle,
        #     Messages.IMU_ANGLE.value  # À créer
        # )
        #
        # com.add_callback(
        #     integrator.on_target_position,
        #     Messages.TARGET_POSITION.value  # À créer
        # )
        
    except Exception as exc:
        logger.error(f"Erreur enregistrement callbacks: {exc}")
    
    logger.info("=== Bridge actif ===")
    logger.info("Pour tester avec des données synthétiques:")
    logger.info("  bridge.publish_data(target_x, target_y, odom_x, odom_y, lidar_x, lidar_y, conf, imu_theta)")
    
    # 5. Boucle principale (attend Ctrl+C)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Arrêt...")
    finally:
        bridge.stop()
        if hasattr(com, "close"):
            com.close()
        logger.info("Fermé")


if __name__ == "__main__":
    main()
