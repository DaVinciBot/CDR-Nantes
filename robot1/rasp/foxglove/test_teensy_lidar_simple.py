#!/usr/bin/env python3
"""
Test simple Teensy + Lidar → Foxglove (SANS fusion, juste affichage brut).

Objectif: Vérifier que vous recevez les données Teensy ET Lidar et les affichez dans Foxglove.
Lidar: Utilise le PoseEngine (trilatération via 3 balises depuis robot1/rasp/lidar/lidar_logic.py)
État: Affichage basique, sans logique de fusion. Juste récréation directe des 2 sources.

Affichage dans Foxglove:
  🔴 ROBOT ROUGE: Position Teensy brute (x_odom, y_odom)
  🟢 TARGET VERT: Position Lidar calculée (x_pose, y_pose) - trilatération balises
  🟡 LIDAR JAUNE: Pas utilisé
  🔵 ODOM BLEU: Pas utilisé
"""

import struct
import sys
import time
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

from loader import loader
from utils import init_robot
from foxglove_bridge_advanced import FoxgloveBridgeAdvanced

# Import Lidar functions
try:
    from lidar.lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime
except ImportError:
    from lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)s | %(message)s"
)
logger = logging.getLogger("test_teensy_lidar_simple")

Messages = loader.load_class("usb_com", "Messages")


class SimpleDataLogger:
    """Log simple des données Teensy + Lidar (via Lidar pose engine)."""
    
    def __init__(self, bridge):
        self.bridge = bridge
        self.odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.lidar_pose = {"x": 0.0, "y": 0.0, "theta": 0.0, "confidence": 0.0}
        self.count = 0
        self.lidar_age = 0.0
    
    def on_teensy_odom(self, data: bytes) -> None:
        """Callback Teensy odométrie."""
        if len(data) < 24:
            logger.warning(f"❌ Payload Teensy trop court: {len(data)} bytes")
            return
        
        x, y, theta = struct.unpack("<ddd", data[:24])
        self.odom = {"x": x, "y": y, "theta": theta}
        
        self.count += 1
        if self.count % 20 == 0:
            logger.info(f"📍 Teensy: X={x:.1f}mm, Y={y:.1f}mm, θ={theta:.3f}rad")
        
        # Afficher dans Foxglove (SANS fusion, juste brut)
        self._publish()
    
    def poll_lidar(self) -> None:
        """Interroge la pose Lidar calculée par le PoseEngine."""
        try:
            pose = get_latest_pose()
            if pose:
                self.lidar_pose = {
                    "x": pose.x,
                    "y": pose.y,
                    "theta": pose.theta,
                    "confidence": pose.confidence,
                    "is_localized": pose.is_localized
                }
                if self.count % 20 == 0:
                    logger.info(
                        f"🎯 Lidar: X={pose.x:.1f}mm, Y={pose.y:.1f}mm, "
                        f"θ={pose.theta:.3f}rad, conf={pose.confidence:.2f}, "
                        f"localized={pose.is_localized}"
                    )
        except Exception as exc:
            logger.warning(f"❌ Erreur Lidar: {exc}")
    
    def _publish(self) -> None:
        """Envoie les données à Foxglove (SANS fusion)."""
        # Récupérer la dernière pose Lidar
        self.poll_lidar()
        
        self.bridge.publish_data(
            # Position Teensy brute → affiche en ROUGE
            odom_x=self.odom["x"],
            odom_y=self.odom["y"],
            
            # Position Lidar (calculée) → affiche en VERT (comme target)
            target_x=self.lidar_pose["x"],
            target_y=self.lidar_pose["y"],
            
            # Les autres ne sont pas utilisés
            lidar_x=0.0,
            lidar_y=0.0,
            lidar_confidence=self.lidar_pose["confidence"],
            imu_theta=self.odom["theta"]  # Angle du Teensy (temporaire, pas IMU encore)
        )


def main():
    logger.info("=" * 70)
    logger.info("TEST SIMPLE: Teensy + Lidar → Foxglove (SANS fusion)")
    logger.info("=" * 70)
    
    # 1. Initialiser COM (hardware ou simulation)
    com, mode = init_robot(logger)
    logger.info(f"✓ Mode: {mode}")
    
    # 2. Créer le bridge Foxglove
    bridge = FoxgloveBridgeAdvanced(port=8765).start()
    time.sleep(1)
    
    # 3. Créer le logger
    logger_data = SimpleDataLogger(bridge)
    
    # 4. Démarrer le thread d'acquisition Lidar
    def lidar_console_cb(msg: str):
        logger.info(f"[LIDAR] {msg}")
    
    def lidar_status_cb(status: str):
        logger.info(f"[LIDAR STATUS] {status}")
    
    try:
        lidar_thread = start_lidar_thread(lidar_console_cb, lidar_status_cb)
        logger.info("✓ Thread Lidar démarré")
        time.sleep(2)  # Laisser le Lidar initialiser
    except Exception as exc:
        logger.error(f"❌ Erreur démarrage Lidar: {exc}")
        return
    
    # 5. Enregistrer les callbacks Teensy
    try:
        com.add_callback(
            logger_data.on_teensy_odom,
            Messages.UPDATE_ROLLING_BASIS.value
        )
        logger.info("✓ Callback Teensy (UPDATE_ROLLING_BASIS) enregistré")
    except Exception as exc:
        logger.error(f"❌ Erreur Teensy: {exc}")
        return
    
    logger.info("")
    logger.info("📡 Écoute Teensy + Lidar...")
    logger.info("")
    logger.info("Lancez Foxglove: ws://localhost:8765")
    logger.info("Vous devriez voir:")
    logger.info("  🔴 ROBOT (ROUGE) = Teensy odométrie brute")
    logger.info("  🟢 TARGET (VERT) = Lidar position calculée (trilatération)")
    logger.info("")
    
    # 6. Boucle principale
    try:
        while True:
            time.sleep(0.5)  # Polling Lidar 2x/sec
    except KeyboardInterrupt:
        logger.info("\n✓ Arrêt utilisateur")
    finally:
        stop_lidar_runtime()
        bridge.stop()
        if hasattr(com, "close"):
            com.close()
        logger.info("✓ Fermé")


if __name__ == "__main__":
    main()
