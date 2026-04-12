#!/usr/bin/env python3
"""
Test simple Teensy + Lidar → Rerun (SANS fusion, juste affichage brut).

Objectif: Vérifier que vous recevez les données Teensy ET Lidar et les affichez dans Rerun.
Lidar: Utilise le PoseEngine (trilatération via 3 balises depuis robot1/rasp/lidar/lidar_logic.py)
État: Affichage basique, sans logique de fusion. Juste récréation directe des 2 sources.

Affichage dans Rerun:
  🔴 ROBOT ROUGE: Position Teensy brute (x_odom, y_odom) → world/robot/odom
  🟢 TARGET VERT: Position Lidar calculée (x_pose, y_pose) → world/lidar/pose (trilatération)
"""

import struct
import sys
import time
import logging
import math
from pathlib import Path

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

# Configurer les chemins AVANT les imports
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))

# Maintenant importer loader
from loader import loader
from utils import init_robot

# Import Lidar functions
try:
    from lidar.lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime
except ImportError:
    sys.path.insert(0, str(Path(__file__).parent.parent / "lidar"))
    from lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)s | %(message)s"
)
logger = logging.getLogger("test_teensy_lidar_simple")

Messages = loader.load_class("usb_com", "Messages")

# ─────────────────────────────────────────────────────────────────────────────
# Constantes — repère terrain (coin bas-gauche = origine)
# ─────────────────────────────────────────────────────────────────────────────

W  = 3000.0
H  = 2000.0
CX = W / 2

# Couleurs RGBA uint8
C_ROBOT   = [51, 153, 255, 220]      # Bleu — Teensy odom
C_LIDAR_P = [255, 80, 80, 255]       # Rouge — Lidar pose
C_TARGET  = [80, 255, 120, 255]      # Vert — Lidar target (inutilisé ici)


# ─────────────────────────────────────────────────────────────────────────────
# Utilitaires Rerun
# ─────────────────────────────────────────────────────────────────────────────

def _log_robot(entity_path: str, x: float, y: float, theta: float, color: list) -> None:
    """Publie un robot (cylindre + flèche de direction) à la position donnée."""
    ct, st = math.cos(theta), math.sin(theta)
    
    # Cylindre robot (rayon 150mm, hauteur 100mm)
    rr.log(f"{entity_path}/body", rr.Cylinders3D(
        centers=[[x, y, 50]],
        radii=[150],
        half_lengths=[50],
        rotations=[rr.Quaternion(xyzw=[0, 0, math.sin(theta/2), math.cos(theta/2)])],
        colors=[color],
    ))
    
    # Flèche de direction (200mm vers l'avant)
    arrow_tip = [x + 200*ct, y + 200*st, 50]
    rr.log(f"{entity_path}/arrow", rr.LineStrips3D(
        strips=[[[x, y, 50], arrow_tip]],
        colors=[color], radii=[8.0],
    ))


class SimpleDataLogger:
    """Log simple des données Teensy + Lidar (via Lidar pose engine)."""
    
    def __init__(self):
        self.odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.lidar_pose = {"x": 0.0, "y": 0.0, "theta": 0.0, "confidence": 0.0}
        self.count = 0
    
    def on_teensy_odom(self, data: bytes) -> None:
        """Callback Teensy odométrie."""
        if len(data) < 24:
            logger.warning(f"❌ Payload Teensy trop court: {len(data)} bytes")
            return
        
        x, y, theta = struct.unpack("<ddd", data[:24])
        self.odom = {"x": x, "y": y, "theta": theta}
        
        self.count += 1
        
        # Afficher dans Rerun
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
            pass  # Silencieusement ignorer si pas de pose encore
    
    def _publish(self) -> None:
        """Envoie les données à Rerun."""
        # Position Teensy brute
        _log_robot("world/robot/odom", 
                   self.odom["x"], self.odom["y"], self.odom["theta"], C_ROBOT)
        
        # Récupérer et publier la pose Lidar
        self.poll_lidar()
        if self.lidar_pose["x"] != 0.0 or self.lidar_pose["y"] != 0.0:  # Éviter (0,0) invalide
            _log_robot("world/lidar/pose",
                       self.lidar_pose["x"], self.lidar_pose["y"], 
                       self.lidar_pose["theta"], C_LIDAR_P)
        
        # Publier les scalaires temporels
        if self.count % 5 == 0:  # Moins fréquent que les positions
            rr.log("data/teensy/x_mm", rr.Scalars(self.odom["x"]))
            rr.log("data/teensy/y_mm", rr.Scalars(self.odom["y"]))
            rr.log("data/teensy/theta_deg", rr.Scalars(math.degrees(self.odom["theta"])))
            
            if self.lidar_pose["confidence"] > 0:
                rr.log("data/lidar/confidence", rr.Scalars(self.lidar_pose["confidence"]))
            
            if self.count % 20 == 0:
                logger.info(f"📍 Teensy: X={self.odom['x']:.1f}mm, Y={self.odom['y']:.1f}mm")


def create_blueprint() -> rrb.Blueprint:
    """Blueprint pour la visualisation Rerun."""
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Spatial3DView(name="Terrain Eurobot 2026", origin="world"),
            rrb.Horizontal(
                rrb.TimeSeriesView(name="Position Teensy (mm)", origin="data/teensy"),
                rrb.TimeSeriesView(name="Position Lidar", origin="data/lidar"),
                column_shares=[2, 1],
            ),
            row_shares=[3, 1],
        ),
    )


def main():
    logger.info("=" * 70)
    logger.info("TEST SIMPLE: Teensy + Lidar → Rerun (SANS fusion)")
    logger.info("=" * 70)
    
    # Initialiser Rerun
    rr.init("test_teensy_lidar", spawn=True)
    rr.send_blueprint(create_blueprint())
    
    # 1. Initialiser COM (hardware ou simulation)
    com, mode = init_robot(logger)
    logger.info(f"✓ Mode: {mode}")
    
    # 2. Créer le logger
    logger_data = SimpleDataLogger()
    
    # 3. Démarrer le thread d'acquisition Lidar
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
    
    # 4. Enregistrer les callbacks Teensy
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
    logger.info("Rerun Viewer devrait s'ouvrir automatiquement.")
    logger.info("Vous devriez voir:")
    logger.info("  🔵 ROBOT (BLEU) = Teensy odométrie brute → world/robot/odom")
    logger.info("  🔴 ROBOT (ROUGE) = Lidar position calculée → world/lidar/pose (trilatération)")
    logger.info("")
    
    # 5. Boucle principale
    try:
        while True:
            time.sleep(0.5)  # Polling Lidar 2x/sec (callback pousse au besoin)
    except KeyboardInterrupt:
        logger.info("\n✓ Arrêt utilisateur")
    finally:
        stop_lidar_runtime()
        if hasattr(com, "close"):
            com.close()
        logger.info("✓ Fermé")


if __name__ == "__main__":
    main()
