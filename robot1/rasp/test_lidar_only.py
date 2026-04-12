#!/usr/bin/env python3
"""
Test Lidar SEUL — fusion trilatération affichée dans Rerun.

Objectif: Visualiser UNIQUEMENT la pose Lidar (sans dépendre du Teensy).
  • Lance le thread Lidar (trilatération via 3 balises)
  • Affiche la position calculée
  • Logs en temps réel

Lancement:
    python test_lidar_only.py --mode local
    python test_lidar_only.py --mode serve --port 9876
"""

import sys
import time
import logging
import math
from pathlib import Path

import rerun as rr
import rerun.blueprint as rrb

# Configurer les chemins
sys.path.insert(0, str(Path(__file__).parent))

# Import Lidar
try:
    from lidar.lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime
except ImportError:
    sys.path.insert(0, str(Path(__file__).parent / "lidar"))
    from lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)s | %(message)s"
)
logger = logging.getLogger("test_lidar_only")

# ─────────────────────────────────────────────────────────────────────────────
# Constantes
# ─────────────────────────────────────────────────────────────────────────────

W  = 3000.0
H  = 2000.0
CX = W / 2

C_LIDAR = [80, 255, 120, 255]  # Vert — Lidar pose

# ─────────────────────────────────────────────────────────────────────────────

def _log_robot(entity_path: str, x: float, y: float, theta: float, color: list) -> None:
    """Publie un robot (cylindre + flèche de direction)."""
    ct, st = math.cos(theta), math.sin(theta)
    
    # Cylindre robot
    rr.log(f"{entity_path}/body", rr.Cylinders3D(
        centers=[[x, y, 50]],
        radii=[150],
        half_lengths=[50],
        rotations=[rr.Quaternion(xyzw=[0, 0, math.sin(theta/2), math.cos(theta/2)])],
        colors=[color],
    ))
    
    # Flèche de direction
    arrow_tip = [x + 200*ct, y + 200*st, 50]
    rr.log(f"{entity_path}/arrow", rr.LineStrips3D(
        strips=[[[x, y, 50], arrow_tip]],
        colors=[color], radii=[8.0],
    ))


class LidarOnlyDisplay:
    """Affiche UNIQUEMENT la pose Lidar."""
    
    def __init__(self):
        self.lidar_pose = {"x": 0.0, "y": 0.0, "theta": 0.0, "confidence": 0.0}
        self.count = 0
        
        logger.info("🚀 Démarrage du thread Lidar...")
        start_lidar_thread()
        time.sleep(1)
        logger.info("✅ Lidar prêt")
    
    def poll_and_publish(self) -> None:
        """Interroge la pose Lidar et affiche."""
        try:
            pose = get_latest_pose()
            if pose and (pose.x != 0.0 or pose.y != 0.0):  # Éviter (0,0) invalide
                self.lidar_pose = {
                    "x": pose.x,
                    "y": pose.y,
                    "theta": pose.theta,
                    "confidence": pose.confidence,
                    "is_localized": pose.is_localized
                }
                
                # Afficher le robot
                _log_robot("world/lidar/pose",
                           pose.x, pose.y, pose.theta, C_LIDAR)
                
                # Scalaires temps réel
                self.count += 1
                if self.count % 5 == 0:
                    rr.log("data/lidar/x_mm", rr.Scalars(pose.x))
                    rr.log("data/lidar/y_mm", rr.Scalars(pose.y))
                    rr.log("data/lidar/theta_deg", rr.Scalars(math.degrees(pose.theta)))
                    rr.log("data/lidar/confidence", rr.Scalars(pose.confidence))
                
                # Logs tous les 20 appels
                if self.count % 20 == 0:
                    logger.info(
                        f"🎯 Lidar: X={pose.x:.1f}mm, Y={pose.y:.1f}mm, "
                        f"θ={pose.theta:.4f}rad ({math.degrees(pose.theta):.1f}°), "
                        f"conf={pose.confidence:.2f}, localized={pose.is_localized}"
                    )
        except Exception as exc:
            pass  # Silencieusement ignorer


def create_blueprint():
    """Crée le layout Rerun."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world", name="Lidar Pose"),
            rrb.TimeSeriesView(name="Confidence & Position"),
        )
    )


def main():
    import argparse
    
    p = argparse.ArgumentParser(description="Test Lidar SEUL")
    p.add_argument("--mode", choices=["local", "serve", "connect"], default="local",
                   help="Mode Rerun viewer")
    p.add_argument("--host", default="0.0.0.0", help="Host pour mode serve")
    p.add_argument("--port", type=int, default=9876, help="Port pour mode serve")
    
    args = p.parse_args()
    
    # Initialiser Rerun
    rr.init("test_lidar_only", spawn=args.mode == "local")
    
    if args.mode == "serve":
        rr.serve(open_browser=False, host=args.host, port=args.port)
        logger.info(f"🌐 Serveur Rerun: http://{args.host}:{args.port}")
    elif args.mode == "connect":
        rr.connect()
    
    rr.send_blueprint(create_blueprint())
    
    # Démarrer Lidar
    logger.info("=" * 70)
    display = LidarOnlyDisplay()
    logger.info("=" * 70)
    logger.info("⏳ Affichage des données Lidar...")
    logger.info("=" * 70)
    
    # Boucle principale
    try:
        while True:
            display.poll_and_publish()
            time.sleep(0.05)  # 20 Hz
    
    except KeyboardInterrupt:
        logger.info("\n" + "=" * 70)
        logger.info("🛑 Arrêt...")
        try:
            stop_lidar_runtime()
        except:
            pass
        logger.info("✅ Terminé")


if __name__ == "__main__":
    main()
