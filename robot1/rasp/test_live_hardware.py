#!/usr/bin/env python3
"""
Test en direct : affiche les callbacks Teensy + fusion Lidar dans Rerun.

Objectif: Visualiser les données brutes sans envoyer de commands de mouvement.
  • Affiche l'odométrie Teensy
  • Affiche la pose Lidar (fusion trilatération)
  • Logs temps réel (pas d'envois, juste réception)

Lancement:
    # Mode local (Rerun viewer sur Rasp)
    python test_live_hardware.py --mode local
    
    # Mode serveur (accès HTTP depuis PC)
    python test_live_hardware.py --mode serve --port 9876
    
    # Avec Lidar réel (trilatération active)
    python test_live_hardware.py --mode serve --with-lidar --port 9876
"""

import struct
import sys
import time
import logging
import math
import subprocess
from pathlib import Path
from threading import Thread

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

# Configurer les chemins
sys.path.insert(0, str(Path(__file__).parent))

from loader import loader
from utils import init_robot

# Import Lidar functions (optionnel)
try:
    from lidar.lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime
    HAS_LIDAR = True
except ImportError:
    HAS_LIDAR = False
    logger_info = logging.getLogger("test_live_hardware")
    logger_info.warning("⚠ Lidar module not found, skipping lidar fusion")

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s | %(name)s | %(message)s"
)
logger = logging.getLogger("test_live_hardware")

Messages = loader.load_class("usb_com", "Messages")

# ─────────────────────────────────────────────────────────────────────────────
# Constantes — repère terrain (coin bas-gauche = origine)
# ─────────────────────────────────────────────────────────────────────────────

W  = 3000.0
H  = 2000.0
CX = W / 2

# Couleurs RGBA uint8
C_ROBOT   = [51, 153, 255, 220]      # Bleu — Teensy odom
C_LIDAR_P = [80, 255,120, 255]       # Vert — Lidar pose (fusion)

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


class LiveHardwareDisplay:
    """Reçoit Teensy + Lidar et affiche dans Rerun (sans envois de commands)."""
    
    def __init__(self, with_lidar: bool = False):
        self.odom = {"x": 0.0, "y": 0.0, "theta": 0.0}
        self.lidar_pose = {"x": 0.0, "y": 0.0, "theta": 0.0, "confidence": 0.0}
        self.count = 0
        self.with_lidar = with_lidar and HAS_LIDAR
        
        if self.with_lidar:
            logger.info("🚀 Démarrage du thread Lidar...")
            start_lidar_thread()
            time.sleep(1)  # Laisser le temps au Lidar de démarrer
    
    def on_teensy_odom(self, data: bytes) -> None:
        """Callback Teensy odométrie — juste afficher, pas d'envoi."""
        if len(data) < 24:
            logger.warning(f"❌ Payload Teensy trop court: {len(data)} bytes")
            return
        
        x, y, theta = struct.unpack("<ddd", data[:24])
        self.odom = {"x": x, "y": y, "theta": theta}
        
        self.count += 1
        
        # Afficher dans Rerun
        self._publish()
        
        # Logs tous les 20 appels
        if self.count % 20 == 0:
            logger.info(
                f"📊 Teensy: X={x:.1f}mm, Y={y:.1f}mm, θ={theta:.4f}rad "
                f"({math.degrees(theta):.1f}°) [count={self.count}]"
            )
    
    def poll_lidar(self) -> None:
        """Interroge la pose Lidar calculée par le PoseEngine (si activé)."""
        if not self.with_lidar:
            return
        
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
                        f"🎯 Lidar Fusion: X={pose.x:.1f}mm, Y={pose.y:.1f}mm, "
                        f"θ={pose.theta:.4f}rad, conf={pose.confidence:.2f}, "
                        f"localized={pose.is_localized}"
                    )
        except Exception as exc:
            pass  # Silencieusement ignorer les erreurs de pose
    
    def _publish(self) -> None:
        """Envoie les données à Rerun."""
        # Position Teensy brute (toujours affichée)
        _log_robot("world/robot/odom", 
                   self.odom["x"], self.odom["y"], self.odom["theta"], C_ROBOT)
        
        # Récupérer et publier la pose Lidar (si activé)
        self.poll_lidar()
        if self.with_lidar and (self.lidar_pose["x"] != 0.0 or self.lidar_pose["y"] != 0.0):
            _log_robot("world/lidar/pose",
                       self.lidar_pose["x"], self.lidar_pose["y"], 
                       self.lidar_pose["theta"], C_LIDAR_P)
        
        # Publier les scalaires temporels (moins fréquent)
        if self.count % 5 == 0:
            rr.log("data/teensy/x_mm", rr.Scalars(self.odom["x"]))
            rr.log("data/teensy/y_mm", rr.Scalars(self.odom["y"]))
            rr.log("data/teensy/theta_deg", rr.Scalars(math.degrees(self.odom["theta"])))
            
            if self.with_lidar and self.lidar_pose["confidence"] > 0:
                rr.log("data/lidar/confidence", rr.Scalars(self.lidar_pose["confidence"]))
                rr.log("data/lidar/x_mm", rr.Scalars(self.lidar_pose["x"]))
                rr.log("data/lidar/y_mm", rr.Scalars(self.lidar_pose["y"]))


def create_blueprint():
    """Crée le layout Rerun pour visualisation."""
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial3DView(origin="world", name="Terrain 3D"),
            rrb.TimeSeriesView(name="Données en temps réel"),
        )
    )


def main():
    import argparse
    
    p = argparse.ArgumentParser(description="Test Live — Teensy + Lidar dans Rerun")
    p.add_argument("--mode", choices=["local", "serve", "connect"], default="local",
                   help="Mode Rerun viewer")
    p.add_argument("--host", default="0.0.0.0",
                   help="Host pour mode serve")
    p.add_argument("--port", type=int, default=9876,
                   help="Port pour mode serve")
    p.add_argument("--with-lidar", action="store_true",
                   help="Active la fusion Lidar (trilatération)")
    
    args = p.parse_args()
    
    # Initialiser Rerun
    rr.init(
        "test_live_hardware",
        spawn=args.mode == "local"
    )
    
    if args.mode == "serve":
        rr.serve(open_browser=False, host=args.host, port=args.port)
        logger.info(f"🌐 Serveur Rerun actif: http://{args.host}:{args.port}")
    elif args.mode == "connect":
        rr.connect()
    
    # Appliquer le blueprint
    rr.send_blueprint(create_blueprint())
    
    # Initialiser le robot
    logger.info("=" * 70)
    com, mode = init_robot(logger)
    logger.info(f"Mode détecté: {mode}")
    logger.info("=" * 70)
    
    # Créer le data logger
    logger.info(f"✅ Initialisation complète (avec_lidar={args.with_lidar})")
    logger.info("⏳ En attente de données Teensy...")
    logger.info("=" * 70)
    
    display = LiveHardwareDisplay(with_lidar=args.with_lidar)
    
    # Ajouter le callback Teensy
    com.add_callback(display.on_teensy_odom, Messages.UPDATE_ROLLING_BASIS.value)
    
    # Boucle principale — juste afficher les données reçues
    try:
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        logger.info("\n" + "=" * 70)
        logger.info("🛑 Arrêt...")
        if display.with_lidar:
            try:
                stop_lidar_runtime()
            except:
                pass
        logger.info("✅ Terminé")


if __name__ == "__main__":
    main()
