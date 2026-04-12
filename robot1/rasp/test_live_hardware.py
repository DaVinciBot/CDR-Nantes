#!/usr/bin/env python3
"""
Test Live Hardware — Réutilise l'infrastructure complète du bridge Rerun.

Objectif: Visualiser TOUT le télémétrie hardware sans envoyer de commands:
  • Carte statique complète (balises, caisses, murs, grenier, etc.)
  • Odométrie Teensy (robot bleu)
  • Nuage Lidar brut (points + vue polaire radar)
  • Balises détectées par Lidar (diamants orange)
  • Position calculée Lidar via trilatération (robot rouge)
  • Position fusionnée estimée (robot vert)
  • Trajectoire de test (si fournie)
  • Courbes temporelles de tous les capteurs

Lancement:
    # Mode local (Rerun viewer sur Rasp)
    python test_live_hardware.py --mode local
    
    # Mode serveur WebSocket (accès HTTP depuis PC portable)
    python test_live_hardware.py --mode serve --port 9876
    
    # Avec Lidar réel (trilatération active + détection balises)
    python test_live_hardware.py --mode serve --with-lidar --port 9876
    
    # Mode gRPC (connexion à Rerun Viewer distant)
    python test_live_hardware.py --mode connect --host localhost --port 10000
"""

import argparse
import math
import struct
import sys
import threading
import time
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

# ─────────────────────────────────────────────────────────────────────────────
# Setup paths & imports
# ─────────────────────────────────────────────────────────────────────────────

_DIR = Path(__file__).parent
sys.path.insert(0, str(_DIR))

logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
logger = logging.getLogger("test_live_hardware")

# Importer depuis le bridge pour réutiliser toute l'infra
from rerun import rerun_bridge
from loader import loader
from utils import init_robot

Messages = loader.load_class("usb_com", "Messages")

# Lidar optionnel
try:
    from lidar.lidar_logic import (
        get_latest_scan_data, get_latest_beacon_candidates, get_latest_pose,
    )
    HAS_LIDAR = True
except ImportError:
    HAS_LIDAR = False
    logger.warning("⚠ Lidar module not found")


# ─────────────────────────────────────────────────────────────────────────────
# Réutiliser l'état du bridge
# ─────────────────────────────────────────────────────────────────────────────

_state = rerun_bridge._st  # État partagé du bridge


# ─────────────────────────────────────────────────────────────────────────────
# Callbacks basés sur le bridge
# ─────────────────────────────────────────────────────────────────────────────

def make_teensy_callback():
    """Callback Teensy utilisant rerun_bridge.update_odom()"""
    def cb(data: bytes):
        if len(data) >= 24:
            x, y, t = struct.unpack("<ddd", data[:24])
            rerun_bridge.update_odom(x, y, t)
            logger.debug(f"↓ Teensy odom: x={x:.1f} y={y:.1f} θ={math.degrees(t):.1f}°")
    return cb


def make_lidar_poller():
    """Poller Lidar utilisant les fonctions du bridge"""
    if not HAS_LIDAR:
        return None
    
    def poll():
        try:
            # Nuage Lidar
            cloud = get_latest_scan_data()
            if cloud:
                rerun_bridge.update_lidar_cloud(cloud)
            
            # Balises détectées
            beacons = get_latest_beacon_candidates()
            if beacons:
                rerun_bridge.update_lidar_beacons(beacons)
            
            # Pose calculée via trilatération
            pose = get_latest_pose()
            if pose:
                rerun_bridge.update_lidar_pose(
                    pose.x, pose.y, pose.theta, pose.confidence, pose.is_localized
                )
        except Exception as e:
            logger.debug(f"Lidar poll error: {e}")
    
    return poll


# ─────────────────────────────────────────────────────────────────────────────
# Boucle de publication personnalisée (contrôle de fréquence + logs)
# ─────────────────────────────────────────────────────────────────────────────

def publish_loop(hz: float = 20.0, lidar_poller: Optional[callable] = None) -> None:
    """Publie l'état du bridge à fréquence fixe."""
    dt = 1.0 / hz
    frame_count = 0
    
    while True:
        # Poll Lidar si activé
        if lidar_poller:
            try:
                lidar_poller()
            except Exception as e:
                logger.debug(f"Lidar poller: {e}")
        
        # Publier l'état actuel
        s = _state.snap()
        rerun_bridge._publish(s)
        
        # Logs périodiques
        if frame_count % (hz * 5) == 0:  # Tous les 5 sec
            logger.info(
                f"📊 Frame {frame_count} | "
                f"Teensy: ({s.odom_x:.0f}, {s.odom_y:.0f}) rad={s.odom_theta:.3f} | "
                f"Lidar: conf={s.lidar_conf:.2f} pts={len(s.lidar_cloud)} beacons={len(s.lidar_beacons)}"
            )
        
        frame_count += 1
        time.sleep(dt)


# ─────────────────────────────────────────────────────────────────────────────
# Blueprint complet (reproduit celui du bridge)
# ─────────────────────────────────────────────────────────────────────────────

def create_blueprint() -> rrb.Blueprint:
    """Layout de visualisation complet avec carte 3D + polaire + courbes."""
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(name="Terrain Eurobot 2026", origin="world"),
                rrb.Spatial2DView(name="Lidar Polaire (Radar)", origin="sensors/lidar"),
                column_shares=[1, 1],
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(name="Position Teensy (mm)", origin="data/teensy"),
                rrb.TimeSeriesView(name="Lidar",                origin="data/lidar"),
                rrb.TimeSeriesView(name="Fusionné",             origin="data/fused"),
                rrb.TimeSeriesView(name="Écart odom↔lidar",    origin="data/fusion"),
                column_shares=[3, 2, 2, 2],
            ),
            row_shares=[1, 1],
        ),
    )


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Test Live Hardware — Visualisation complète Rerun",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  # Local (viewer sur Rasp) — test rapide
  python test_live_hardware.py --mode local
  
  # Serveur WebSocket — accès depuis PC portable
  python test_live_hardware.py --mode serve --port 9876
  
  # Avec Lidar réel
  python test_live_hardware.py --mode serve --with-lidar --port 9876
  
  # Connexion gRPC à un viewer distant
  python test_live_hardware.py --mode connect --host 192.168.1.50 --port 10000
        """
    )
    p.add_argument("--mode", choices=["local","serve","connect"], default="local",
                   help="Mode visualisation Rerun")
    p.add_argument("--host", default="0.0.0.0",
                   help="Host pour serveur (0.0.0.0 = accès de partout)")
    p.add_argument("--port", type=int, default=9876,
                   help="Port WebSocket ou gRPC")
    p.add_argument("--with-lidar", action="store_true",
                   help="Active Lidar hardware polling (trilatération + détection balises)")
    
    args = p.parse_args()
    
    # ─────────────────────────────────────────────────────────────────────────
    # Initialiser Rerun
    # ─────────────────────────────────────────────────────────────────────────
    
    rr.init("test_live_hardware", spawn=(args.mode == "local"))
    
    if args.mode == "serve":
        logger.info("🌐 Mode SERVE activé")
        logger.info(f"   Serveur WebSocket sur {args.host}:{args.port}")
        logger.info(f"   Accédez depuis navigateur: http://[RaspIP]:{args.port}")
        rr.serve_web(open_browser=False, web_port=args.port)
    elif args.mode == "connect":
        logger.info(f"🔗 Connexion gRPC à {args.host}:{args.port}")
        rr.connect_grpc(f"{args.host}:{args.port}")
    
    # Appliquer le blueprint complet
    logger.info("📐 Applying blueprint...")
    rr.send_blueprint(create_blueprint())
    
    # ─────────────────────────────────────────────────────────────────────────
    # Publier la carte statique (du bridge)
    # ─────────────────────────────────────────────────────────────────────────
    
    logger.info("🗺️  Publishing static map...")
    rerun_bridge.log_static_map()
    
    # ─────────────────────────────────────────────────────────────────────────
    # Initialiser hardware Teensy
    # ─────────────────────────────────────────────────────────────────────────
    
    logger.info("=" * 70)
    try:
        com, mode = init_robot(logger)
        logger.info(f"✓ Teensy connecté (mode: {mode})")
    except Exception as e:
        logger.error(f"❌ Teensy init failed: {e}")
        com, mode = None, None
    logger.info("=" * 70)
    
    # ─────────────────────────────────────────────────────────────────────────
    # Préparer les callbacks
    # ─────────────────────────────────────────────────────────────────────────
    
    lidar_poller = None
    if args.with_lidar:
        lidar_poller = make_lidar_poller()
        if lidar_poller:
            logger.info("✓ Lidar poller activé")
        else:
            logger.warning("⚠ Lidar poller non disponible")
    
    # ─────────────────────────────────────────────────────────────────────────
    # Ajouter callback Teensy
    # ─────────────────────────────────────────────────────────────────────────
    
    if com:
        try:
            com.add_callback(make_teensy_callback(), Messages.UPDATE_ROLLING_BASIS.value)
            logger.info("✓ Callback Teensy enregistré")
        except Exception as e:
            logger.warning(f"⚠ Callback registration failed: {e}")
    else:
        logger.warning("⚠ Aucun hardware Teensy — données statiques seulement")
    
    # ─────────────────────────────────────────────────────────────────────────
    # Boucle principale
    # ─────────────────────────────────────────────────────────────────────────
    
    logger.info("=" * 70)
    logger.info(f"✅ Initialisation complète")
    logger.info(f"   Mode: {args.mode}:/{args.port}")
    logger.info(f"   Lidar: {'Oui' if args.with_lidar else 'Non'}")
    logger.info(f"   Hardware: {'Teensy trouvé' if com else 'Aucun hardware'}")
    logger.info("▶ Publication à 20 Hz...")
    logger.info("=" * 70)
    
    try:
        publish_loop(hz=20.0, lidar_poller=lidar_poller)
    except KeyboardInterrupt:
        logger.info("\n" + "=" * 70)
        logger.info("🛑 Arrêt...")
        logger.info("✅ Terminé")


if __name__ == "__main__":
    main()
