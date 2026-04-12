#!/usr/bin/env python3
"""
Test Lidar Bridge — Wrapper autour du bridge Rerun avec Lidar seul.

Réutilise toute la logique du bridge (carte, blueprint, publication)
en remplaçant juste l'initialisation Teensy par une simulation robot.

Lancement:
    python test_lidar_bridge.py --mode local
    python test_lidar_bridge.py --mode serve --port 9876
"""

import argparse
import math
import sys
import threading
import time
import logging
from pathlib import Path

import rerun as rr

# ─────────────────────────────────────────────────────────────────────────────
# Setup paths & imports
# ─────────────────────────────────────────────────────────────────────────────

_DIR = Path(__file__).parent
sys.path.insert(0, str(_DIR))
sys.path.insert(0, str(_DIR / "rerun"))
sys.path.insert(0, str(_DIR / "lidar"))

logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(message)s")
logger = logging.getLogger("test_lidar_bridge")

# Importer depuis le bridge les fonctions réutilisables
import rerun_bridge
from lidar_logic import (
    get_latest_scan_data,
    get_latest_beacon_candidates,
    get_latest_pose,
    start_lidar_thread,
    stop_lidar_runtime,
    MAP_W_MM,        # Dimensions du terrain depuis lidar_logic
    MAP_H_MM,
)

# ─────────────────────────────────────────────────────────────────────────────
# Constantes du terrain — importées depuis les sources
# ─────────────────────────────────────────────────────────────────────────────

W = float(MAP_W_MM)
H = float(MAP_H_MM)
CX = W / 2
CY = H / 2

C_ROBOT = [51, 153, 255, 220]       # Bleu — position robot


# ─────────────────────────────────────────────────────────────────────────────
# État partagé du bridge
# ─────────────────────────────────────────────────────────────────────────────

_state = rerun_bridge._st  # Réutiliser l'état du bridge


# ─────────────────────────────────────────────────────────────────────────────
# Poller Lidar — appelle les fonctions du bridge
# ─────────────────────────────────────────────────────────────────────────────

def poll_lidar():
    """Récupère les données Lidar et met à jour l'état du bridge."""
    try:
        cloud = get_latest_scan_data()
        if cloud:
            rerun_bridge.update_lidar_cloud(cloud)
        
        beacons = get_latest_beacon_candidates()
        if beacons:
            rerun_bridge.update_lidar_beacons(beacons)
        
        pose = get_latest_pose()
        if pose:
            rerun_bridge.update_lidar_pose(
                pose.x, pose.y, pose.theta, pose.confidence, pose.is_localized
            )
    except Exception as e:
        logger.debug(f"Lidar poll error: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# Simulation robot
# ─────────────────────────────────────────────────────────────────────────────

def robot_sim_loop():
    """Simule le robot en position statique au centre du terrain."""
    logger.info("Robot position: CENTER (simulation statique)")
    
    while True:
        # Position fixe au centre du terrain
        rerun_bridge.update_odom(CX, CY, 0.0)
        
        # Fusion simple : Lidar si ok, sinon position Teensy
        s = _state.snap()
        if s.lidar_ok:
            fused_x = 0.5 * CX + 0.5 * s.lidar_x
            fused_y = 0.5 * CY + 0.5 * s.lidar_y
            rerun_bridge.update_fused(fused_x, fused_y, 0.0)
        else:
            rerun_bridge.update_fused(CX, CY, 0.0)
        
        time.sleep(0.05)


# ─────────────────────────────────────────────────────────────────────────────
# Boucle publication — utilise la fonction du bridge
# ─────────────────────────────────────────────────────────────────────────────

def publish_loop_with_lidar(hz: float = 20.0):
    """Publie l'état à fréquence fixe en pollant le Lidar."""
    dt = 1.0 / hz
    frame_count = 0
    
    while True:
        # Poll Lidar pour mettre à jour l'état
        poll_lidar()
        
        # Utiliser la fonction de publication du bridge
        s = _state.snap()
        rerun_bridge._publish(s)
        
        # Logs périodiques
        if frame_count % (hz * 5) == 0:
            logger.info(
                f"📊 Frame {frame_count} | "
                f"Lidar: conf={s.lidar_conf:.2f} pts={len(s.lidar_cloud)} beacons={len(s.lidar_beacons)}"
            )
        
        frame_count += 1
        time.sleep(dt)


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Test Lidar Bridge — Wrapper du bridge avec Lidar seul",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples:
  # Local (viewer sur PC)
  python test_lidar_bridge.py --mode local
  
  # Serveur WebSocket
  python test_lidar_bridge.py --mode serve --port 9876
  
  # Connexion gRPC distant
  python test_lidar_bridge.py --mode connect --host localhost --port 10000
        """
    )
    p.add_argument("--mode", choices=["local", "serve", "connect"], default="local",
                   help="Mode visualisation Rerun")
    p.add_argument("--host", default="0.0.0.0",
                   help="Host pour serveur")
    p.add_argument("--port", type=int, default=9876,
                   help="Port WebSocket ou gRPC")
    
    args = p.parse_args()
    
    # ─────────────────────────────────────────────────────────────────────────
    # Initialiser Rerun — utiliser les fonctions du bridge
    # ─────────────────────────────────────────────────────────────────────────
    
    logger.info("=" * 70)
    logger.info(" Test Lidar Bridge — Visualisation Rerun (Lidar seul)")
    logger.info("=" * 70)
    
    rr.init("test_lidar_bridge", spawn=(args.mode == "local"))
    
    if args.mode == "serve":
        logger.info(" Mode SERVE activé")
        logger.info(f"   Serveur WebSocket sur {args.host}:{args.port}")
        rr.serve_web(open_browser=False, web_port=args.port)
    elif args.mode == "connect":
        logger.info(f" Connexion gRPC à {args.host}:{args.port}")
        rr.connect_grpc(f"{args.host}:{args.port}")
    
    # Utiliser le blueprint du bridge (pas dupliquer)
    logger.info(" Applying blueprint...")
    rr.send_blueprint(rerun_bridge.create_blueprint())
    
    # Utiliser la carte statique du bridge
    logger.info("  Publishing static map...")
    rerun_bridge.log_static_map()
    
    # Démarrer simulation robot
    logger.info(" Démarrage simulation robot...")
    threading.Thread(target=robot_sim_loop, daemon=True).start()
    
    # Démarrer thread Lidar
    logger.info(" Démarrage Lidar...")
    try:
        console_callback = lambda msg: logger.debug(f"[Lidar] {msg}")
        status_callback = lambda status: logger.debug(f"[Lidar Status] {status}")
        
        start_lidar_thread(console_callback, status_callback)
        logger.info("Lidar thread démarré")
    except Exception as e:
        logger.error(f"Erreur Lidar: {e}")
        sys.exit(1)
    
    # Boucle principale — utiliser la fonction de publication
    logger.info("=" * 70)
    logger.info(" Publication à 20 Hz...")
    logger.info("=" * 70)
    
    try:
        publish_loop_with_lidar(hz=20.0)
    except KeyboardInterrupt:
        logger.info("\n" + "=" * 70)
        logger.info(" Arrêt...")
        try:
            stop_lidar_runtime()
        except:
            pass
        logger.info("✅ Terminé")


if __name__ == "__main__":
    main()
