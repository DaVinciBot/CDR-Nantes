#!/usr/bin/env python3
"""Test Foxglove avec données REELLES Teensy + Lidar."""

import struct
import time
import logging
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

# Import Lidar
try:
    from lidar.lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime
except ImportError:
    sys.path.insert(0, str(Path(__file__).parent / "lidar"))
    from lidar_logic import get_latest_pose, start_lidar_thread, stop_lidar_runtime

com, mode = init_robot(logger)
logger.info("Mode actif: %s", mode)

# Démarre Foxglove 3D
bridge = FoxgloveBridge(com, port=8765, auto_subscribe=False).start()

logger.info("Bridge Foxglove démarré sur ws://localhost:8765")
logger.info("Démarrage Lidar...")
time.sleep(2)

# Démarrer Lidar
def lidar_console_cb(msg: str):
    logger.info(f"[LIDAR] {msg}")

def lidar_status_cb(status: str):
    logger.info(f"[LIDAR STATUS] {status}")

try:
    lidar_thread = start_lidar_thread(lidar_console_cb, lidar_status_cb)
    time.sleep(2)
except Exception as exc:
    logger.error(f"Erreur Lidar: {exc}")

# Données actuelles
teensy_pos = {"x": 0.0, "y": 0.0, "theta": 0.0}
lidar_pos = {"x": 0.0, "y": 0.0, "theta": 0.0}

def handle_teensy(data: bytes) -> None:
    """Callback Teensy."""
    global teensy_pos
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])
        teensy_pos = {"x": x, "y": y, "theta": theta}
        logger.info(f"Teensy: X={x:.0f}, Y={y:.0f}")

com.add_callback(handle_teensy, Messages.UPDATE_ROLLING_BASIS.value)

logger.info("Écoute Teensy + Lidar...")
logger.info("   ROBOT (ROUGE) = Teensy")
logger.info("   TARGET (VERT) = Lidar")

try:
    while True:
        # Récupérer Lidar
        pose = get_latest_pose()
        if pose:
            lidar_pos = {"x": pose.x, "y": pose.y, "theta": pose.theta}
            logger.info(f"Lidar: X={pose.x:.0f}, Y={pose.y:.0f}, conf={pose.confidence:.2f}")
        
        # Publier Teensy (utilise angle Teensy pour l'instant)
        bridge.publish(teensy_pos["x"], teensy_pos["y"], teensy_pos["theta"])
        
        time.sleep(0.5)  # 2 Hz - suffisant pour le monitoring
        
except KeyboardInterrupt:
    logger.info("\nArrêt")
finally:
    stop_lidar_runtime()
    bridge.stop()
    if hasattr(com, "close"):
        com.close()
    logger.info("Fermé")
