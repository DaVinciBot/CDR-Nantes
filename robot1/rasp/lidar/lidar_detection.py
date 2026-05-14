"""
lidar_detection.py
──────────────────
Détection du robot adverse par LiDAR A2M12 — version simple.

Centroïde brut du cluster + lissage temporel + gating de vitesse.
Pas de fit géométrique : marche quelle que soit la forme de l'objet.

API publique :
    start()                        → lance le thread LiDAR
    stop()                         → arrête proprement
    get_opponent()                 → (x_mm, y_mm, confiance) ou None
    update_robot_pose(x, y, theta) → appelé depuis robot.py à chaque update()
"""

import math
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple, List

import numpy as np
from rplidar import RPLidar

logger = logging.getLogger("LIDAR_DETECT")

# ── CONFIGURATION ─────────────────────────────────────────────────────────────

# Matériel
PORT     = 'COM5'
BAUDRATE = 256000
TIMEOUT  = 3

# Filtres
MIN_DIST_MM    = 200     # dead zone fiable du A2M12
DETECT_DIST_MM = 1500     # < 520mm → murs du terrain test invisibles
                         # CDR : remonter à 1500
MIN_QUALITY    = 5      # qualité RPLidar (0–15), filtre le bruit
                         # baisser à 5-7 si trop de pertes de points

# Terrain (mm)
# Test : 1040 × 1040     CDR : 3000 × 2000
FIELD_W      = 3000
FIELD_H      = 2000
FIELD_MARGIN = 500       # CDR : 500

# Clustering
CLUSTER_GAP_MM  = 80     # distance max entre 2 points consécutifs d'un cluster
CLUSTER_MIN_PTS = 3      # cluster trop petit = bruit

# Tracking temporel
MAX_MISSED_SCANS    = 5        # scans sans détection avant oubli
ALPHA_SMOOTH        = 0.4      # lissage exponentiel (0=très lisse, 1=brut)
MAX_OPP_SPEED_MM_S  = 2500     # vitesse max plausible adversaire CDR
STALE_TIMEOUT_S     = 0.5      # timeout de fraîcheur

# Offset angulaire LiDAR (degrés)
# 0   si câble LiDAR = avant du robot
# 180 si câble LiDAR = arrière du robot
ANGLE_OFFSET_DEG = 0

# ── ÉTAT PARTAGÉ ──────────────────────────────────────────────────────────────

@dataclass
class _OpponentState:
    x:          float = 0.0
    y:          float = 0.0
    confidence: float = 0.0
    missed:     int   = 0
    timestamp:  float = 0.0

@dataclass
class _RobotPose:
    x:     float = 0.0
    y:     float = 0.0
    theta: float = 0.0

_opponent      = _OpponentState()
_robot_pose    = _RobotPose()
_opponent_lock = threading.Lock()
_pose_lock     = threading.Lock()
_running       = False
_thread: Optional[threading.Thread] = None

# ── API PUBLIQUE ──────────────────────────────────────────────────────────────

def update_robot_pose(x: float, y: float, theta: float) -> None:
    """Met à jour la pose du robot (odométrie Teensy)."""
    with _pose_lock:
        _robot_pose.x     = x
        _robot_pose.y     = y
        _robot_pose.theta = theta


def get_opponent() -> Optional[Tuple[float, float, float]]:
    """
    Retourne (x_mm, y_mm, confiance) ou None.
    Coordonnées dans le repère terrain.
    """
    with _opponent_lock:
        if _opponent.confidence < 0.1 or _opponent.timestamp == 0.0:
            return None
        if time.time() - _opponent.timestamp > STALE_TIMEOUT_S:
            return None
        return (_opponent.x, _opponent.y, _opponent.confidence)


def start() -> None:
    """Lance le thread d'acquisition LiDAR."""
    global _running, _thread
    if _thread and _thread.is_alive():
        logger.warning("Thread LiDAR déjà actif.")
        return
    _running = True
    _thread = threading.Thread(target=_lidar_loop, daemon=True, name="LidarDetect")
    _thread.start()
    logger.info("Thread LiDAR démarré.")


def stop() -> None:
    """Arrête proprement le thread LiDAR (attente max 3 s)."""
    global _running
    _running = False
    if _thread:
        _thread.join(timeout=3.0)
    logger.info("Thread LiDAR arrêté.")

# ── TRAITEMENT D'UN SCAN ─────────────────────────────────────────────────────

def _process_scan(raw_scan) -> None:
    """Traite un scan complet : filtre → terrain → cluster → tracking."""

    # 1. Filtre qualité/distance + projection repère robot
    pts_robot = []
    for quality, angle_deg, dist_mm in raw_scan:
        if dist_mm < MIN_DIST_MM or dist_mm > DETECT_DIST_MM:
            continue
        if quality < MIN_QUALITY:
            continue
        rad = math.radians(angle_deg + ANGLE_OFFSET_DEG)
        pts_robot.append((dist_mm * math.sin(rad), dist_mm * math.cos(rad)))

    if len(pts_robot) < CLUSTER_MIN_PTS:
        _mark_missed()
        return

    # 2. Conversion en coordonnées terrain
    with _pose_lock:
        rx, ry, rtheta = _robot_pose.x, _robot_pose.y, _robot_pose.theta

    cos_t, sin_t = math.cos(rtheta), math.sin(rtheta)
    pts_terrain = []
    for xr, yr in pts_robot:
        x_t = rx + xr * cos_t - yr * sin_t
        y_t = ry + xr * sin_t + yr * cos_t
        if (-FIELD_MARGIN <= x_t <= FIELD_W + FIELD_MARGIN and
            -FIELD_MARGIN <= y_t <= FIELD_H + FIELD_MARGIN):
            pts_terrain.append((x_t, y_t))

    if len(pts_terrain) < CLUSTER_MIN_PTS:
        _mark_missed()
        return

    # 3. Clustering angulaire
    pts_sorted = sorted(pts_terrain,
                        key=lambda p: math.atan2(p[1] - ry, p[0] - rx))

    clusters: List[List[Tuple[float, float]]] = []
    current = [pts_sorted[0]]
    for i in range(1, len(pts_sorted)):
        if math.hypot(pts_sorted[i][0] - current[-1][0],
                      pts_sorted[i][1] - current[-1][1]) > CLUSTER_GAP_MM:
            clusters.append(current)
            current = [pts_sorted[i]]
        else:
            current.append(pts_sorted[i])
    clusters.append(current)

    # 4. Meilleur cluster : le plus proche avec assez de points
    best = None
    best_dist = float('inf')
    for cluster in clusters:
        if len(cluster) < CLUSTER_MIN_PTS:
            continue
        arr = np.array(cluster, dtype=np.float32)
        cx = float(arr[:, 0].mean())
        cy = float(arr[:, 1].mean())
        dist = math.hypot(cx - rx, cy - ry)
        if dist < best_dist:
            best_dist = dist
            conf = min(1.0, len(cluster) / 8.0)
            best = (cx, cy, conf, len(cluster))

    # 5. Mise à jour avec lissage + gating
    now = time.time()
    with _opponent_lock:
        if best is None:
            _opponent.missed += 1
            if _opponent.missed > MAX_MISSED_SCANS:
                _opponent.confidence = 0.0
            return

        cx, cy, conf, npts = best

        # Gating : rejette les sauts physiquement impossibles
        if _opponent.confidence > 0.1 and _opponent.missed < MAX_MISSED_SCANS:
            dt = max(now - _opponent.timestamp, 0.05)
            max_jump = MAX_OPP_SPEED_MM_S * dt
            jump = math.hypot(cx - _opponent.x, cy - _opponent.y)
            if jump > max_jump:
                logger.debug(f"GATING saut={jump:.0f}mm > max={max_jump:.0f}mm → rejeté")
                _opponent.missed += 1
                return
            # Lissage exponentiel
            _opponent.x = ALPHA_SMOOTH * cx + (1 - ALPHA_SMOOTH) * _opponent.x
            _opponent.y = ALPHA_SMOOTH * cy + (1 - ALPHA_SMOOTH) * _opponent.y
        else:
            # Nouvelle détection : pas de lissage
            _opponent.x = cx
            _opponent.y = cy

        _opponent.confidence = conf
        _opponent.missed     = 0
        _opponent.timestamp  = now
        logger.debug(
            f"OPP ({_opponent.x:6.0f},{_opponent.y:6.0f}) "
            f"d={best_dist:5.0f}mm pts={npts} conf={conf:.2f}"
        )


def _mark_missed() -> None:
    """Incrémente le compteur d'absence."""
    with _opponent_lock:
        _opponent.missed += 1
        if _opponent.missed > MAX_MISSED_SCANS:
            _opponent.confidence = 0.0

# ── THREAD D'ACQUISITION ─────────────────────────────────────────────────────

def _lidar_loop() -> None:
    """Boucle d'acquisition. Tourne dans un thread daemon."""
    lidar = None
    try:
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        lidar._serial.flushInput()
        time.sleep(0.5)
        logger.info(f"LiDAR connecté sur {PORT}.")

        try:
            scan_iter = lidar.iter_scans(max_buf_meas=1500, min_len=20)
        except TypeError:
            scan_iter = lidar.iter_scans()

        for scan in scan_iter:
            if not _running:
                break
            _process_scan(scan)

    except Exception as exc:
        logger.error(f"Erreur LiDAR : {exc}")
    finally:
        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
        logger.info("Thread LiDAR terminé.")