"""
lidar_detection.py
──────────────────
Détection du robot adverse par LiDAR, version SIMPLIFIÉE.

Ce fichier fait UNE seule chose : lire le LiDAR A2M12 et retourner
la position du robot adverse en coordonnées terrain (mm).

La pose du robot vient de l'odométrie Teensy — aucune localisation LiDAR.

API publique (3 fonctions) :
    start()                        → lance le thread LiDAR
    stop()                         → arrête proprement
    get_opponent()                 → (x_mm, y_mm, confiance) ou None
    update_robot_pose(x, y, theta) → appelé depuis robot.py à chaque update()

Intégration dans robot.py :
    import lidar_detection as lidar

    # Dans __init__ :
    lidar.start()

    # Dans update() :
    lidar.update_robot_pose(self.x, self.y, self.theta)
    opp = lidar.get_opponent()
    if opp:
        opp_x, opp_y, conf = opp
        obstacles.append((opp_x, opp_y))

    # Dans stopper_tout() :
    lidar.stop()
"""

import math
import threading
import time
import logging
from dataclasses import dataclass, field
from typing import Optional, Tuple, List

import numpy as np
from rplidar import RPLidar

logger = logging.getLogger("LIDAR_DETECT")

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
# Matériel
PORT     = "/dev/ttyUSB0"
BAUDRATE = 256000
TIMEOUT  = 3          # secondes

# Filtres de base
MIN_DIST_MM = 50      # ignore les points trop proches (bruit interne)
MAX_DIST_MM = 3500   # légèrement plus grand que la diagonale du terrain (3606 mm)
MIN_QUALITY = 3       # qualité RPLidar (0–15), 3 = valeur sûre

# Terrain (mm)
FIELD_W = 3000
FIELD_H = 2000
FIELD_MARGIN = 150    # tolérance hors-terrain pour ne pas rejeter les bords

# Clustering angulaire
CLUSTER_GAP_MM   = 120    # si deux points consécutifs sont plus éloignés → nouvelle grappe
CLUSTER_MIN_PTS  = 3      # grappe trop petite → bruit

# Taille du robot adverse
ROBOT_RADIUS_MIN_MM = 60   # rayon minimal du cluster pour être un robot
ROBOT_RADIUS_MAX_MM = 220  # rayon maximal

# Anti-stale : on oublie l'adversaire s'il n'est plus vu depuis N scans
MAX_MISSED_SCANS = 10

# ── ÉTAT PARTAGÉ ──────────────────────────────────────────────────────────────
@dataclass
class _OpponentState:
    x:           float = 0.0
    y:           float = 0.0
    confidence:  float = 0.0
    missed:      int   = 0
    timestamp:   float = 0.0

@dataclass
class _RobotPose:
    x:     float = 0.0
    y:     float = 0.0
    theta: float = 0.0   # radians, 0 = axe X du terrain

_opponent      = _OpponentState()
_robot_pose    = _RobotPose()
_opponent_lock = threading.Lock()
_pose_lock     = threading.Lock()
_running       = False
_thread: Optional[threading.Thread] = None

# ── API PUBLIQUE ───────────────────────────────────────────────────────────────

def update_robot_pose(x: float, y: float, theta: float) -> None:
    """Met à jour la pose du robot depuis l'odométrie Teensy."""
    with _pose_lock:
        _robot_pose.x     = x
        _robot_pose.y     = y
        _robot_pose.theta = theta


def get_opponent() -> Optional[Tuple[float, float, float]]:
    """
    Retourne (x_mm, y_mm, confiance) si un adversaire est détecté, None sinon.
    Les coordonnées sont dans le repère terrain (0–3000, 0–2000).
    """
    with _opponent_lock:
        if _opponent.confidence < 0.1:
            return None
        if time.time() - _opponent.timestamp > 1.0:   # stale > 1 s → None
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

# ── TRAITEMENT D'UN SCAN ───────────────────────────────────────────────────────

def _process_scan(raw_scan) -> None:
    """
    Traite un scan brut RPLidar :
      1. Filtre distance / qualité
      2. Convertit en coordonnées terrain via pose odométrie
      3. Cluster les points
      4. Sélectionne le meilleur candidat adverse
      5. Met à jour _opponent
    """
    # ── 1. Filtre et conversion en coordonnées relatives robot ────────────────
    # Le LiDAR renvoie (qualité, angle_deg, distance_mm)
    # On convertit en (x_robot, y_robot) dans le repère centré sur le robot.
    pts_robot = []
    for quality, angle_deg, dist_mm in raw_scan:
        if dist_mm < MIN_DIST_MM or dist_mm > MAX_DIST_MM:
            continue
        if quality < MIN_QUALITY:
            continue
        rad = math.radians(angle_deg)
        pts_robot.append((dist_mm * math.cos(rad), dist_mm * math.sin(rad)))

    if len(pts_robot) < CLUSTER_MIN_PTS:
        _mark_missed()
        return

    # ── 2. Conversion en coordonnées terrain ─────────────────────────────────
    # On applique la rotation et la translation de la pose odométrie.
    with _pose_lock:
        rx, ry, rtheta = _robot_pose.x, _robot_pose.y, _robot_pose.theta

    cos_t, sin_t = math.cos(rtheta), math.sin(rtheta)

    pts_terrain = []
    for xr, yr in pts_robot:
        # Rotation vers le repère terrain, puis translation
        x_t = rx + xr * cos_t - yr * sin_t
        y_t = ry + xr * sin_t + yr * cos_t
        # Garde uniquement les points dans (ou proches de) le terrain
        if (-FIELD_MARGIN <= x_t <= FIELD_W + FIELD_MARGIN and
                -FIELD_MARGIN <= y_t <= FIELD_H + FIELD_MARGIN):
            pts_terrain.append((x_t, y_t))

    if len(pts_terrain) < CLUSTER_MIN_PTS:
        _mark_missed()
        return

    # ── 3. Clustering angulaire simple ────────────────────────────────────────
    # On trie par angle (vue du robot) et on coupe quand le gap est grand.
    # C'est rapide et suffisant pour un terrain 2×3 m.
    def angle_from_robot(pt):
        return math.atan2(pt[1] - ry, pt[0] - rx)

    pts_sorted = sorted(pts_terrain, key=angle_from_robot)

    clusters: List[List[Tuple[float, float]]] = []
    current = [pts_sorted[0]]

    for i in range(1, len(pts_sorted)):
        px, py = pts_sorted[i]
        qx, qy = current[-1]
        gap = math.hypot(px - qx, py - qy)
        if gap > CLUSTER_GAP_MM:
            clusters.append(current)
            current = [(px, py)]
        else:
            current.append((px, py))
    clusters.append(current)

    # ── 4. Sélection du meilleur candidat ─────────────────────────────────────
    best = None
    best_conf = 0.0

    for cluster in clusters:
        if len(cluster) < CLUSTER_MIN_PTS:
            continue

        arr = np.array(cluster, dtype=np.float32)
        cx, cy = float(arr[:, 0].mean()), float(arr[:, 1].mean())

        # Rayon = distance max du centroïde (enveloppe du cluster)
        radii  = np.linalg.norm(arr - np.array([cx, cy]), axis=1)
        radius = float(radii.max())

        if not (ROBOT_RADIUS_MIN_MM <= radius <= ROBOT_RADIUS_MAX_MM):
            continue  # trop petit (bruit) ou trop grand (mur)

        # Confiance basée sur le nombre de points et la circularité
        circularity = float(radii.std() / (radii.mean() + 1e-6))
        n_score     = min(1.0, len(cluster) / 20.0)   # 0→1 quand npts→20
        c_score     = max(0.0, 1.0 - circularity)     # 1 = cercle parfait
        conf        = 0.5 * n_score + 0.5 * c_score

        if conf > best_conf:
            best_conf = conf
            best      = (cx, cy, conf)

    # ── 5. Mise à jour état partagé ────────────────────────────────────────────
    with _opponent_lock:
        if best is not None:
            _opponent.x          = best[0]
            _opponent.y          = best[1]
            _opponent.confidence = best[2]
            _opponent.missed     = 0
            _opponent.timestamp  = time.time()
            logger.debug(
                f"Adversaire: ({best[0]:.0f}, {best[1]:.0f}) mm  conf={best[2]:.2f}"
            )
        else:
            _mark_missed()


def _mark_missed() -> None:
    """Incrémente le compteur d'absence ; efface l'adversaire si trop absent."""
    with _opponent_lock:
        _opponent.missed += 1
        if _opponent.missed > MAX_MISSED_SCANS:
            _opponent.confidence = 0.0

# ── THREAD D'ACQUISITION ───────────────────────────────────────────────────────

def _lidar_loop() -> None:
    """Boucle principale d'acquisition. Tourne dans un thread daemon."""
    lidar = None
    try:
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        lidar._serial.flushInput()
        time.sleep(0.5)
        logger.info(f"LiDAR connecté sur {PORT}.")

        # iter_scans() accepte ou non des kwargs selon la version rplidar
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