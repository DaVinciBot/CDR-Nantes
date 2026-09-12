"""Opponent detection from the A2M12 LiDAR.

Raw cluster centroid, exponential smoothing, speed gating. No geometric fit, so
it works whatever the shape of the object.

This is the only perception layer the match loop (app.py) uses. The beacon and
SVD stack lives in localization.py and is frozen, see vision/README.md.

Public API:
    start()                        start the acquisition thread, returns success
    stop()                         stop it cleanly
    get_opponent()                 (x_mm, y_mm, confidence) or None
    get_status()                   (connected, last_error)
    update_robot_pose(x, y, theta) called by app.py on every update()
"""

import math
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple, List

import numpy as np
from rplidar import RPLidar

from .lidar_config import BAUDRATE, PORT, TIMEOUT

logger = logging.getLogger("LIDAR_DETECT")

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
# Serial link: PORT / BAUDRATE / TIMEOUT come from config.json through
# lidar_config.py. Never hardcode a port here, it is what silently disabled
# opponent detection on the robot until 11/09/2026.

# Filters
MIN_DIST_MM    = 200     # reliable dead zone of the A2M12
DETECT_DIST_MM = 1500    # < 520 mm hides the walls of the test field
                         # CDR value: 1500
MIN_QUALITY    = 5       # RPLidar quality (0-15), filters out noise
                         # lower to 5-7 if too many points are dropped

# Field (mm)
# Test field: 1040 x 1040   CDR field: 3000 x 2000
FIELD_W      = 3000
FIELD_H      = 2000
FIELD_MARGIN = 500       # CDR value: 500

# Clustering
CLUSTER_GAP_MM  = 80     # max distance between two consecutive points of a cluster
CLUSTER_MIN_PTS = 3      # a cluster this small is noise

# Time tracking
MAX_MISSED_SCANS    = 5        # scans without detection before forgetting
ALPHA_SMOOTH        = 0.4      # exponential smoothing (0=very smooth, 1=raw)
MAX_OPP_SPEED_MM_S  = 2500     # plausible max opponent speed at the CDR
STALE_TIMEOUT_S     = 0.5      # freshness timeout

# LiDAR angular offset (degrees)
# 0   if the LiDAR cable points to the front of the robot
# 180 if it points to the rear
ANGLE_OFFSET_DEG = 0

# How long start() waits for the first successful connection.
CONNECT_TIMEOUT_S = 3.0

# ── SHARED STATE ──────────────────────────────────────────────────────────────

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

@dataclass
class _Status:
    connected:  bool = False
    last_error: str  = ""

_opponent      = _OpponentState()
_robot_pose    = _RobotPose()
_status        = _Status()
_opponent_lock = threading.Lock()
_pose_lock     = threading.Lock()
_status_lock   = threading.Lock()
_running       = False
_thread: Optional[threading.Thread] = None

# ── PUBLIC API ────────────────────────────────────────────────────────────────

def update_robot_pose(x: float, y: float, theta: float) -> None:
    """Update the robot pose (Teensy odometry)."""
    with _pose_lock:
        _robot_pose.x     = x
        _robot_pose.y     = y
        _robot_pose.theta = theta


def get_opponent() -> Optional[Tuple[float, float, float]]:
    """Return (x_mm, y_mm, confidence) in field coordinates, or None."""
    with _opponent_lock:
        if _opponent.confidence < 0.1 or _opponent.timestamp == 0.0:
            return None
        if time.time() - _opponent.timestamp > STALE_TIMEOUT_S:
            return None
        return (_opponent.x, _opponent.y, _opponent.confidence)


def get_status() -> Tuple[bool, str]:
    """Return (connected, last_error) of the acquisition thread."""
    with _status_lock:
        return _status.connected, _status.last_error


def start(timeout_s: float = CONNECT_TIMEOUT_S) -> bool:
    """Start the acquisition thread and wait for the LiDAR to answer.

    Returns True once the LiDAR is connected. A failure is logged as an error
    and returned to the caller: an acquisition thread that dies on its own must
    never look like a working detection.
    """
    global _running, _thread
    if _thread and _thread.is_alive():
        logger.warning("Thread LiDAR deja actif.")
        return get_status()[0]

    with _status_lock:
        _status.connected  = False
        _status.last_error = ""

    _running = True
    _thread = threading.Thread(target=_lidar_loop, daemon=True, name="LidarDetect")
    _thread.start()

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        connected, error = get_status()
        if connected:
            logger.info("Thread LiDAR demarre.")
            return True
        if error or not _thread.is_alive():
            break
        time.sleep(0.05)

    _, error = get_status()
    logger.error(
        "LiDAR indisponible sur %s : %s",
        PORT,
        error or f"aucune reponse en {timeout_s:.1f} s",
    )
    return False


def stop() -> None:
    """Stop the acquisition thread cleanly (waits up to 3 s)."""
    global _running
    _running = False
    if _thread:
        _thread.join(timeout=3.0)
    logger.info("Thread LiDAR arrete.")

# ── SCAN PROCESSING ───────────────────────────────────────────────────────────

def _process_scan(raw_scan) -> None:
    """Process one full scan: filter, project to field, cluster, track."""

    # 1. Quality and distance filter, then projection into the robot frame
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

    # 2. Conversion to field coordinates
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

    # 3. Angular clustering
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

    # 4. Best cluster: the closest one with enough points.
    #    KNOWN LIMITATION: nothing tells a wall, a beacon mast or a fixed
    #    obstacle apart from the opponent. Whatever stands closest within
    #    DETECT_DIST_MM wins. Only DETECT_DIST_MM and the field bounding box
    #    keep it honest today. To be validated in match conditions before the
    #    rewrite, see doc_ref/TODO.md section 6.
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

    # 5. Update with smoothing and gating
    now = time.time()
    with _opponent_lock:
        if best is None:
            _opponent.missed += 1
            if _opponent.missed > MAX_MISSED_SCANS:
                _opponent.confidence = 0.0
            return

        cx, cy, conf, npts = best

        # Gating: reject physically impossible jumps
        if _opponent.confidence > 0.1 and _opponent.missed < MAX_MISSED_SCANS:
            dt = max(now - _opponent.timestamp, 0.05)
            max_jump = MAX_OPP_SPEED_MM_S * dt
            jump = math.hypot(cx - _opponent.x, cy - _opponent.y)
            if jump > max_jump:
                logger.debug(f"GATING saut={jump:.0f}mm > max={max_jump:.0f}mm -> rejete")
                _opponent.missed += 1
                return
            # Exponential smoothing
            _opponent.x = ALPHA_SMOOTH * cx + (1 - ALPHA_SMOOTH) * _opponent.x
            _opponent.y = ALPHA_SMOOTH * cy + (1 - ALPHA_SMOOTH) * _opponent.y
        else:
            # New detection: no smoothing
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
    """Increment the miss counter."""
    with _opponent_lock:
        _opponent.missed += 1
        if _opponent.missed > MAX_MISSED_SCANS:
            _opponent.confidence = 0.0

# ── ACQUISITION THREAD ────────────────────────────────────────────────────────

def _set_status(connected: bool, error: str = "") -> None:
    """Publish the state of the acquisition thread for get_status()."""
    with _status_lock:
        _status.connected = connected
        if error:
            _status.last_error = error


def _lidar_loop() -> None:
    """Acquisition loop. Runs in a daemon thread."""
    lidar = None
    try:
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        lidar._serial.flushInput()
        time.sleep(0.5)
        _set_status(True)
        logger.info(f"LiDAR connecte sur {PORT}.")

        try:
            scan_iter = lidar.iter_scans(max_buf_meas=1500, min_len=20)
        except TypeError:
            scan_iter = lidar.iter_scans()

        for scan in scan_iter:
            if not _running:
                break
            _process_scan(scan)

    except Exception as exc:
        _set_status(False, str(exc))
        logger.error(f"Erreur LiDAR sur {PORT} : {exc}")
    finally:
        _set_status(False)
        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
        logger.info("Thread LiDAR termine.")
