import importlib
import itertools
import logging
import math
import os
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from rplidar import RPLidar

# Source unique: dimensions du terrain + layout des balises via loader.
try:
    from ..loader import loader
except ImportError:
    ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if ROOT_DIR not in sys.path:
        sys.path.insert(0, ROOT_DIR)
    from loader import loader

try:
    BeaconLayout = loader.load_class('terrain', 'BeaconLayout')
    _terrain_module = importlib.import_module(BeaconLayout.__module__)
except Exception:
    ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if ROOT_DIR not in sys.path:
        sys.path.insert(0, ROOT_DIR)
    import terrain_jeu as _terrain_module
    BeaconLayout = _terrain_module.BeaconLayout

FIELD_WIDTH_MM  = float(getattr(_terrain_module, 'FIELD_WIDTH_MM',  3000.0))
FIELD_HEIGHT_MM = float(getattr(_terrain_module, 'FIELD_HEIGHT_MM', 2000.0))

logger = logging.getLogger("LIDAR")


def _build_display_beacons(
    beacons_by_id: Dict[int, Tuple[float, float]]
) -> Dict[str, Tuple[float, float]]:
    """Construit un mapping A/B/C... pour l'UI et la couche fusion."""
    labels  = ("A", "B", "C", "D", "E")
    mapping: Dict[str, Tuple[float, float]] = {}
    for idx, beacon_id in enumerate(sorted(beacons_by_id)):
        label      = labels[idx] if idx < len(labels) else f"B{beacon_id}"
        bx, by     = beacons_by_id[beacon_id]
        mapping[label] = (float(bx), float(by))
    return mapping


# ── VALIDATION BeaconLayout ───────────────────────────────────────────────────
# Pas de fallback silencieux : si BeaconLayout.BEACONS est absent ou vide,
# on lève une erreur explicite dès le chargement du module.
if not hasattr(BeaconLayout, 'BEACON_SIZE_MM'):
    raise RuntimeError(
        "BeaconLayout.BEACON_SIZE_MM non défini dans terrain_jeu.py. "
        "Vérifier que la classe BeaconLayout est correctement configurée."
    )
if not hasattr(BeaconLayout, 'BEACONS') or not BeaconLayout.BEACONS:
    raise RuntimeError(
        "BeaconLayout.BEACONS est absent ou vide dans terrain_jeu.py. "
        "Définir les positions des balises avant de démarrer le LiDAR."
    )

BEACON_SIZE_MM = float(BeaconLayout.BEACON_SIZE_MM)


# ── CONFIG RUNTIME LIDAR ──────────────────────────────────────────────────────
PORT      = '/dev/ttyUSB0'
BAUDRATE  = 256000
TIMEOUT   = 3
MIN_DIST  = 50
MAX_DIST  = 12000
MIN_QUAL  = 2

SCAN_MAX_BUF_MEAS   = 1500
SCAN_MIN_LEN        = 20
SCAN_HISTORY_LEN    = 2
MERGE_ANGLE_BIN_DEG = 0.5
MAX_MERGED_POINTS   = 1200

MAP_W_MM = int(FIELD_WIDTH_MM)
MAP_H_MM = int(FIELD_HEIGHT_MM)


# ── ÉTAT BALISES (modifiable via set_team_color) ─────────────────────────────
# Initialisé en BLEU par défaut ; Robot.__init__() appelle set_team_color()
# AVANT start_lidar_thread() pour charger les bonnes coordonnées.
_team_color: str = "BLUE"

BEACONS_BY_ID: Dict[int, Tuple[float, float]] = {}
_beacon_xs:    List[float] = []
_beacon_ys:    List[float] = []

BEACON_OUTSIDE_LEFT_MM:   float = 0.0
BEACON_OUTSIDE_RIGHT_MM:  float = 0.0
BEACON_OUTSIDE_BOTTOM_MM: float = 0.0
BEACON_OUTSIDE_TOP_MM:    float = 0.0
OFFSET_BALISE_MM:         float = 0.0
MAP_VIEW_MARGIN_MM:       float = 150.0
POSE_MAX_DIST_MM:         int   = 4000

BEACONS_TEST:      Dict[str, Tuple[float, float]] = {}
_THEO_DISTS:       Dict[Tuple[int, int], float]   = {}
_beacon_ids_list:  List[int]                       = []


def _recompute_beacon_globals() -> None:
    """Recalcule toutes les constantes dérivées de BEACONS_BY_ID."""
    global _beacon_xs, _beacon_ys
    global BEACON_OUTSIDE_LEFT_MM, BEACON_OUTSIDE_RIGHT_MM
    global BEACON_OUTSIDE_BOTTOM_MM, BEACON_OUTSIDE_TOP_MM
    global OFFSET_BALISE_MM, MAP_VIEW_MARGIN_MM, POSE_MAX_DIST_MM
    global BEACONS_TEST, _THEO_DISTS, _beacon_ids_list

    _beacon_xs = [bx for bx, _ in BEACONS_BY_ID.values()]
    _beacon_ys = [by for _, by in BEACONS_BY_ID.values()]

    BEACON_OUTSIDE_LEFT_MM   = max(0.0, -min(_beacon_xs))
    BEACON_OUTSIDE_RIGHT_MM  = max(0.0,  max(_beacon_xs) - float(MAP_W_MM))
    BEACON_OUTSIDE_BOTTOM_MM = max(0.0, -min(_beacon_ys))
    BEACON_OUTSIDE_TOP_MM    = max(0.0,  max(_beacon_ys) - float(MAP_H_MM))

    OFFSET_BALISE_MM = max(
        BEACON_OUTSIDE_LEFT_MM, BEACON_OUTSIDE_RIGHT_MM,
        BEACON_OUTSIDE_BOTTOM_MM, BEACON_OUTSIDE_TOP_MM,
    )
    MAP_VIEW_MARGIN_MM = max(
        150.0, OFFSET_BALISE_MM + (BEACON_SIZE_MM * 0.5) + 20.0
    )
    POSE_MAX_DIST_MM = int(
        math.hypot(
            MAP_W_MM + BEACON_OUTSIDE_LEFT_MM + BEACON_OUTSIDE_RIGHT_MM,
            MAP_H_MM + BEACON_OUTSIDE_BOTTOM_MM + BEACON_OUTSIDE_TOP_MM,
        ) + 200.0
    )

    BEACONS_TEST     = _build_display_beacons(BEACONS_BY_ID)
    _beacon_ids_list = sorted(BEACONS_BY_ID.keys())

    _THEO_DISTS = {}
    for i, bid1 in enumerate(_beacon_ids_list):
        for bid2 in _beacon_ids_list[i + 1:]:
            bx1, by1 = BEACONS_BY_ID[bid1]
            bx2, by2 = BEACONS_BY_ID[bid2]
            _THEO_DISTS[(min(bid1, bid2), max(bid1, bid2))] = math.hypot(
                bx1 - bx2, by1 - by2
            )


def set_team_color(color: str) -> None:
    """
    Charge les positions des balises pour la bonne couleur d'équipe.

    Doit être appelé depuis Robot.__init__() AVANT start_lidar_thread().
    Utilise BeaconLayout.get_beacon() qui gère déjà la symétrie.

    Args:
        color: "BLUE" ou "YELLOW"
    """
    global _team_color, BEACONS_BY_ID

    _team_color = color.upper()

    # Utiliser BeaconLayout.get_beacon() qui gère la symétrie YELLOW
    BEACONS_BY_ID = {
        bid: BeaconLayout.get_beacon(bid, _team_color)
        for bid in BeaconLayout.BEACONS.keys()
    }

    _recompute_beacon_globals()

    logger.info(
        f"[LIDAR] Balises chargées pour équipe {_team_color}: {BEACONS_BY_ID}"
    )


# Charger BLEU par défaut au démarrage du module
set_team_color("BLUE")


# ── CONSTANTES LOCALISATION ───────────────────────────────────────────────────
POSE_MIN_DIST_MM = 120

CLUSTER_GAP_MM     = max(120.0, 1.2 * BEACON_SIZE_MM)
CLUSTER_MIN_POINTS = 4
TRACK_MATCH_MM     = 220
TRACK_MAX_MISSED   = 8
TRACK_HISTORY_LEN  = 20

AUTO_BEACON_LOCALIZATION        = True
BEACON_QUAL_MIN                 = 2
BEACON_ANG_GAP_RAD              = math.radians(2.5)
BEACON_DIST_GAP_MM              = max(40.0, 0.90 * BEACON_SIZE_MM)
BEACON_MIN_RETURNS_PER_CLUSTER  = 2
BEACON_MAX_CANDIDATES           = 3
BEACON_FIT_MAX_RMS_MM           = 80.0
BEACON_GEOM_TOL_MM              = 70.0
BEACON_EDGE_DROP_POINTS         = 1
BEACON_HALF_DEPTH_MM            = 0.5 * BEACON_SIZE_MM
BEACON_FACE_MIN_LEN_MM          = max(25.0, 0.65 * BEACON_SIZE_MM)
BEACON_FACE_MAX_LEN_MM          = 1.05 * BEACON_SIZE_MM
AUTO_POSE_MIN_CONFIDENCE        = 0.20
TWO_BEACON_BASE_CONFIDENCE      = 0.50
POSE_CONTINUITY_RMS_TOL_MM      = 28.0
POSE_BLEND_ALPHA_XY             = 0.35
POSE_BLEND_ALPHA_THETA          = 0.30
POSE_DEADBAND_MM                = 15.0
POSE_DEADBAND_DEG               = 1.5
MIN_POINTS_FOR_POSE             = 120
POSE_MAX_JUMP_MM                = 260.0
POSE_MAX_JUMP_DEG               = 28.0
POSE_MAX_MISSES_BEFORE_UNLOCK   = 10
FUSION_MIN_CONFIDENCE           = 0.14

# ── CORRECTION D'ODOMÉTRIE ────────────────────────────────────────────────────
BEACON_WINDOW_ANGLE_RAD         = math.radians(10.0)
BEACON_WINDOW_DIST_MM           = 200.0
POSE_CORRECTION_MIN_CONFIDENCE  = 0.60
POSE_CORRECTION_MIN_BEACONS     = 2
POSE_SEND_BACK_INTERVAL_S       = 1.0

# ── DÉTECTION ADVERSAIRE ──────────────────────────────────────────────────────
ROBOT_MIN_RADIUS_MM          = 60.0
ROBOT_MAX_RADIUS_MM          = 220.0
OPPONENT_BEACON_EXCLUSION_MM = max(150.0, BEACON_SIZE_MM + 50.0)
OPPONENT_MAX_MISSED          = 12

# ── CLASSIFICATION CLUSTERS ───────────────────────────────────────────────────
LINEARITY_THRESHOLD   = 2.0
CIRCULARITY_THRESHOLD = 0.15

__all__ = [
    'PORT', 'BAUDRATE', 'TIMEOUT', 'MIN_DIST', 'MAX_DIST', 'MIN_QUAL',
    'SCAN_MAX_BUF_MEAS', 'SCAN_MIN_LEN', 'SCAN_HISTORY_LEN',
    'MERGE_ANGLE_BIN_DEG', 'MAX_MERGED_POINTS',
    'MAP_W_MM', 'MAP_H_MM', 'OFFSET_BALISE_MM', 'MAP_VIEW_MARGIN_MM',
    'BEACON_SIZE_MM', 'BEACONS_BY_ID',
    'POSE_MIN_DIST_MM', 'POSE_MAX_DIST_MM',
    'CLUSTER_GAP_MM', 'CLUSTER_MIN_POINTS', 'TRACK_MATCH_MM',
    'TRACK_MAX_MISSED', 'TRACK_HISTORY_LEN',
    'BEACONS_TEST',
    'AUTO_BEACON_LOCALIZATION', 'BEACON_QUAL_MIN', 'BEACON_ANG_GAP_RAD',
    'BEACON_DIST_GAP_MM', 'BEACON_MIN_RETURNS_PER_CLUSTER',
    'BEACON_MAX_CANDIDATES', 'BEACON_FIT_MAX_RMS_MM', 'BEACON_GEOM_TOL_MM',
    'BEACON_EDGE_DROP_POINTS', 'BEACON_HALF_DEPTH_MM',
    'BEACON_FACE_MIN_LEN_MM', 'BEACON_FACE_MAX_LEN_MM',
    'AUTO_POSE_MIN_CONFIDENCE', 'TWO_BEACON_BASE_CONFIDENCE',
    'POSE_CONTINUITY_RMS_TOL_MM', 'POSE_BLEND_ALPHA_XY',
    'POSE_BLEND_ALPHA_THETA', 'POSE_DEADBAND_MM', 'POSE_DEADBAND_DEG',
    'MIN_POINTS_FOR_POSE', 'POSE_MAX_JUMP_MM', 'POSE_MAX_JUMP_DEG',
    'POSE_MAX_MISSES_BEFORE_UNLOCK', 'FUSION_MIN_CONFIDENCE',
    'ROBOT_MIN_RADIUS_MM', 'ROBOT_MAX_RADIUS_MM',
    'OPPONENT_BEACON_EXCLUSION_MM', 'OPPONENT_MAX_MISSED',
    'PoseState', 'OpponentState',
    'set_team_color',
    'get_latest_scan_data', 'get_latest_beacon_candidates',
    'get_latest_opponent', 'get_corrected_pose', 'get_latest_pose',
    'update_teensy_pose', 'should_send_correction_to_teensy',
    'stop_lidar_runtime', 'start_lidar_thread',
]


# ── STRUCTURES DE DONNÉES ─────────────────────────────────────────────────────

@dataclass
class PoseState:
    """État de pose du robot estimée depuis LiDAR."""
    x: float                        = 0.0
    y: float                        = 0.0
    theta: float                    = 0.0
    confidence: float               = 0.0
    beacon_ids: Optional[List[str]] = None
    is_localized: bool              = False
    last_update_time: float         = 0.0


@dataclass
class OpponentState:
    """État du robot adverse détecté."""
    x: float             = 0.0
    y: float             = 0.0
    confidence: float    = 0.0
    missed_count: int    = 0
    last_update_time: float = 0.0


# ── ÉTAT PARTAGÉ ADVERSAIRE ───────────────────────────────────────────────────
_opponent_state = OpponentState()
_opponent_lock  = threading.Lock()


def _update_opponent(candidate: Optional[Dict]) -> None:
    """Met à jour l'état du robot adverse détecté."""
    global _opponent_state
    with _opponent_lock:
        if candidate:
            _opponent_state = OpponentState(
                x=float(candidate.get('x', 0.0)),
                y=float(candidate.get('y', 0.0)),
                confidence=float(candidate.get('confidence', 0.0)),
                missed_count=0,
                last_update_time=time.time(),
            )
        else:
            _opponent_state.missed_count += 1
            if _opponent_state.missed_count > OPPONENT_MAX_MISSED:
                _opponent_state.confidence = 0.0


# ── DÉTECTION ADVERSAIRE (O(N) clustering angulaire) ─────────────────────────

def _detect_opponent_fast(
    merged_data: List[Tuple],
    beacon_candidates: List[Dict],
    pose_estimate: Optional[PoseState] = None,
) -> Optional[Dict]:
    """
    Détecte le robot adverse via clustering angulaire O(N).

    Filtres :
      1. Taille cluster : rayon ∈ [ROBOT_MIN_RADIUS_MM, ROBOT_MAX_RADIUS_MM]
      2. Exclusion balises : distance > OPPONENT_BEACON_EXCLUSION_MM
      3. Contrainte terrain : x ∈ [-50, MAP_W+50], y ∈ [-50, MAP_H+50]
      4. Linéarité PCA : rejeter si trop linéaire (= balise)
    """
    if not merged_data or len(merged_data) < 3:
        return None

    try:
        arr    = np.asarray(merged_data, dtype=np.float32)
        angles = arr[:, 0]
        dists  = arr[:, 1]

        xs = dists * np.cos(angles)
        ys = dists * np.sin(angles)

        order  = np.argsort(angles)
        xs_s   = xs[order]
        ys_s   = ys[order]

        gap    = np.hypot(np.diff(xs_s), np.diff(ys_s))
        breaks = np.where(gap > CLUSTER_GAP_MM)[0] + 1

        splits_x = np.split(xs_s, breaks)
        splits_y = np.split(ys_s, breaks)

        beacon_pos = (
            np.array([[c['x_r'], c['y_r']] for c in beacon_candidates],
                     dtype=np.float32)
            if beacon_candidates else np.empty((0, 2), dtype=np.float32)
        )

        candidates = []

        for sx, sy in zip(splits_x, splits_y):
            if len(sx) < CLUSTER_MIN_POINTS:
                continue

            pts      = np.column_stack([sx, sy])
            centroid = pts.mean(axis=0)
            radii    = np.linalg.norm(pts - centroid, axis=1)
            radius   = radii.max()

            if not (ROBOT_MIN_RADIUS_MM <= radius <= ROBOT_MAX_RADIUS_MM):
                continue

            if len(beacon_pos) > 0:
                if np.linalg.norm(beacon_pos - centroid, axis=1).min() \
                        < OPPONENT_BEACON_EXCLUSION_MM:
                    continue

            x, y = float(centroid[0]), float(centroid[1])
            if not (-50 <= x <= MAP_W_MM + 50 and -50 <= y <= MAP_H_MM + 50):
                continue

            pts_c     = pts - centroid
            cov       = (pts_c.T @ pts_c) / len(pts)
            s         = np.linalg.svd(cov, compute_uv=False)
            linearity = s[0] / (s[1] + 1e-9)

            if linearity > LINEARITY_THRESHOLD:
                continue

            cv         = np.std(radii) / (np.mean(radii) + 1e-9)
            confidence = 0.6 + 0.3 * float(cv < CIRCULARITY_THRESHOLD)

            candidates.append({
                'x':              x,
                'y':              y,
                'confidence':     float(confidence),
                'points_count':   len(sx),
                'cluster_radius': float(radius),
            })

        if not candidates:
            return None
        return max(candidates, key=lambda c: c['confidence'])

    except Exception as e:
        logger.debug(f"Opponent detection error: {e}")
        return None


# Alias déprécié
def _detect_opponent(
    merged_data: List[Tuple],
    beacon_candidates: List[Dict],
    pose_estimate: Optional[PoseState] = None,
) -> Optional[Dict]:
    """DEPRECATED — utiliser _detect_opponent_fast()."""
    return _detect_opponent_fast(merged_data, beacon_candidates, pose_estimate)


# ── ÉTAT PARTAGÉ ACQUISITION ──────────────────────────────────────────────────
scan_read_buf  = []
scan_write_buf = []
scan_frames    = deque(maxlen=SCAN_HISTORY_LEN)
lock           = threading.Lock()
running        = True
lidar_obj      = None

beacon_candidates_buf = []
beacon_lock           = threading.Lock()

_teensy_pose      = (None, None, None)
_teensy_pose_lock = threading.Lock()

_corrected_pose      = PoseState()
_corrected_pose_lock = threading.Lock()
_last_correction_time = 0.0


# ── UTILITAIRES ───────────────────────────────────────────────────────────────

def _angle_diff(a1: float, a2: float) -> float:
    """Différence angulaire minimale avec wrap-around [0, π]."""
    return abs((a1 - a2 + math.pi) % (2 * math.pi) - math.pi)


# ── ASSOCIATION CANDIDATS → BALISES ──────────────────────────────────────────

def _hungarian_assign(
    candidates: List[Dict],
    beacon_ids: List[int],
    theo_dists: Dict,
    cand_dists: Dict,
) -> Dict[int, int]:
    """
    Association optimale candidats→balises par comparaison de distances.
    Max 3! = 6 permutations pour 3 balises.
    """
    k          = min(len(candidates), len(beacon_ids))
    best_cost  = float('inf')
    best_assign: Dict[int, int] = {}

    for beacon_perm in itertools.permutations(beacon_ids, k):
        cost = 0.0
        for i in range(k):
            for j in range(i + 1, k):
                key    = (min(beacon_perm[i], beacon_perm[j]),
                          max(beacon_perm[i], beacon_perm[j]))
                theo_d = theo_dists.get(key)
                if theo_d is None:
                    continue
                meas_d = cand_dists.get((i, j), 0.0)
                cost  += abs(theo_d - meas_d)

        if cost < best_cost:
            best_cost   = cost
            best_assign = {i: beacon_perm[i] for i in range(k)}

    if best_cost > BEACON_GEOM_TOL_MM * k:
        return {}

    return best_assign


def _associate_candidates_to_beacons(
    candidates: List[Dict],
    teensy_x: float,
    teensy_y: float,
    teensy_theta: float,
    windows: Optional[Dict] = None,
) -> List[Dict]:
    """
    Associe chaque candidat-balise à une balise connue (ajoute 'beacon_id').

    Mode 1 — fenêtres prédites (pose Teensy connue).
    Mode 2 — hungarian sur distances inter-candidats (démarrage).
    """
    if not candidates:
        return []

    enriched: List[Dict] = []
    used_beacon_ids: set = set()

    if windows:
        for cand in candidates:
            angle_c    = cand['angle']
            dist_c     = cand['distance']
            best_bid   = None
            best_score = float('inf')

            for bid, win in windows.items():
                if bid in used_beacon_ids:
                    continue
                a_diff = _angle_diff(angle_c, win['angle_pred'])
                d_diff = abs(dist_c - win['dist_pred'])

                if (a_diff <= BEACON_WINDOW_ANGLE_RAD and
                        d_diff <= BEACON_WINDOW_DIST_MM):
                    score = (a_diff / BEACON_WINDOW_ANGLE_RAD +
                             d_diff / BEACON_WINDOW_DIST_MM)
                    if score < best_score:
                        best_score = score
                        best_bid   = bid

            if best_bid is not None:
                c = dict(cand)
                c['beacon_id']         = best_bid
                c['association_score'] = best_score
                enriched.append(c)
                used_beacon_ids.add(best_bid)

    else:
        cand_dists: Dict = {}
        for i in range(len(candidates)):
            for j in range(i + 1, len(candidates)):
                xi, yi = candidates[i]['x_r'], candidates[i]['y_r']
                xj, yj = candidates[j]['x_r'], candidates[j]['y_r']
                cand_dists[(i, j)] = math.hypot(xi - xj, yi - yj)

        assignment = _hungarian_assign(
            candidates, _beacon_ids_list, _THEO_DISTS, cand_dists
        )

        for cand_idx, bid in assignment.items():
            c = dict(candidates[cand_idx])
            c['beacon_id'] = bid
            enriched.append(c)

    return enriched


def _validate_beacon_geometry(associated_cands: List[Dict]) -> bool:
    """
    Vérifie la cohérence géométrique avant le SVD.
    Retourne False si bid_i est None ou si la distance mesurée
    dépasse BEACON_GEOM_TOL_MM par rapport à la distance théorique.
    """
    if len(associated_cands) < 2:
        return False

    for i in range(len(associated_cands)):
        for j in range(i + 1, len(associated_cands)):
            ci    = associated_cands[i]
            cj    = associated_cands[j]
            bid_i = ci.get('beacon_id')
            bid_j = cj.get('beacon_id')

            if bid_i is None or bid_j is None:
                return False
            if bid_i not in BEACONS_BY_ID or bid_j not in BEACONS_BY_ID:
                return False

            meas_dist = math.hypot(
                ci['x_r'] - cj['x_r'],
                ci['y_r'] - cj['y_r'],
            )
            key       = (min(bid_i, bid_j), max(bid_i, bid_j))
            theo_dist = _THEO_DISTS.get(key, 0.0)

            if abs(meas_dist - theo_dist) > BEACON_GEOM_TOL_MM:
                logger.debug(
                    f"Géométrie invalide {bid_i}-{bid_j}: "
                    f"mesuré={meas_dist:.0f}mm théorique={theo_dist:.0f}mm"
                )
                return False

    return True


# ── CORRECTION SVD UMEYAMA ────────────────────────────────────────────────────

def _compute_corrected_pose(
    beacon_candidates: List[Dict],
    teensy_x: float,
    teensy_y: float,
    teensy_theta: float,
) -> Optional[PoseState]:
    """
    Corrige la pose Teensy via SVD Umeyama 2D.
    Requiert que chaque candidat ait un champ 'beacon_id'.
    """
    if len(beacon_candidates) < POSE_CORRECTION_MIN_BEACONS:
        return None

    cos_theta = math.cos(teensy_theta)
    sin_theta = math.sin(teensy_theta)

    measured_points:    List[List[float]] = []
    theoretical_points: List[List[float]] = []
    beacon_ids_used:    List[int]         = []

    for cand in beacon_candidates:
        if 'beacon_id' not in cand:
            continue
        bid = int(cand['beacon_id'])
        if bid not in BEACONS_BY_ID:
            continue

        x_lidar = cand.get('x_r', 0.0)
        y_lidar = cand.get('y_r', 0.0)

        x_measured = teensy_x + (x_lidar * cos_theta - y_lidar * sin_theta)
        y_measured = teensy_y + (x_lidar * sin_theta + y_lidar * cos_theta)

        measured_points.append([x_measured, y_measured])
        theoretical_points.append(list(BEACONS_BY_ID[bid]))
        beacon_ids_used.append(bid)

    if len(measured_points) < POSE_CORRECTION_MIN_BEACONS:
        return None

    measured    = np.array(measured_points,    dtype=np.float64)
    theoretical = np.array(theoretical_points, dtype=np.float64)

    centroid_m = measured.mean(axis=0)
    centroid_t = theoretical.mean(axis=0)

    measured_c    = measured    - centroid_m
    theoretical_c = theoretical - centroid_t

    H        = measured_c.T @ theoretical_c
    U, S, Vt = np.linalg.svd(H)
    R        = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    delta_theta = float(np.arctan2(R[1, 0], R[0, 0]))
    t           = centroid_t - R @ centroid_m
    delta_x     = float(t[0])
    delta_y     = float(t[1])

    if abs(delta_theta) > math.radians(POSE_MAX_JUMP_DEG):
        logger.debug(f"SVD delta_theta trop grand: {math.degrees(delta_theta):.1f}°")
        return None

    if math.hypot(delta_x, delta_y) > POSE_MAX_JUMP_MM:
        logger.debug(f"SVD delta XY trop grand: {math.hypot(delta_x, delta_y):.0f}mm")
        return None

    corrected_measured = (measured @ R.T) + t
    residuals          = corrected_measured - theoretical
    rms                = float(np.sqrt(np.mean(residuals ** 2)))

    if rms > BEACON_FIT_MAX_RMS_MM:
        return None

    confidence = (
        max(0.7, 1.0 - rms / 100.0) if len(beacon_ids_used) >= 3
        else max(0.5, 0.8 - rms / 100.0)
    )

    x_corrected     = teensy_x + delta_x
    y_corrected     = teensy_y + delta_y
    theta_corrected = (teensy_theta + delta_theta + math.pi) % (2 * math.pi) - math.pi

    return PoseState(
        x=float(x_corrected),
        y=float(y_corrected),
        theta=float(theta_corrected),
        confidence=float(confidence),
        beacon_ids=beacon_ids_used,
        is_localized=confidence >= POSE_CORRECTION_MIN_CONFIDENCE,
        last_update_time=time.time(),
    )


# ── PRÉDICTION FENÊTRES BALISES ───────────────────────────────────────────────

def _predict_beacon_windows(
    teensy_x: float,
    teensy_y: float,
    teensy_theta: float,
) -> Optional[Dict]:
    """Calcule pour chaque balise sa fenêtre de prédiction dans le scan."""
    if teensy_x is None or teensy_y is None or teensy_theta is None:
        return None

    cos_theta = math.cos(teensy_theta)
    sin_theta = math.sin(teensy_theta)
    windows: Dict = {}

    for beacon_id, (bx_world, by_world) in BEACONS_BY_ID.items():
        dx_world = bx_world - teensy_x
        dy_world = by_world - teensy_y

        dx_robot =  dx_world * cos_theta + dy_world * sin_theta
        dy_robot = -dx_world * sin_theta + dy_world * cos_theta

        dist_pred  = math.hypot(dx_robot, dy_robot)
        angle_pred = math.atan2(dy_robot, dx_robot)

        if dist_pred < POSE_MIN_DIST_MM or dist_pred > POSE_MAX_DIST_MM:
            continue

        windows[beacon_id] = {
            'angle_pred': float(angle_pred),
            'dist_pred':  float(dist_pred),
            'angle_min':  float(angle_pred - BEACON_WINDOW_ANGLE_RAD),
            'angle_max':  float(angle_pred + BEACON_WINDOW_ANGLE_RAD),
            'dist_min':   float(max(POSE_MIN_DIST_MM, dist_pred - BEACON_WINDOW_DIST_MM)),
            'dist_max':   float(min(POSE_MAX_DIST_MM, dist_pred + BEACON_WINDOW_DIST_MM)),
        }

    return windows if windows else None


# ── EXTRACTION CANDIDATS BALISES ──────────────────────────────────────────────

def _extract_beacon_candidates_fast(points) -> List[Dict]:
    """
    Extrait les candidats-balises depuis un scan fusionné.
    Pipeline : distance → qualité → clustering angulaire → validation face.
    """
    if not points:
        return []

    arr = np.asarray(points, dtype=np.float32)

    d_mask = (arr[:, 1] >= POSE_MIN_DIST_MM) & (arr[:, 1] <= POSE_MAX_DIST_MM)
    arr    = arr[d_mask]
    if len(arr) < BEACON_MIN_RETURNS_PER_CLUSTER:
        return []

    q_mask = arr[:, 2] >= BEACON_QUAL_MIN
    arr_hq = arr[q_mask]
    if len(arr_hq) < BEACON_MIN_RETURNS_PER_CLUSTER:
        return []

    order  = np.argsort(arr_hq[:, 0])
    arr_hq = arr_hq[order]

    angles = arr_hq[:, 0]
    dists  = arr_hq[:, 1]
    quals  = arr_hq[:, 2]

    d_angles = np.abs(np.diff(angles))
    d_dists  = np.abs(np.diff(dists))
    breaks   = np.where(
        (d_angles > BEACON_ANG_GAP_RAD) | (d_dists > BEACON_DIST_GAP_MM)
    )[0] + 1

    idx_splits = np.split(np.arange(len(angles)), breaks)

    if len(idx_splits) >= 2:
        first_idx = idx_splits[0][0]
        last_idx  = idx_splits[-1][-1]
        wrap_ang  = (angles[first_idx] + 2.0 * math.pi) - angles[last_idx]
        wrap_dist = abs(dists[first_idx] - dists[last_idx])
        both_ok   = (
            len(idx_splits[0])  >= BEACON_MIN_RETURNS_PER_CLUSTER and
            len(idx_splits[-1]) >= BEACON_MIN_RETURNS_PER_CLUSTER
        )
        if wrap_ang <= BEACON_ANG_GAP_RAD and wrap_dist <= BEACON_DIST_GAP_MM and both_ok:
            merged_idx = np.concatenate([idx_splits[-1], idx_splits[0]])
            idx_splits = [merged_idx] + idx_splits[1:-1]

    candidates: List[Dict] = []

    for idxs in idx_splits:
        if len(idxs) < BEACON_MIN_RETURNS_PER_CLUSTER:
            continue

        a_cl = angles[idxs]
        d_cl = dists[idxs]
        q_cl = quals[idxs]

        drop = BEACON_EDGE_DROP_POINTS
        if len(a_cl) > 2 * drop + 1:
            a_cl = a_cl[drop:-drop]
            d_cl = d_cl[drop:-drop]
            q_cl = q_cl[drop:-drop]

        if len(a_cl) < 2:
            continue

        mean_d   = float(np.mean(d_cl))
        ang_span = float(a_cl[-1] - a_cl[0])
        face_est = mean_d * math.tan(ang_span / 2.0) * 2.0

        if face_est < BEACON_FACE_MIN_LEN_MM or face_est > BEACON_FACE_MAX_LEN_MM:
            continue

        x_r = float(np.mean(d_cl * np.sin(a_cl)))
        y_r = float(np.mean(d_cl * np.cos(a_cl)))

        dist_ctr = math.hypot(x_r, y_r)
        if dist_ctr > 1e-6:
            x_r += BEACON_HALF_DEPTH_MM * x_r / dist_ctr
            y_r += BEACON_HALF_DEPTH_MM * y_r / dist_ctr

        angle_ctr = float(math.atan2(x_r, y_r) % (2.0 * math.pi))
        dist_ctr  = float(math.hypot(x_r, y_r))

        candidates.append({
            "angle":    angle_ctr,
            "distance": dist_ctr,
            "quality":  float(np.mean(q_cl)),
            "count":    int(len(a_cl)),
            "face_est": float(face_est),
            "x_r":      float(x_r),
            "y_r":      float(y_r),
        })

    candidates.sort(key=lambda c: (c["quality"], c["count"]), reverse=True)
    return candidates[:BEACON_MAX_CANDIDATES]


# ── FUSION SCANS ──────────────────────────────────────────────────────────────

def _merge_scan_frames(frames) -> List[Tuple[float, float, float]]:
    """Fusion vectorisée de scans successifs par bins angulaires."""
    if not frames:
        return []

    arrays = [np.asarray(pts, dtype=np.float32) for pts in frames if pts]
    if not arrays:
        return []

    all_pts = np.vstack(arrays)
    if all_pts.size == 0:
        return []

    rads  = all_pts[:, 0]
    dists = all_pts[:, 1]
    quals = all_pts[:, 2]

    bin_rad = math.radians(max(0.1, float(MERGE_ANGLE_BIN_DEG)))
    bins    = np.floor(np.mod(rads, 2.0 * math.pi) / bin_rad).astype(np.int32)

    order  = np.lexsort((dists, -quals, bins))
    bins_s = bins[order]
    r_s    = rads[order]
    d_s    = dists[order]
    q_s    = quals[order]

    _, first_idx = np.unique(bins_s, return_index=True)
    merged       = np.column_stack((r_s[first_idx], d_s[first_idx], q_s[first_idx]))
    merged       = merged[np.argsort(merged[:, 0])]

    if MAX_MERGED_POINTS > 0 and len(merged) > MAX_MERGED_POINTS:
        stride = max(1, len(merged) // MAX_MERGED_POINTS)
        merged = merged[::stride][:MAX_MERGED_POINTS]

    return [
        (float(r), float(d), float(q))
        for r, d, q in merged
    ]


# ── THREAD PRINCIPAL D'ACQUISITION ───────────────────────────────────────────

def lidar_thread(console_callback, status_callback):
    global lidar_obj, running, scan_read_buf, scan_write_buf, beacon_candidates_buf

    try:
        lidar_obj = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        lidar_obj._serial.flushInput()
        time.sleep(1)
        status_callback('Connecte - scan en cours')
        console_callback(f'[OK] Connecté sur {PORT} à {BAUDRATE} baud\n')
        console_callback(f'[INFO] Équipe: {_team_color} | Balises: {list(BEACONS_BY_ID.keys())}\n')

        try:
            scan_iter = lidar_obj.iter_scans(
                max_buf_meas=SCAN_MAX_BUF_MEAS,
                min_len=SCAN_MIN_LEN,
            )
        except TypeError:
            scan_iter = lidar_obj.iter_scans()

        for scan in scan_iter:
            if not running:
                break

            points = []
            for quality, angle, distance in scan:
                if MIN_DIST <= distance <= MAX_DIST:
                    points.append((math.radians(angle), distance, quality))

            if not points:
                continue

            if SCAN_HISTORY_LEN > 1:
                scan_frames.append(points)
                merged_data = _merge_scan_frames(scan_frames)
            else:
                merged_data = points

            beacon_cands = _extract_beacon_candidates_fast(merged_data)

            with lock:
                scan_read_buf = list(merged_data)
                merged_count  = len(scan_read_buf)

            with beacon_lock:
                beacon_candidates_buf = beacon_cands

            with _teensy_pose_lock:
                teensy_x, teensy_y, teensy_theta = _teensy_pose

            windows = None
            if teensy_x is not None:
                windows = _predict_beacon_windows(teensy_x, teensy_y, teensy_theta)

            if len(beacon_cands) >= POSE_CORRECTION_MIN_BEACONS:
                t_x     = teensy_x     if teensy_x     is not None else 0.0
                t_y     = teensy_y     if teensy_y     is not None else 0.0
                t_theta = teensy_theta if teensy_theta is not None else 0.0

                associated = _associate_candidates_to_beacons(
                    beacon_cands, t_x, t_y, t_theta, windows
                )

                if (len(associated) >= POSE_CORRECTION_MIN_BEACONS and
                        _validate_beacon_geometry(associated) and
                        teensy_x is not None):
                    try:
                        corrected_pose = _compute_corrected_pose(
                            associated, teensy_x, teensy_y, teensy_theta
                        )
                        if corrected_pose is not None:
                            with _corrected_pose_lock:
                                _corrected_pose = corrected_pose
                    except Exception as e:
                        logger.debug(f"SVD correction error: {e}")

            opponent_candidate = _detect_opponent_fast(merged_data, beacon_cands)
            _update_opponent(opponent_candidate)

            arr_raw = np.asarray(points, dtype=np.float32)
            d_raw   = arr_raw[:, 1]
            console_callback(
                f'Scan brut {len(points):3d} pts | '
                f'fusion {merged_count:3d} pts | '
                f'balises {len(beacon_cands)} | '
                f'min {d_raw.min():6.0f} mm | '
                f'max {d_raw.max():6.0f} mm | '
                f'moy {d_raw.mean():6.0f} mm\n'
            )

    except Exception as exc:
        status_callback(f'Erreur : {exc}')
        console_callback(f'[ERREUR] {exc}\n')
    finally:
        if lidar_obj is not None:
            try:
                lidar_obj.stop()
                lidar_obj.stop_motor()
                lidar_obj.disconnect()
            except Exception:
                pass
            lidar_obj = None
        status_callback('Deconnecte')
        console_callback('[INFO] Déconnecté.\n')


# ── API PUBLIQUE — ODOMÉTRIE ──────────────────────────────────────────────────

def update_teensy_pose(x: float, y: float, theta: float) -> None:
    """Reçoit la pose actuelle de la Teensy (appelée par robot.py)."""
    global _teensy_pose
    with _teensy_pose_lock:
        _teensy_pose = (float(x), float(y), float(theta))


def get_corrected_pose() -> PoseState:
    """Retourne la dernière pose corrigée par SVD Umeyama."""
    with _corrected_pose_lock:
        return _corrected_pose


def should_send_correction_to_teensy() -> bool:
    """True si une correction peut être envoyée vers la Teensy."""
    global _last_correction_time

    with _corrected_pose_lock:
        if not _corrected_pose.is_localized:
            return False
        if _corrected_pose.confidence < POSE_CORRECTION_MIN_CONFIDENCE:
            return False
        if len(_corrected_pose.beacon_ids or []) < POSE_CORRECTION_MIN_BEACONS:
            return False

    if time.time() - _last_correction_time < POSE_SEND_BACK_INTERVAL_S:
        return False

    _last_correction_time = time.time()
    return True


# ── API PUBLIQUE — SCAN ───────────────────────────────────────────────────────

def get_latest_scan_data() -> List[Tuple[float, float, float]]:
    """Snapshot thread-safe du dernier scan fusionné."""
    with lock:
        return list(scan_read_buf)


def get_latest_beacon_candidates() -> List[Dict]:
    """Snapshot thread-safe des candidats-balises pré-filtrés."""
    with beacon_lock:
        return list(beacon_candidates_buf)


# ── API PUBLIQUE — POSE ET ADVERSAIRE ────────────────────────────────────────

def get_latest_pose() -> PoseState:
    """Alias de compatibilité → get_corrected_pose()."""
    return get_corrected_pose()


def get_latest_opponent() -> OpponentState:
    """Retourne la dernière position détectée du robot adverse."""
    with _opponent_lock:
        return _opponent_state


# ── CONTRÔLE THREAD ───────────────────────────────────────────────────────────

def stop_lidar_runtime() -> None:
    """Demande l'arrêt propre du thread d'acquisition."""
    global running
    running = False


def start_lidar_thread(console_callback, status_callback) -> threading.Thread:
    """Lance le thread d'acquisition LiDAR."""
    global running, scan_read_buf, scan_write_buf

    running = True
    scan_frames.clear()
    with lock:
        scan_read_buf  = []
        scan_write_buf = []

    thread = threading.Thread(
        target=lidar_thread,
        args=(console_callback, status_callback),
        daemon=True,
    )
    thread.start()
    return thread