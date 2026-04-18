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
    # Fallback de compatibilite si la cle "terrain" n'est pas configuree.
    ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    if ROOT_DIR not in sys.path:
        sys.path.insert(0, ROOT_DIR)
    import terrain_jeu as _terrain_module
    BeaconLayout = _terrain_module.BeaconLayout

FIELD_WIDTH_MM = float(getattr(_terrain_module, 'FIELD_WIDTH_MM', 3000.0))
FIELD_HEIGHT_MM = float(getattr(_terrain_module, 'FIELD_HEIGHT_MM', 2000.0))

# Logger configuration
logger = logging.getLogger("LIDAR")


def _build_display_beacons(beacons_by_id: Dict[int, Tuple[float, float]]) -> Dict[str, Tuple[float, float]]:
    """Construit un mapping A/B/C... pour l'UI et la couche fusion."""
    labels = ("A", "B", "C", "D", "E")
    mapping: Dict[str, Tuple[float, float]] = {}
    for idx, beacon_id in enumerate(sorted(beacons_by_id)):
        label = labels[idx] if idx < len(labels) else f"B{beacon_id}"
        bx, by = beacons_by_id[beacon_id]
        mapping[label] = (float(bx), float(by))
    return mapping

# ── CONFIG RUNTIME LIDAR ──────────────────────────────────────────────────────
PORT = '/dev/ttyUSB0'
BAUDRATE = 256000
TIMEOUT = 3
MIN_DIST = 50
MAX_DIST = 12000
MIN_QUAL = 1

# Acquisition scan: fusion de scans successifs pour depasser la densite brute
# d'un seul tour sans laisser exploser la charge CPU/UI.
SCAN_MAX_BUF_MEAS = 1500
SCAN_MIN_LEN = 20
SCAN_HISTORY_LEN = 2
MERGE_ANGLE_BIN_DEG = 0.5
MAX_MERGED_POINTS = 1200

# Plateau reel: dimensions centralisees dans terrain_jeu.py
MAP_W_MM = int(FIELD_WIDTH_MM)
MAP_H_MM = int(FIELD_HEIGHT_MM)

# Balises: source unique depuis BeaconLayout
BEACON_SIZE_MM = float(getattr(BeaconLayout, 'BEACON_SIZE_MM', 100.0))
BEACONS_BY_ID = {
    int(bid): (float(pos[0]), float(pos[1]))
    for bid, pos in getattr(BeaconLayout, 'BEACONS', {}).items()
}
if not BEACONS_BY_ID:
    BEACONS_BY_ID = {
        1: (3050.0, 1950.0),
        2: (3050.0, 50.0),
        3: (-50.0, 1000.0),
    }

_beacon_xs = [bx for bx, _ in BEACONS_BY_ID.values()]
_beacon_ys = [by for _, by in BEACONS_BY_ID.values()]

BEACON_OUTSIDE_LEFT_MM = max(0.0, -min(_beacon_xs))
BEACON_OUTSIDE_RIGHT_MM = max(0.0, max(_beacon_xs) - float(MAP_W_MM))
BEACON_OUTSIDE_BOTTOM_MM = max(0.0, -min(_beacon_ys))
BEACON_OUTSIDE_TOP_MM = max(0.0, max(_beacon_ys) - float(MAP_H_MM))

OFFSET_BALISE_MM = max(
    BEACON_OUTSIDE_LEFT_MM,
    BEACON_OUTSIDE_RIGHT_MM,
    BEACON_OUTSIDE_BOTTOM_MM,
    BEACON_OUTSIDE_TOP_MM,
)
MAP_VIEW_MARGIN_MM = max(150.0, OFFSET_BALISE_MM + (BEACON_SIZE_MM * 0.5) + 20.0)

# Fenetre dediee a la localisation: on ignore les retours tres lointains
# tout en couvrant les balises placees hors du plateau.
POSE_MIN_DIST_MM = 120
POSE_MAX_DIST_MM = int(
    math.hypot(
        MAP_W_MM + BEACON_OUTSIDE_LEFT_MM + BEACON_OUTSIDE_RIGHT_MM,
        MAP_H_MM + BEACON_OUTSIDE_BOTTOM_MM + BEACON_OUTSIDE_TOP_MM,
    ) + 200.0
)

# Clustering / suivi objets sur le plateau
CLUSTER_GAP_MM = max(120.0, 1.2 * BEACON_SIZE_MM)
CLUSTER_MIN_POINTS = 4
TRACK_MATCH_MM = 220
TRACK_MAX_MISSED = 8
TRACK_HISTORY_LEN = 20

# Balises de reference pour la localisation et l'affichage (A/B/C...)
BEACONS_TEST = _build_display_beacons(BEACONS_BY_ID)

# Localisation auto a partir de 2 ou 3 balises detectees.
AUTO_BEACON_LOCALIZATION = True
BEACON_QUAL_MIN = 2
BEACON_ANG_GAP_RAD = math.radians(2.5)
BEACON_DIST_GAP_MM = max(40.0, 0.90 * BEACON_SIZE_MM)
BEACON_MIN_RETURNS_PER_CLUSTER = 2
BEACON_MAX_CANDIDATES = 3          # était 6 — exactement 3 balises sur le plateau
BEACON_FIT_MAX_RMS_MM = 80.0       # était 140 — plus strict
BEACON_GEOM_TOL_MM = 70.0          # était 180 — géométrie connue et fixe
BEACON_EDGE_DROP_POINTS = 1
BEACON_HALF_DEPTH_MM = 0.5 * BEACON_SIZE_MM
BEACON_FACE_MIN_LEN_MM = max(25.0, 0.65 * BEACON_SIZE_MM)
BEACON_FACE_MAX_LEN_MM = 1.05 * BEACON_SIZE_MM
AUTO_POSE_MIN_CONFIDENCE = 0.20
TWO_BEACON_BASE_CONFIDENCE = 0.50
POSE_CONTINUITY_RMS_TOL_MM = 28.0
POSE_BLEND_ALPHA_XY = 0.35
POSE_BLEND_ALPHA_THETA = 0.30
POSE_DEADBAND_MM = 15.0
POSE_DEADBAND_DEG = 1.5
MIN_POINTS_FOR_POSE = 120
POSE_MAX_JUMP_MM = 260.0
POSE_MAX_JUMP_DEG = 28.0
POSE_MAX_MISSES_BEFORE_UNLOCK = 10
FUSION_MIN_CONFIDENCE = 0.14

# ── CORRECTION D'ODOMÉTRIE (Prédiction fenêtres + SVD) ──────────────────────
BEACON_WINDOW_ANGLE_RAD = math.radians(10.0)    # ±10° autour angle prédit
BEACON_WINDOW_DIST_MM = 200.0                   # ±200mm autour distance prédite
POSE_CORRECTION_MIN_CONFIDENCE = 0.60           # Seuil confiance pour correction
POSE_CORRECTION_MIN_BEACONS = 2                 # Min 2 balises pour correction
POSE_SEND_BACK_INTERVAL_S = 1.0                 # Correction Teensy toutes les 1s

# Detection robot adverse (coordonnees absolues sur le plateau)
ROBOT_MIN_RADIUS_MM = 60.0
ROBOT_MAX_RADIUS_MM = 220.0
OPPONENT_BEACON_EXCLUSION_MM = max(150.0, BEACON_SIZE_MM + 50.0)
OPPONENT_MAX_MISSED = 12

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
    'get_latest_scan_data', 'get_latest_beacon_candidates',
    'get_latest_pose', 'get_latest_opponent',
    'stop_lidar_runtime', 'start_lidar_thread',
]

# ── STRUCTURES DE DONNEES ─────────────────────────────────────────────────────

@dataclass
class PoseState:
    """État de pose du robot estimée depuis LiDAR."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    confidence: float = 0.0
    beacon_ids: Optional[List[str]] = None
    is_localized: bool = False
    last_update_time: float = 0.0

@dataclass
class OpponentState:
    """État du robot adverse détecté."""
    x: float = 0.0
    y: float = 0.0
    confidence: float = 0.0
    missed_count: int = 0
    last_update_time: float = 0.0


# ── MESURES GÉOMÉTRIQUES (Classification Balise vs Robot) ────────────────────

# Seuils de classification
LINEARITY_THRESHOLD = 2.0       # Ratio λ_max/λ_min : > 2.0 = linéaire (balise)
CIRCULARITY_THRESHOLD = 0.15    # Coeff variation rayons : < 0.15 = circulaire (robot)


def _measure_linearity(points_xy: np.ndarray) -> float:
    """
    Mesure la linéarité d'un cluster via PCA.
    
    Args:
        points_xy: Array (N, 2) de points [x, y]
    
    Returns:
        float: Ratio λ_max / λ_min des valeurs propres
               > 2.0 = objet linéaire (balise)
               ≈ 1.0 = objet isotrope (robot)
    """
    if points_xy is None or len(points_xy) < 2:
        return 1.0
    
    try:
        # Centrer les points
        centroid = np.mean(points_xy, axis=0)
        points_centered = points_xy - centroid
        
        # Covariance
        cov_matrix = np.cov(points_centered.T)
        
        # Valeurs propres via SVD
        _, s, _ = np.linalg.svd(cov_matrix)
        eigenvalues = s ** 2
        
        # Ratio λ_max / λ_min
        if len(eigenvalues) >= 2 and eigenvalues[1] > 1e-6:
            ratio = eigenvalues[0] / eigenvalues[1]
        else:
            ratio = 1.0
        
        return float(ratio)
    
    except Exception as e:
        logger.debug(f"Linearity measurement error: {e}")
        return 1.0


def _measure_circularity(points_xy: np.ndarray) -> float:
    """
    Mesure la circularité d'un cluster.
    
    Args:
        points_xy: Array (N, 2) de points [x, y]
    
    Returns:
        float: Coefficient de variation des rayons (écart-type / moyenne)
               < 0.15 = profil circulaire (robot)
               > 0.20 = profil irrégulier (balise)
    """
    if points_xy is None or len(points_xy) < 3:
        return 1.0
    
    try:
        # Centroïde
        centroid = np.mean(points_xy, axis=0)
        
        # Rayons au centroïde
        distances = np.linalg.norm(points_xy - centroid, axis=1)
        
        # Coefficient de variation
        if np.mean(distances) > 1e-6:
            cv = np.std(distances) / np.mean(distances)
        else:
            cv = 1.0
        
        return float(cv)
    
    except Exception as e:
        logger.debug(f"Circularity measurement error: {e}")
        return 1.0


def _classify_cluster(points_xy: np.ndarray) -> dict:
    """
    Classifie un cluster comme balise ou robot adverse.
    
    Args:
        points_xy: Array (N, 2) de points [x, y]
    
    Returns:
        dict avec clés:
            - is_beacon_like: bool (linéaire + irrégulier)
            - is_robot_like: bool (circulaire + régulier)
            - linearity: float (ratio PCA)
            - circularity: float (coeff variation)
    """
    linearity = _measure_linearity(points_xy)
    circularity = _measure_circularity(points_xy)
    
    is_beacon_like = (linearity > LINEARITY_THRESHOLD)
    is_robot_like = (circularity < CIRCULARITY_THRESHOLD)
    
    return {
        'is_beacon_like': is_beacon_like,
        'is_robot_like': is_robot_like,
        'linearity': linearity,
        'circularity': circularity,
    }


def _detect_opponent(merged_data: List[Tuple], 
                     beacon_candidates: List[Dict],
                     pose_estimate: Optional[PoseState] = None) -> Optional[Dict]:
    """
    Détecte le robot adverse via cascade de filtres.
    
    On connaît la position des balises → tout ce qui n'est pas une balise 
    dans la bonne position est un candidat adversaire.
    
    Args:
        merged_data: Points bruts LiDAR fusionnés [(angle_rad, dist_mm, quality), ...]
        beacon_candidates: Candidats détectés comme balises (pour exclusion)
        pose_estimate: Pose robot (optionnel)
    
    Returns:
        dict {x, y, confidence, points_count} ou None
    
    Filtres appliqués (dans l'ordre):
        1. Taille du cluster: rayon 60-220 mm
        2. Exclusion balises: distance > 150 mm de tout candidat balise
        3. Contrainte terrain: x ∈ [-50, 3050], y ∈ [-50, 2050]
        4. Linéarité: rejeter si trop linéaire (balise)
    """
    if not merged_data or len(merged_data) < 3:
        return None
    
    try:
        # ─ Convertir scan brut en points XY ──────────────────────────────────
        all_points_xy = []
        for angle_rad, dist_mm, quality in merged_data:
            x = dist_mm * np.cos(angle_rad)
            y = dist_mm * np.sin(angle_rad)
            all_points_xy.append([x, y])
        
        all_points_xy = np.array(all_points_xy)
        
        # ─ Clustering spatial simple (DBScan-like) ──────────────────────────
        # Grouper les points proches les uns des autres
        clusters = []
        used = set()
        
        for i, point in enumerate(all_points_xy):
            if i in used:
                continue
            
            # Trouver tous les points proches
            dists = np.linalg.norm(all_points_xy - point, axis=1)
            neighbors = np.where(dists < CLUSTER_GAP_MM)[0]
            
            if len(neighbors) >= CLUSTER_MIN_POINTS:
                cluster_points = all_points_xy[neighbors]
                for idx in neighbors:
                    used.add(idx)
                clusters.append(cluster_points)
        
        if not clusters:
            return None
        
        # ─ Filtrer clusters pour trouver adversaire ──────────────────────────
        candidates = []
        
        for cluster_points in clusters:
            # Filtre 1: Taille du cluster (rayon 60-220mm)
            centroid = np.mean(cluster_points, axis=0)
            distances = np.linalg.norm(cluster_points - centroid, axis=1)
            cluster_radius = np.max(distances)
            
            if not (ROBOT_MIN_RADIUS_MM <= cluster_radius <= ROBOT_MAX_RADIUS_MM):
                continue
            
            # Filtre 2: Exclusion balises (distance > 150mm de toute balise connue)
            too_close_to_beacon = False
            for beacon_cand in beacon_candidates:
                beacon_x = beacon_cand.get('x_r', 0)  # Relatif au robot
                beacon_y = beacon_cand.get('y_r', 0)
                dist_to_beacon = np.linalg.norm(
                    centroid - np.array([beacon_x, beacon_y])
                )
                if dist_to_beacon < OPPONENT_BEACON_EXCLUSION_MM:
                    too_close_to_beacon = True
                    break
            
            if too_close_to_beacon:
                continue
            
            # Filtre 3: Contrainte terrain
            x, y = centroid
            if not (-50 <= x <= MAP_W_MM + 50 and -50 <= y <= MAP_H_MM + 50):
                continue
            
            # Filtre 4: Linéarité (rejeter si trop linéaire = balise)
            classification = _classify_cluster(cluster_points)
            if classification['is_beacon_like']:
                continue
            
            # ─ Candidat valide ────────────────────────────────────────────────
            confidence = 0.6 + 0.3 * float(classification['is_robot_like'])
            candidates.append({
                'x': float(x),
                'y': float(y),
                'confidence': float(confidence),
                'points_count': len(cluster_points),
                'cluster_radius': float(cluster_radius),
                'linearity': classification['linearity'],
                'circularity': classification['circularity'],
            })
        
        if not candidates:
            return None
        
        # Retourner le candidat avec la plus haute confiance
        best = max(candidates, key=lambda c: c['confidence'])
        return best
    
    except Exception as e:
        logger.debug(f"Opponent detection error: {e}")
        return None


# ── MOTEUR DE POSE ────────────────────────────────────────────────────────────

class PoseEngine:
    """
    Calcule la pose du robot (x, y, theta) à partir des balises détectées.
    
    Stratégies :
      - 3 balises : trilatération SVD complète
      - 2 balises : trilatération restreinte + confiance réduite
    """
    
    def __init__(self):
        self.last_pose = PoseState()
        self.last_opponent = OpponentState()
        self.pose_lock = threading.Lock()
        self.opponent_lock = threading.Lock()
    
    def estimate_pose_from_beacons(self, beacon_candidates: List[Dict], quality_scores: Dict) -> Optional[PoseState]:
        """
        Estime la pose depuis les candidats-balises.
        
        Args:
            beacon_candidates: Liste de dicts [[{'angle_rad': ..., 'distance_mm': ...}, ...], ...]
            quality_scores: Dict {beacon_id: score} pour confiance
        
        Returns:
            PoseState ou None si échouée
        """
        if not beacon_candidates or len(beacon_candidates) < 2:
            return None
        
        try:
            # Regrouper les points par balise
            clusters = self._cluster_beacon_points(beacon_candidates)
            
            if len(clusters) < 2:
                return None
            
            # Essayer trilatération 3 balises puis 2 balises
            if len(clusters) >= 3:
                pose = self._estimate_from_three_beacons(clusters, quality_scores)
                if pose:
                    return pose
            
            # Secours : 2 balises
            if len(clusters) >= 2:
                pose = self._estimate_from_two_beacons(clusters, quality_scores)
                if pose:
                    return pose
            
            return None
        
        except Exception as e:
            print(f"[PoseEngine] Erreur estimation pose: {e}")
            return None
    
    def _cluster_beacon_points(self, candidates: List[Dict]) -> Dict[int, np.ndarray]:
        """
        Regroupe les points bruts par balise (clustering spatial).
        
        Returns:
            Dict {beacon_id: array of (x, y) points}
        """
        clusters = {}
        beacons_ref = BEACONS_BY_ID
        
        # Convertir points (angle, distance) → (x, y)
        points_xy = []
        for point in candidates:
            angle = point.get('angle_rad', 0)
            dist = point.get('distance_mm', 0)
            x = dist * np.cos(angle)
            y = dist * np.sin(angle)
            points_xy.append((x, y))
        
        if not points_xy:
            return {}
        
        points_xy = np.array(points_xy)
        
        # Assigner chaque point à la balise la plus proche
        for i, (bid, (bx, by)) in enumerate(beacons_ref.items()):
            beacon_pos = np.array([bx, by])
            dists = np.linalg.norm(points_xy - beacon_pos, axis=1)
            
            # Seuil : points dans ±CLUSTER_GAP_MM autour balise
            mask = dists < CLUSTER_GAP_MM
            if np.any(mask):
                clusters[bid] = points_xy[mask]
        
        return clusters
    
    def _estimate_from_three_beacons(self, clusters: Dict[int, np.ndarray], 
                                    quality_scores: Dict) -> Optional[PoseState]:
        """Trilatération 3 balises via SVD."""
        beacon_ids = list(clusters.keys())[:3]
        if len(beacon_ids) < 3:
            return None
        
        beacons_ref = BEACONS_BY_ID
        
        try:
            # Centroïde de chaque cluster
            centers = []
            for bid in beacon_ids:
                center = np.mean(clusters[bid], axis=0)
                centers.append(center)
            centers = np.array(centers)
            
            # Positions référence balises
            beacon_positions = np.array([beacons_ref[bid] for bid in beacon_ids])
            
            # SVD pour meilleure transformation affine
            # Formulation: chercher (x_robot, y_robot) tel que
            # distance(robot, balise_i) = ||center_i||
            
            dists = np.linalg.norm(centers, axis=1)
            
            # Construction système surdéterminé en 2D
            # (x - bx)^2 + (y - by)^2 = d^2  →  linéarisé
            A = []
            b_vec = []
            for i, bid in enumerate(beacon_ids):
                bx, by = beacon_positions[i]
                d = dists[i]
                # Linéarisation: 2*bx*x + 2*by*y + const = rhs
                A.append([2*bx, 2*by])
                b_vec.append(bx**2 + by**2 - d**2)
            
            A = np.array(A)
            b_vec = np.array(b_vec)
            
            # Moindres carrés
            pose_xy, residuals, rank, s = np.linalg.lstsq(A, b_vec, rcond=None)
            
            # RMS error validation
            if residuals.size > 0:
                rms = np.sqrt(residuals[0] / len(beacon_ids))
                if rms > BEACON_FIT_MAX_RMS_MM:
                    return None
                confidence = max(AUTO_POSE_MIN_CONFIDENCE, 1.0 - rms / 100)
            else:
                confidence = 1.0
            
            # Calcul theta (direction moyenne des balises)
            angles = np.arctan2(centers[:, 1], centers[:, 0])
            theta = float(np.mean(angles))
            
            state = PoseState(
                x=float(pose_xy[0]),
                y=float(pose_xy[1]),
                theta=theta,
                confidence=float(confidence),
                beacon_ids=[str(bid) for bid in beacon_ids],
                is_localized=confidence >= AUTO_POSE_MIN_CONFIDENCE,
                last_update_time=time.time()
            )
            
            with self.pose_lock:
                self.last_pose = state
            
            return state
        
        except Exception as e:
            print(f"[PoseEngine] 3-balises échoué: {e}")
            return None
    
    def _estimate_from_two_beacons(self, clusters: Dict[int, np.ndarray],
                                   quality_scores: Dict) -> Optional[PoseState]:
        """Trilatération restreinte 2 balises (confiance réduite)."""
        beacon_ids = list(clusters.keys())[:2]
        if len(beacon_ids) < 2:
            return None
        
        beacons_ref = BEACONS_BY_ID
        
        try:
            # Centroïdes
            centers = []
            for bid in beacon_ids:
                center = np.mean(clusters[bid], axis=0)
                centers.append(center)
            centers = np.array(centers)
            
            dists = np.linalg.norm(centers, axis=1)
            
            # Positions de référence
            b1_pos = np.array(beacons_ref[beacon_ids[0]])
            b2_pos = np.array(beacons_ref[beacon_ids[1]])
            d1, d2 = dists[0], dists[1]
            
            # Géométrie cercle-cercle simple
            # Deux cercles: C1(b1, d1) et C2(b2, d2)
            # Intersection en 2 points, prendre celui devant balises
            
            mid = (b1_pos + b2_pos) / 2
            
            # Projection orthogonale sur ligne balises
            line_vec = b2_pos - b1_pos
            line_dist = np.linalg.norm(line_vec)
            
            if line_dist < 1e-6:
                return None
            
            line_unit = line_vec / line_dist
            
            # Position robot approximée (moyenne des deux distances)
            robot_approx = mid + (np.mean([d1, d2]) / line_dist) * 0.1 * line_unit
            robot_approx[1] *= -1  # Géométrie terrain
            
            confidence = TWO_BEACON_BASE_CONFIDENCE
            
            # Theta depuis direction moyenne
            angles = np.arctan2(centers[:, 1], centers[:, 0])
            theta = float(np.mean(angles))
            
            state = PoseState(
                x=float(robot_approx[0]),
                y=float(robot_approx[1]),
                theta=theta,
                confidence=float(confidence),
                beacon_ids=[str(bid) for bid in beacon_ids],
                is_localized=confidence >= AUTO_POSE_MIN_CONFIDENCE,
                last_update_time=time.time()
            )
            
            with self.pose_lock:
                self.last_pose = state
            
            return state
        
        except Exception as e:
            print(f"[PoseEngine] 2-balises échoué: {e}")
            return None
    
    def update_opponent_tracking(self, opponent_candidates: List[Dict]) -> None:
        """
        Met à jour la position du robot adverse détecté.
        """
        if not opponent_candidates:
            # Incrémenter missed count
            with self.opponent_lock:
                self.last_opponent.missed_count += 1
                if self.last_opponent.missed_count > OPPONENT_MAX_MISSED:
                    self.last_opponent.confidence = 0.0
            return
        
        try:
            # Prendre le cluster principal
            opp_center = np.mean([
                (c.get('x', 0), c.get('y', 0)) 
                for c in opponent_candidates
            ], axis=0)
            
            new_state = OpponentState(
                x=float(opp_center[0]),
                y=float(opp_center[1]),
                confidence=0.7,  # A affiner avec scores
                missed_count=0,
                last_update_time=time.time()
            )
            
            with self.opponent_lock:
                self.last_opponent = new_state
        
        except Exception as e:
            print(f"[PoseEngine] Opposition tracking échoué: {e}")
    
    def get_pose(self) -> PoseState:
        """Récupère la dernière pose estimée."""
        with self.pose_lock:
            return self.last_pose
    
    def get_opponent(self) -> OpponentState:
        """Récupère la dernière position de l'adversaire."""
        with self.opponent_lock:
            return self.last_opponent


# Instance globale
pose_engine = PoseEngine()

# ── ETAT PARTAGE ACQUISITION ──────────────────────────────────────────────────
scan_read_buf = []
scan_write_buf = []
scan_frames = deque(maxlen=SCAN_HISTORY_LEN)
lock = threading.Lock()
running = True
lidar_obj = None

# ── ETAT PARTAGE BALISES PRE-FILTREES ────────────────────────────────────────
# Contient uniquement les points candidats-balises (haute intensite, bonne distance)
# extraits a chaque scan, prets pour le clustering dans lidar_gui.py.
beacon_candidates_buf = []
beacon_lock = threading.Lock()

# ── CORRECTION D'ODOMÉTRIE ─────────────────────────────────────────────────────
# Pose actuelle de la Teensy (x, y, theta) - mise à jour par robot.py
_teensy_pose = (None, None, None)
_teensy_pose_lock = threading.Lock()

# Dernière pose corrigée par LiDAR (à envoyer vers Teensy)
_corrected_pose = PoseState()
_corrected_pose_lock = threading.Lock()

# Timestamp dernier renvoi vers Teensy
_last_correction_time = 0.0


# ── PRE-FILTRAGE BALISES ──────────────────────────────────────────────────────



# ── CORRECTION SVD UMEYAMA ────────────────────────────────────────────────────

def _compute_corrected_pose(beacon_candidates: List[Dict], 
                            teensy_x: float, teensy_y: float, 
                            teensy_theta: float) -> Optional[PoseState]:
    """
    Corrige la pose Teensy en comparant balises mesurées vs théoriques (SVD Umeyama 2D).
    
    Args:
        beacon_candidates: Candidats avec 'beacon_id' fourni
        teensy_x/y/theta: Pose Teensy courante (transformation provisoire)
    
    Returns:
        PoseState corrigée ou None si échec
    """
    if len(beacon_candidates) < POSE_CORRECTION_MIN_BEACONS:
        return None
    
    # Étape A : Construire correspondances
    measured_points = []  # Positions mesurées en monde
    theoretical_points = []  # Positions théoriques
    beacon_ids_used = []
    
    cos_theta = math.cos(teensy_theta)
    sin_theta = math.sin(teensy_theta)
    
    for cand in beacon_candidates:
        if 'beacon_id' not in cand:
            continue
        
        bid = int(cand['beacon_id'])
        if bid not in BEACONS_BY_ID:
            continue
        
        # Coordonnées mesurées relatives au LiDAR
        x_lidar = cand.get('x_r', 0.0)
        y_lidar = cand.get('y_r', 0.0)
        
        # Transformer en monde via pose Teensy provisoire
        x_measured = teensy_x + (x_lidar * cos_theta - y_lidar * sin_theta)
        y_measured = teensy_y + (x_lidar * sin_theta + y_lidar * cos_theta)
        
        measured_points.append([x_measured, y_measured])
        theoretical_points.append(list(BEACONS_BY_ID[bid]))
        beacon_ids_used.append(str(bid))
    
    if len(measured_points) < POSE_CORRECTION_MIN_BEACONS:
        return None
    
    # Étape B : SVD Umeyama 2D
    measured = np.array(measured_points)
    theoretical = np.array(theoretical_points)
    
    # Centroïdes
    centroid_m = np.mean(measured, axis=0)
    centroid_t = np.mean(theoretical, axis=0)
    
    # Centrer
    measured_c = measured - centroid_m
    theoretical_c = theoretical - centroid_t
    
    # Covariance croisée
    H = measured_c.T @ theoretical_c
    
    # SVD
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Garantir rotation pure (det=1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Extraire theta de rotation
    delta_theta = float(np.arctan2(R[1, 0], R[0, 0]))
    
    # Translation
    t = centroid_t - R @ centroid_m
    delta_x = float(t[0])
    delta_y = float(t[1])
    
    # Étape C : Validation
    corrected_measured = (measured @ R.T) + t
    residuals = corrected_measured - theoretical
    rms = float(np.sqrt(np.mean(residuals**2)))
    
    if rms > BEACON_FIT_MAX_RMS_MM:
        return None
    
    # Confidence basée sur nombre de balises et RMS
    if len(beacon_ids_used) >= 3:
        confidence = max(0.7, 1.0 - rms / 100.0)
    else:
        confidence = max(0.5, 0.8 - rms / 100.0)
    
    # Pose corrigée finale
    x_corrected = teensy_x + delta_x
    y_corrected = teensy_y + delta_y
    theta_corrected = teensy_theta + delta_theta
    
    return PoseState(
        x=float(x_corrected),
        y=float(y_corrected),
        theta=float(theta_corrected),
        confidence=float(confidence),
        beacon_ids=beacon_ids_used,
        is_localized=confidence >= POSE_CORRECTION_MIN_CONFIDENCE,
        last_update_time=time.time()
    )


# ── PRÉDICTION FENÊTRES BALISES ───────────────────────────────────────────────

def _predict_beacon_windows(teensy_x: float, teensy_y: float, 
                            teensy_theta: float) -> Optional[Dict]:
    """
    Calcule pour chaque balise sa fenêtre de prédiction dans le scan LiDAR.
    
    Args:
        teensy_x/y/theta: Pose Teensy actuelle (repère monde)
    
    Returns:
        Dict {beacon_id: {
            'angle_pred': rad,
            'dist_pred': mm,
            'angle_min': rad,
            'angle_max': rad,
            'dist_min': mm,
            'dist_max': mm
        }} ou None si pose invalide
    """
    if teensy_x is None or teensy_y is None or teensy_theta is None:
        return None
    
    windows = {}
    
    cos_theta = math.cos(teensy_theta)
    sin_theta = math.sin(teensy_theta)
    
    for beacon_id, (bx_world, by_world) in BEACONS_BY_ID.items():
        # Transformer balise du monde au repère robot
        dx_world = bx_world - teensy_x
        dy_world = by_world - teensy_y
        
        # Rotation inverse pour passer au repère robot
        dx_robot = dx_world * cos_theta + dy_world * sin_theta
        dy_robot = -dx_world * sin_theta + dy_world * cos_theta
        
        # Conversion en polaire
        dist_pred = math.hypot(dx_robot, dy_robot)
        angle_pred = math.atan2(dy_robot, dx_robot)
        
        # Filtrer si hors portée LiDAR
        if dist_pred < POSE_MIN_DIST_MM or dist_pred > POSE_MAX_DIST_MM:
            continue
        
        # Fenêtres de recherche
        angle_min = angle_pred - BEACON_WINDOW_ANGLE_RAD
        angle_max = angle_pred + BEACON_WINDOW_ANGLE_RAD
        dist_min = max(POSE_MIN_DIST_MM, dist_pred - BEACON_WINDOW_DIST_MM)
        dist_max = min(POSE_MAX_DIST_MM, dist_pred + BEACON_WINDOW_DIST_MM)
        
        windows[beacon_id] = {
            'angle_pred': float(angle_pred),
            'dist_pred': float(dist_pred),
            'angle_min': float(angle_min),
            'angle_max': float(angle_max),
            'dist_min': float(dist_min),
            'dist_max': float(dist_max),
        }
    
    return windows if windows else None


def _extract_beacon_candidates_fast(points):
    """
    Extrait les candidats-balises depuis un scan fusionne.

    Criteres appliques dans l'ordre (du plus rapide au plus selectif) :
      1. Fenetre de distance utile [POSE_MIN_DIST_MM, POSE_MAX_DIST_MM]
         → elimine le bruit tres proche et les retours hors plateau.
            2. Qualite >= BEACON_QUAL_MIN (= 2)
         → elimine la grande majorite des points mysterieux (bruit, multi-path,
           retours de faible reflexion). C'est le filtre le plus discriminant.
      3. Tri angulaire + clustering angulaire/distance
         → regroupe les points contigus qui pourraient former une face de balise.
      4. Validation taille du cluster (>= BEACON_MIN_RETURNS_PER_CLUSTER)
         → rejette les singletons isoles.

    Retourne une liste de dicts :
        {"angle": rad, "distance": mm, "quality": float,
         "count": int, "x_r": mm, "y_r": mm}
    """
    if not points:
        return []

    arr = np.asarray(points, dtype=float)  # shape (N, 3): rad, mm, qual

    # ── Etape 1 : fenetre distance ────────────────────────────────────────────
    d_mask = (arr[:, 1] >= POSE_MIN_DIST_MM) & (arr[:, 1] <= POSE_MAX_DIST_MM)
    arr = arr[d_mask]
    if len(arr) < BEACON_MIN_RETURNS_PER_CLUSTER:
        return []

    # ── Etape 2 : filtre qualite haute ───────────────────────────────────────
    q_mask = arr[:, 2] >= BEACON_QUAL_MIN
    arr_hq = arr[q_mask]
    if len(arr_hq) < BEACON_MIN_RETURNS_PER_CLUSTER:
        return []

    # ── Etape 3 : tri angulaire ───────────────────────────────────────────────
    order = np.argsort(arr_hq[:, 0])
    arr_hq = arr_hq[order]

    angles = arr_hq[:, 0]
    dists  = arr_hq[:, 1]
    quals  = arr_hq[:, 2]

    # ── Etape 4 : clustering angulaire + distance ─────────────────────────────
    # Deux points consecutifs appartiennent au meme cluster si :
    #   - ecart angulaire <= BEACON_ANG_GAP_RAD
    #   - ecart distance  <= BEACON_DIST_GAP_MM
    d_angles = np.abs(np.diff(angles))
    d_dists  = np.abs(np.diff(dists))
    breaks   = np.where(
        (d_angles > BEACON_ANG_GAP_RAD) | (d_dists > BEACON_DIST_GAP_MM)
    )[0] + 1

    # Gestion du wrap-around 360°->0°
    idx_splits = np.split(np.arange(len(angles)), breaks)

    # Fusion eventuelle du premier et dernier cluster (wrap 360°)
    # Condition supplementaire : les deux clusters doivent avoir le minimum
    # de points requis chacun avant fusion pour eviter deux singletons
    # en limite de scan qui formeraient un faux candidat.
    if len(idx_splits) >= 2:
        first_idx = idx_splits[0][0]
        last_idx  = idx_splits[-1][-1]
        wrap_ang  = (angles[first_idx] + 2.0 * math.pi) - angles[last_idx]
        wrap_dist = abs(dists[first_idx] - dists[last_idx])
        both_have_enough = (
            len(idx_splits[0]) >= BEACON_MIN_RETURNS_PER_CLUSTER
            and len(idx_splits[-1]) >= BEACON_MIN_RETURNS_PER_CLUSTER
        )
        if wrap_ang <= BEACON_ANG_GAP_RAD and wrap_dist <= BEACON_DIST_GAP_MM and both_have_enough:
            merged_idx = np.concatenate([idx_splits[-1], idx_splits[0]])
            idx_splits = [merged_idx] + idx_splits[1:-1]

    # ── Etape 5 : validation et calcul du centre de chaque cluster ────────────
    candidates = []
    for idxs in idx_splits:
        if len(idxs) < BEACON_MIN_RETURNS_PER_CLUSTER:
            continue

        a_cl = angles[idxs]
        d_cl = dists[idxs]
        q_cl = quals[idxs]

        # Suppression des bords (retours d'angle rasant, moins fiables)
        drop = BEACON_EDGE_DROP_POINTS
        if len(a_cl) > 2 * drop + 1:
            a_cl = a_cl[drop:-drop]
            d_cl = d_cl[drop:-drop]
            q_cl = q_cl[drop:-drop]

        if len(a_cl) < 2:
            continue

        # Validation taille de face (distance × etendue angulaire ≈ longueur)
        mean_d  = float(np.mean(d_cl))
        ang_span = float(a_cl[-1] - a_cl[0])
        face_est = mean_d * math.tan(ang_span / 2.0) * 2.0  # estimation grossiere

        # On accepte les faces dans la plage physique d'une balise 10x10 cm
        if face_est < BEACON_FACE_MIN_LEN_MM or face_est > BEACON_FACE_MAX_LEN_MM:
            continue  # trop grand ou trop petit : pas une balise

        # Centre en coordonnees cartesiennes relatives au lidar
        x_r = float(np.mean(d_cl * np.sin(a_cl)))
        y_r = float(np.mean(d_cl * np.cos(a_cl)))

        # Decalage vers l'interieur de la balise (demi-profondeur)
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

    # Trier par qualite decroissante, limiter le nombre
    candidates.sort(key=lambda c: (c["quality"], c["count"]), reverse=True)
    return candidates[:BEACON_MAX_CANDIDATES]


def _merge_scan_frames(frames):
    """Fusion vectorisee de scans successifs par bins angulaires."""
    if not frames:
        return []

    arrays = [np.asarray(pts, dtype=float) for pts in frames if pts]
    if not arrays:
        return []

    all_pts = np.vstack(arrays)
    if all_pts.size == 0:
        return []

    rads  = all_pts[:, 0]
    dists = all_pts[:, 1]
    quals = all_pts[:, 2]

    bin_deg = max(0.1, float(MERGE_ANGLE_BIN_DEG))
    bin_rad = math.radians(bin_deg)
    bins = np.floor(np.mod(rads, 2.0 * math.pi) / bin_rad).astype(np.int32)

    # Par bin : meilleure qualite puis distance minimale.
    order   = np.lexsort((dists, -quals, bins))
    bins_s  = bins[order]
    r_s     = rads[order]
    d_s     = dists[order]
    q_s     = quals[order]

    _, first_idx = np.unique(bins_s, return_index=True)
    merged = np.column_stack((r_s[first_idx], d_s[first_idx], q_s[first_idx]))
    merged = merged[np.argsort(merged[:, 0])]

    if MAX_MERGED_POINTS > 0 and len(merged) > MAX_MERGED_POINTS:
        stride = max(1, len(merged) // MAX_MERGED_POINTS)
        merged = merged[::stride][:MAX_MERGED_POINTS]

    return [
        (float(rad), float(dist), float(qual))
        for rad, dist, qual in merged
    ]


def lidar_thread(console_callback, status_callback):
    global lidar_obj, running, scan_read_buf, scan_write_buf, beacon_candidates_buf

    try:
        lidar_obj = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        lidar_obj._serial.flushInput()
        time.sleep(1)
        status_callback('Connecte - scan en cours')
        console_callback(f'[OK] Connecté sur {PORT} à {BAUDRATE} baud\n')

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

            # ── Collecte brute ────────────────────────────────────────────────
            points = []
            for quality, angle, distance in scan:
                if MIN_DIST <= distance <= MAX_DIST:
                    rad = math.radians(angle)
                    points.append((rad, distance, quality))

            # ── Fusion temporelle ─────────────────────────────────────────────
            merged_data = points
            if SCAN_HISTORY_LEN > 1:
                scan_frames.append(points)
                merged_data = _merge_scan_frames(scan_frames)

            # ── Pre-filtrage balises (haute qualite + distance utile) ──────────
            beacon_cands = _extract_beacon_candidates_fast(merged_data)
            pose_estimate = None  # Sera calculée si >= 2 balises

            # ── Double-buffer scan complet ────────────────────────────────────
            with lock:
                scan_write_buf = merged_data
                scan_read_buf, scan_write_buf = scan_write_buf, scan_read_buf
                merged_count = len(scan_read_buf)

            # ── Buffer balises thread-safe ────────────────────────────────────
            with beacon_lock:
                beacon_candidates_buf = beacon_cands
            
            # ── CORRECTION SVD UMEYAMA: Pose corrigée via balises ──────────────
            # Récupérer pose Teensy courante pour prédiction fenêtres
            with _teensy_pose_lock:
                teensy_x, teensy_y, teensy_theta = _teensy_pose
            
            if teensy_x is not None and len(beacon_cands) >= 2:
                try:
                    # Ajouter beacon_id à chaque candidat pour SVD
                    # (Normalement fait par association aux balises connues)
                    corrected_pose = _compute_corrected_pose(
                        beacon_cands, teensy_x, teensy_y, teensy_theta
                    )
                    
                    if corrected_pose is not None:
                        with _corrected_pose_lock:
                            _corrected_pose = corrected_pose
                            _last_correction_time = time.time()
                
                except Exception as e:
                    logger.debug(f"SVD correction error: {e}")
            
            # ── Estimation de la pose depuis les balises ──────────────────────
            if len(beacon_cands) >= 2:
                # Convertir format pour PoseEngine
                beacon_list = []
                quality_scores = {}
                for cand in beacon_cands:
                    beacon_list.append({
                        'angle_rad': cand['angle'],
                        'distance_mm': cand['distance']
                    })
                    quality_scores[len(beacon_list)] = cand['quality']
                
                # Estimation pose
                pose_estimate = pose_engine.estimate_pose_from_beacons(
                    beacon_list, quality_scores
                )
            
            # ── DÉTECTION ADVERSAIRE: Clusters non-balise ──────────────────────
            # Chercher des objets circul aires qui ne sont pas des balises
            opponent_candidate = _detect_opponent(merged_data, beacon_cands, pose_estimate)
            if opponent_candidate:
                try:
                    pose_engine.update_opponent_tracking([opponent_candidate])
                except Exception as e:
                    logger.debug(f"Opponent tracking error: {e}")
            else:
                # Incrémenter missed count si pas détecté
                try:
                    pose_engine.update_opponent_tracking([])
                except Exception as e:
                    logger.debug(f"Opponent missed count error: {e}")

            # ── Log console ───────────────────────────────────────────────────
            if points:
                dists_raw = [p[1] for p in points]
                n_beacons = len(beacon_cands)
                log = (
                    f'Scan brut {len(points):3d} pts | '
                    f'fusion {merged_count:3d} pts | '
                    f'balises {n_beacons} | '
                    f'min {min(dists_raw):6.0f} mm | '
                    f'max {max(dists_raw):6.0f} mm | '
                    f'moy {sum(dists_raw) / len(dists_raw):6.0f} mm\n'
                )
                console_callback(log)

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


# ── API PUBLIQUES CORRECTION ODOMÉTRIE ─────────────────────────────────────────

def update_teensy_pose(x: float, y: float, theta: float) -> None:
    """
    Reçoit la pose actuelle de la Teensy.
    Appelée par robot.py à chaque callback USB (50 Hz).
    
    Args:
        x: Position X (mm)
        y: Position Y (mm)
        theta: Orientation (rad)
    """
    global _teensy_pose
    with _teensy_pose_lock:
        _teensy_pose = (float(x), float(y), float(theta))


def get_corrected_pose() -> PoseState:
    """
    Retourne la dernière pose corrigée par LiDAR (SVD).
    À envoyer vers la Teensy si confiance suffisante.
    """
    with _corrected_pose_lock:
        return _corrected_pose


def should_send_correction_to_teensy() -> bool:
    """
    Retourne True si une correction doit être envoyée vers la Teensy.
    Vérifie: confiance, nombre de balises, intervalle minimum écoulé.
    """
    global _last_correction_time
    
    with _corrected_pose_lock:
        if not _corrected_pose.is_localized:
            return False
        if _corrected_pose.confidence < POSE_CORRECTION_MIN_CONFIDENCE:
            return False
        if len(_corrected_pose.beacon_ids or []) < POSE_CORRECTION_MIN_BEACONS:
            return False
    
    now = time.time()
    if now - _last_correction_time < POSE_SEND_BACK_INTERVAL_S:
        return False
    
    return True


def get_latest_scan_data():
    """Retourne un snapshot thread-safe du dernier scan fusionne (tous points)."""
    with lock:
        return list(scan_read_buf)


def get_latest_beacon_candidates():
    """
    Retourne un snapshot thread-safe des candidats-balises pre-filtres.

    Ces candidats ont deja passe :
      - filtre distance [POSE_MIN_DIST_MM, POSE_MAX_DIST_MM]
            - filtre qualite >= BEACON_QUAL_MIN (2)
      - clustering angulaire coherent
      - validation taille de face

    Chaque element est un dict :
        angle    (rad)   : angle du centre de la balise depuis le lidar
        distance (mm)    : distance du centre
        quality  (float) : qualite moyenne du cluster
        count    (int)   : nombre de points dans le cluster
        face_est (mm)    : estimation de la longueur de face
        x_r      (mm)    : coordonnee X relative au lidar
        y_r      (mm)    : coordonnee Y relative au lidar
    """
    with beacon_lock:
        return list(beacon_candidates_buf)


def stop_lidar_runtime():
    """Demande l'arret propre du thread d'acquisition."""
    global running
    running = False


def start_lidar_thread(console_callback, status_callback):
    """Initialise l'etat runtime puis lance le thread d'acquisition."""
    global running, scan_read_buf, scan_write_buf

    running = True
    scan_frames.clear()
    with lock:
        scan_read_buf = []
        scan_write_buf = []
    with beacon_lock:
        pass  # reset implicite via lidar_thread

    thread = threading.Thread(
        target=lidar_thread,
        args=(console_callback, status_callback),
        daemon=True,
    )
    thread.start()
    return thread


# ── API PUBLIQUES POSE ET ADVERSAIRE ───────────────────────────────────────────

def get_latest_pose() -> PoseState:
    """
    Récupère l'estimation de pose la plus récente du robot.
    
    Returns:
        PoseState avec x, y, theta, confidence, is_localized
    """
    return pose_engine.get_pose()


def get_latest_opponent() -> OpponentState:
    """
    Récupère la dernière position détectée du robot adverse.
    
    Returns:
        OpponentState avec x, y, confidence, missed_count
    """
    return pose_engine.get_opponent()