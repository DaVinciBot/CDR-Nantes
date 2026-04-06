import itertools
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from rplidar import RPLidar

# ── CONFIG RUNTIME LIDAR ──────────────────────────────────────────────────────
PORT = 'COM5'
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

# Plateau simule: 112 cm x 63 cm
MAP_W_MM = 1120
MAP_H_MM = 630
OFFSET_BALISE_MM = 100.0
MAP_VIEW_MARGIN_MM = 150.0

# Fenetre dediee a la localisation: on ignore les retours tres lointains
# tout en couvrant les balises placees hors du plateau.
POSE_MIN_DIST_MM = 120
POSE_MAX_DIST_MM = int(
    math.hypot(
        MAP_W_MM + 2.0 * OFFSET_BALISE_MM,
        MAP_H_MM + 2.0 * OFFSET_BALISE_MM,
    ) + 200.0
) #175 cm 

# Clustering / suivi objets sur le plateau
CLUSTER_GAP_MM = 120
CLUSTER_MIN_POINTS = 4
TRACK_MATCH_MM = 220
TRACK_MAX_MISSED = 8
TRACK_HISTORY_LEN = 20

# Balises de reference pour la localisation (deportees hors plateau)
BEACONS_TEST = {
    'A': (-OFFSET_BALISE_MM, float(MAP_H_MM) + OFFSET_BALISE_MM),
    'B': (float(MAP_W_MM) / 2.0, -OFFSET_BALISE_MM),
    'C': (float(MAP_W_MM) + OFFSET_BALISE_MM, float(MAP_H_MM) + OFFSET_BALISE_MM),
}  # Normalement A Haut gauche vers (0,730), B Bas centre (560,-100), C Haut droite (1220,730)

# Localisation auto a partir de 2 ou 3 balises detectees.
AUTO_BEACON_LOCALIZATION = True
BEACON_QUAL_MIN = 2
BEACON_ANG_GAP_RAD = math.radians(2.5)
BEACON_DIST_GAP_MM = 90.0          # était 140 — cohérent avec face 100 mm max
BEACON_MIN_RETURNS_PER_CLUSTER = 2
BEACON_MAX_CANDIDATES = 3          # était 6 — exactement 3 balises sur le plateau
BEACON_FIT_MAX_RMS_MM = 80.0       # était 140 — plus strict
BEACON_GEOM_TOL_MM = 70.0          # était 180 — géométrie connue et fixe
BEACON_EDGE_DROP_POINTS = 1
BEACON_HALF_DEPTH_MM = 50.0
BEACON_FACE_MIN_LEN_MM = 65.0      # était 45 — face 100 mm, min à angle rasant ~65 mm
BEACON_FACE_MAX_LEN_MM = 105.0     # était 130 — balise 100 mm + marge 5 mm
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

# Detection robot adverse (coordonnees absolues sur le plateau)
ROBOT_MIN_RADIUS_MM = 60.0
ROBOT_MAX_RADIUS_MM = 220.0
OPPONENT_BEACON_EXCLUSION_MM = 150.0
OPPONENT_MAX_MISSED = 12

__all__ = [
    'PORT', 'BAUDRATE', 'TIMEOUT', 'MIN_DIST', 'MAX_DIST', 'MIN_QUAL',
    'SCAN_MAX_BUF_MEAS', 'SCAN_MIN_LEN', 'SCAN_HISTORY_LEN',
    'MERGE_ANGLE_BIN_DEG', 'MAX_MERGED_POINTS',
    'MAP_W_MM', 'MAP_H_MM', 'OFFSET_BALISE_MM', 'MAP_VIEW_MARGIN_MM',
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
        from .terrain_jeu import BeaconLayout
        
        clusters = {}
        beacons_ref = BeaconLayout.BEACONS
        
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
        from .terrain_jeu import BeaconLayout
        
        beacon_ids = list(clusters.keys())[:3]
        if len(beacon_ids) < 3:
            return None
        
        beacons_ref = BeaconLayout.BEACONS
        
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
        from .terrain_jeu import BeaconLayout
        
        beacon_ids = list(clusters.keys())[:2]
        if len(beacon_ids) < 2:
            return None
        
        beacons_ref = BeaconLayout.BEACONS
        
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
# extraits a chaque scan, prets pour le clustering dans test_lidar.py.
beacon_candidates_buf = []
beacon_lock = threading.Lock()


# ── PRE-FILTRAGE BALISES ──────────────────────────────────────────────────────

def _extract_beacon_candidates_fast(points):
    """
    Extrait les candidats-balises depuis un scan fusionne.

    Criteres appliques dans l'ordre (du plus rapide au plus selectif) :
      1. Fenetre de distance utile [POSE_MIN_DIST_MM, POSE_MAX_DIST_MM]
         → elimine le bruit tres proche et les retours hors plateau.
      2. Qualite >= BEACON_QUAL_MIN (= 8)
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

            # ── Double-buffer scan complet ────────────────────────────────────
            with lock:
                scan_write_buf = merged_data
                scan_read_buf, scan_write_buf = scan_write_buf, scan_read_buf
                merged_count = len(scan_read_buf)

            # ── Buffer balises thread-safe ────────────────────────────────────
            with beacon_lock:
                beacon_candidates_buf = beacon_cands
            
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


def get_latest_scan_data():
    """Retourne un snapshot thread-safe du dernier scan fusionne (tous points)."""
    with lock:
        return list(scan_read_buf)


def get_latest_beacon_candidates():
    """
    Retourne un snapshot thread-safe des candidats-balises pre-filtres.

    Ces candidats ont deja passe :
      - filtre distance [POSE_MIN_DIST_MM, POSE_MAX_DIST_MM]
      - filtre qualite >= BEACON_QUAL_MIN (8)
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