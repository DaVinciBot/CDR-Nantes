"""
lidar.py
Wrapper simplifié pour encapsuler la localisation et tracking adversaire.

Fournit une interface unique pour :
- Fusion pose lidar (SVD Umeyama) + odométrie Teensy
- Tracking robot adverse
- Export pour pathfinding
"""

import logging
import math
import threading
from typing import Dict, List, Optional, Tuple

try:
    from .lidar_navigation_bridge import LidarNavigationBridge, OpponentTrack
    from .lidar_logic import (
        get_corrected_pose,
        get_latest_opponent,
        get_latest_scan_data,
        get_latest_beacon_candidates,
        stop_lidar_runtime,
    )
except ImportError:
    from lidar_navigation_bridge import LidarNavigationBridge, OpponentTrack
    from lidar_logic import (
        get_corrected_pose,
        get_latest_opponent,
        get_latest_scan_data,
        get_latest_beacon_candidates,
        stop_lidar_runtime,
    )


class LidarInterface:
    """
    Interface simplifiée pour accéder à la localisation fusionnée
    et au tracking adversaire.

    Usage:
        lidar = LidarInterface(team_color="BLUE")

        # Lors de chaque update robot
        x, y, theta, conf = lidar.get_fused_position(teensy_x, teensy_y, teensy_theta)
        opp = lidar.get_opponent()   # (x, y, conf) ou None
    """

    def __init__(self, team_color: str = "BLUE", opponent_timeout_s: float = 0.70):
        self.logger     = logging.getLogger("LIDAR_INTERFACE")
        self.team_color = team_color.upper()
        self._bridge    = LidarNavigationBridge(
            team_color=self.team_color,
            opponent_timeout_s=opponent_timeout_s,
        )
        self._lock = threading.Lock()
        self.logger.info(f"LidarInterface initialisé pour équipe {self.team_color}")

    # ── POSITION FUSIONNÉE ────────────────────────────────────────────────────

    def get_fused_position(
        self,
        teensy_x: float,
        teensy_y: float,
        teensy_theta: float,
    ) -> Tuple[float, float, float, float]:
        """
        Retourne la position fusionnée LiDAR + Teensy.

        Blend adaptatif basé sur la confiance SVD :
          - conf < 0.2  → alpha = 0.85 (85 % Teensy)
          - conf > 0.8  → alpha = 0.25 (75 % LiDAR)
          - intermédiaire → transition linéaire

        Returns:
            (x_mm, y_mm, theta_rad, confidence)
        """
        with self._lock:
            corrected = get_corrected_pose()

            if corrected is None or not corrected.is_localized:
                return teensy_x, teensy_y, teensy_theta, 0.0

            conf = corrected.confidence

            # Alpha adaptatif
            if conf < 0.2:
                alpha = 0.85
            elif conf > 0.8:
                alpha = 0.25
            else:
                alpha = 0.85 - (conf - 0.2) / 0.6 * 0.60

            fused_x = (1.0 - alpha) * corrected.x + alpha * teensy_x
            fused_y = (1.0 - alpha) * corrected.y + alpha * teensy_y

            # Theta toujours depuis l'IMU Teensy
            return fused_x, fused_y, teensy_theta, conf

    # ── ADVERSAIRE ────────────────────────────────────────────────────────────

    def get_opponent(self) -> Optional[Tuple[float, float, float]]:
        """
        Retourne la position du robot adverse si disponible.

        Returns:
            (x_mm, y_mm, confidence) ou None
        """
        with self._lock:
            opp = get_latest_opponent()
            if opp is None or opp.confidence < 0.1:
                return None
            return (opp.x, opp.y, opp.confidence)

    # ── OBSTACLES ────────────────────────────────────────────────────────────

    def get_obstacles(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
    ) -> List[Dict]:
        """
        Retourne les obstacles LiDAR pertinents pour le pathfinding.

        NOTE: Le LiDAR ne voit que :
          - Balises (exclues — pour localisation SVD uniquement)
          - Robot adverse (retourné via get_opponent() séparément)
          - Bruit/reflets isolés (ignorés)

        Les obstacles statiques (murs, caisses, grenier) sont définis
        dans terrain_jeu.py et ajoutés par robot.py.

        Returns:
            Liste vide [] en temps normal
            (LiDAR n'a pas d'obstacles pertinents pour pathfinding)
        """
        # Le LiDAR détecte principalement les balises et l'adversaire.
        # Les balises sont exclues (localisation SVD), l'adversaire est
        # via get_opponent(). Les points isolés sont du bruit.
        # Les vrais obstacles (murs, caisses) viennent de terrain_jeu.
        return []

    # ── DIAGNOSTIC ───────────────────────────────────────────────────────────

    def get_diagnostic_info(self) -> dict:
        """Informations de debug sur l'état de localisation."""
        with self._lock:
            pose = get_corrected_pose()

        if pose is None:
            return {
                "is_localized":    False,
                "nb_beacons":      0,
                "confidence":      0.0,
                "beacon_ids":      [],
                "last_update_time": 0.0,
            }

        return {
            "is_localized":    pose.is_localized,
            "nb_beacons":      len(pose.beacon_ids or []),
            "confidence":      pose.confidence,
            "beacon_ids":      pose.beacon_ids or [],
            "last_update_time": pose.last_update_time,
        }

    # ── ARRÊT ─────────────────────────────────────────────────────────────────

    def arreter(self) -> None:
        """Arrête proprement le thread LiDAR."""
        stop_lidar_runtime()
        self.logger.info("LiDAR arrêté.")