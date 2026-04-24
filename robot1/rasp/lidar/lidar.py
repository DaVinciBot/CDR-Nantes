"""
lidar.py
Wrapper simplifié pour encapsuler la localisation et tracking adversaire.
"""

import logging
import threading
from typing import Optional, Tuple

try:
    from .lidar_logic import (
        get_corrected_pose,
        get_latest_opponent,
        stop_lidar_runtime,
    )
except ImportError:
    from lidar_logic import (
        get_corrected_pose,
        get_latest_opponent,
        stop_lidar_runtime,
    )


class LidarInterface:
    """
    Interface simplifiée pour accéder à la localisation fusionnée
    et au tracking adversaire.

    """

    def __init__(self, team_color: str = "BLUE", opponent_timeout_s: float = 0.70):
        self.logger     = logging.getLogger("LIDAR_INTERFACE")
        self.team_color = team_color.upper()
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

            if conf < 0.2:
                alpha = 0.85
            elif conf > 0.8:
                alpha = 0.25
            else:
                alpha = 0.85 - (conf - 0.2) / 0.6 * 0.60

            fused_x = (1.0 - alpha) * corrected.x + alpha * teensy_x
            fused_y = (1.0 - alpha) * corrected.y + alpha * teensy_y

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