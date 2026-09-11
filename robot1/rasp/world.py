"""Table geometry: dimensions, team color, fixed obstacles and beacon layout.

This module holds NO LiDAR link parameter, whatever its header claimed until
11/09/2026: the serial port lives in config.json through vision/lidar_config.py,
and the processing thresholds in vision/detection.py and vision/localization.py.
The only thing here the LiDAR consumes is BeaconLayout.
"""
import math
from dataclasses import dataclass
from enum import Enum

# --- Constantes Dimensionnelles (en mm) ---
FIELD_WIDTH_MM  = 3000
FIELD_HEIGHT_MM = 2000

# WARNING: INDEPENDENT homonym of the ROBOT_RADIUS in
# robot1/teensy_moteur/include/config.h (160.0 mm).
# Here: obstacle inflation radius for the A* (nav/pathfinder.py), a planning
# margin. There: the physical center-to-wheel-axis distance, used by the
# holonomic kinematics and the rotational odometry.
# They have no reason to be equal. NEVER "align" them.
ROBOT_RADIUS_MM = 120

# --- Positions de départ (mm, rad) -----------------------------------------------
# Frame as the code uses it: origin at one corner of the table, X in
# [0, 3000] along the long side, Y in [0, 2000], theta=0 pointing to +X.
# The BeaconLayout beacons deliberately fall outside these bounds
# (x = -50 and x = 3050): they stand around the table, not on it.
#
#   Table seen from above:
#   (0,2000) ─────────────────────────────────── (3000,2000)
#      │  Nid adverse  │      Grenier            │   y = 1550..2000
#      │               │                         ● BLUE (2775, 1700)
#      │                                         │
#   (0,0) ───────────────────────────────────── (3000,0)
#
# BLUE is the reference, YELLOW mirrors on X: x_yellow = 3000 - x_blue, Y kept.
# The YELLOW start therefore lands at (225, 1700), not at the bottom of the table.
#
# TO BE CONFIRMED on the real table: which physical corner carries the origin,
# and whether the YELLOW start really sits on the same Y side as the BLUE one.
# The code is the only source today, it was never checked against the table
# (doc_ref/TODO.md).
_BLUE_START_X     = 2775.0    # mm  (centre X zone gauche)
_BLUE_START_Y     = 1700.0    # mm  (haut du terrain)
_BLUE_START_THETA = 0.0       # rad  (0 = pointe vers +X)

START_POSITIONS = {
    "BLUE": (
        _BLUE_START_X,
        _BLUE_START_Y,
        _BLUE_START_THETA,
    ),
    "YELLOW": (
        FIELD_WIDTH_MM - _BLUE_START_X,          # symétrie X : 3000 - 2775 = 225
        _BLUE_START_Y,                           # Y inchangé
        math.pi - _BLUE_START_THETA,             # angle miroir
    ),
}


@dataclass
class Obstacle:
    name: str
    x: float
    y: float
    width: float
    height: float


class TeamColor(Enum):
    BLUE   = "BLUE"
    YELLOW = "YELLOW"


class Terrain:
    def __init__(self, team_color: str):
        try:
            self.team = TeamColor[team_color.upper()]
        except KeyError:
            self.team = TeamColor.BLUE

        self.WIDTH        = FIELD_WIDTH_MM
        self.HEIGHT       = FIELD_HEIGHT_MM
        self.ROBOT_RADIUS = ROBOT_RADIUS_MM
        self.obstacles    = []

        self._load_static_obstacles()

    def _load_static_obstacles(self):
        """Définit les obstacles selon les règles (référentiel BLEU)."""
        self.add_obstacle("Grenier",      x_blue=600, y_blue=1550, width=1800, height=450)
        self.add_obstacle("Nid Adverse",  x_blue=0,   y_blue=1550, width=600,  height=450)

    def add_obstacle(self, name, x_blue, y_blue, width, height):
        """Ajoute un obstacle en appliquant la symétrie si on est JAUNE."""
        final_x = x_blue
        final_y = y_blue
        if self.team == TeamColor.YELLOW:
            final_x = self.WIDTH  - (x_blue + width)

        self.obstacles.append(Obstacle(name, final_x, final_y, width, height))

    # No get_static_obstacles() here: fixed obstacles are burnt once and for
    # all into PathFinder._build_static_grid(). Feeding them again to
    # create_dynamic_grid() would inflate them twice. The method existed,
    # returned a hardcoded [] and had no caller: removed on 11/09/2026.

    def get_start_position(self) -> tuple:
        """Retourne (x_mm, y_mm, theta_rad) de départ pour la couleur d'équipe."""
        return START_POSITIONS[self.team.value]


# ==============================================================================
# BALISES — Source unique pour positions (référentiel principal du plateau) ====
# ==============================================================================

class BeaconLayout:
    """
    Positions fixes des balises de localisation (calibré pour le terrain réel 3000x2000).
    Référentiel BLEU — symétrie appliquée automatiquement en JAUNE.
    """
    BEACON_SIZE_MM = 100.0

    # Beacon center positions (x, y) in mm for the BLUE team.
    # Which physical corner carries the origin is unverified, see the frame
    # note at the top of this file.
    BEACONS = {
        1: (3050.0, 1950.0),    # haut droite (hors terrain)
        2: (3050.0,   50.0),    # bas droite  (hors terrain)
        3: (  -50.0, 1000.0),   # milieu gauche (hors terrain)
    }

    @classmethod
    def get_beacon(cls, beacon_id: int, team_color: str = "BLUE") -> tuple:
        """Retourne les coordonnées d'une balise après symétrie X si nécessaire."""
        if beacon_id not in cls.BEACONS:
            raise ValueError(f"Balise {beacon_id} non trouvée")

        x, y = cls.BEACONS[beacon_id]

        if team_color.upper() == "YELLOW":
            x = FIELD_WIDTH_MM - x   # symétrie X (gauche <-> droite)

        return (x, y)