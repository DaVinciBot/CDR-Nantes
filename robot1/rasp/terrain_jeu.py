"""
terrain_jeu.py
Gère les dimensions, la couleur d'équipe et les obstacles fixes du plateau.
Centralise aussi tous les paramètres du LiDAR réel et les positions des balises.
"""
import math
from dataclasses import dataclass
from enum import Enum

# --- Constantes Dimensionnelles (en mm) ---
FIELD_WIDTH_MM  = 3000
FIELD_HEIGHT_MM = 2000
ROBOT_RADIUS_MM = 120

# --- Positions de départ (mm, rad) -----------------------------------------------
# Zones de départ CDR 2026 (symétrie X)
# Origine du repère : bas-droite (0,0) = coin bas-droit, (3000,2000) = coin haut-gauche
# 
#   Zone Jaune (Bas-Gauche):   X: 2550-3000mm (gauche), Y: 0-600mm,      centre: (2775, 300)
#   Zone Bleue (Haut-Gauche):  X: 2550-3000mm (gauche), Y: 1400-2000mm,  centre: (2775, 1700)
#
#   Table vue de dessus :
#   (0,3000) ───────────────────── (3000,3000)
#      │  ZONE BLEUE (1775, 2700)  │
#   (0,2000) ───────────────────── (3000,2000)  [Grenier y=1550-2000]
#      │  ZONE JAUNE (1775, 300)   │
#   (0,0)  ────────────────────── (3000,0)
#
# Référence BLEU → symétrie X pour JAUNE : x_jaune = 3000 - x_bleu
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

    def get_static_obstacles(self) -> list:
        """
        Retourne les obstacles statiques comme liste de tuples (x, y) compatibles
        avec PathFinder.create_dynamic_grid().

        NOTE : en pratique ces obstacles sont déjà gravés dans la static_grid du
        PathFinder à l'init — on retourne donc [] pour éviter le double-comptage.
        La méthode existe pour rendre l'interface explicite et supprimer le hasattr
        trompeur dans robot.py.
        """
        # Les obstacles statiques sont pre-calculés dans PathFinder._build_static_grid().
        # Ne pas les repasser à create_dynamic_grid(), sinon double-inflation.
        return []

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

    # Positions centre des balises (x, y) en mm pour l'équipe BLEUE
    # Origine = coin bas-droite du terrain.
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
            x = FIELD_WIDTH_MM - x   # symétrie X (gauche ↔ droite)

        return (x, y)