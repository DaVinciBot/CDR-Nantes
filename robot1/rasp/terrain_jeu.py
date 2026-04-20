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
# Référentiel BLEU (origine bas-gauche). Ajuster selon le placement réel sur table.
# La position JAUNE est déduite par symétrie : x_jaune = 3000 - x_bleu, θ inversé.
#
#   Table vue de dessus :
#   (0,2000) ───────────────────── (3000,2000)
#      │  [ZONE JAUNE]  │  [ZONE BLEUE]  │
#   (0,0)  ────────────────────── (3000,0)
#
# ⚠  À AJUSTER selon le règlement CDR 2026 et le côté de départ retenu.
_BLUE_START_X     = 200.0    # mm
_BLUE_START_Y     = 200.0    # mm
_BLUE_START_THETA = 0.0      # rad  (0 = pointe vers +X)

START_POSITIONS = {
    "BLUE": (
        _BLUE_START_X,
        _BLUE_START_Y,
        _BLUE_START_THETA,
    ),
    "YELLOW": (
        FIELD_WIDTH_MM - _BLUE_START_X,          # symétrie X
        _BLUE_START_Y,                            # Y identique
        (math.pi - _BLUE_START_THETA) % (2 * math.pi),   # angle miroir
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
            final_y = self.HEIGHT - (y_blue + height)

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
    # Origine = coin bas-gauche du terrain.
    BEACONS = {
        1: (3050.0, 1950.0),    # haut droite (hors terrain)
        2: (3050.0,   50.0),    # bas droite  (hors terrain)
        3: (  -50.0, 1000.0),   # milieu gauche (hors terrain)
    }

    @classmethod
    def get_beacon(cls, beacon_id: int, team_color: str = "BLUE") -> tuple:
        """Retourne les coordonnées d'une balise après symétrie si nécessaire."""
        if beacon_id not in cls.BEACONS:
            raise ValueError(f"Balise {beacon_id} non trouvée")

        x, y = cls.BEACONS[beacon_id]

        if team_color.upper() == "YELLOW":
            x = FIELD_WIDTH_MM - x   # symétrie axe vertical

        return (x, y)