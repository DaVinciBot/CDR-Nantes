"""
terrain_jeu.py
Gère les dimensions, la couleur d'équipe et les obstacles fixes du plateau.
"""
from dataclasses import dataclass
from enum import Enum

# --- Constantes Dimensionnelles (en mm) ---
FIELD_WIDTH_MM = 3000
FIELD_HEIGHT_MM = 2000
ROBOT_RADIUS_MM = 120

@dataclass
class Obstacle:
    name: str
    x: float
    y: float
    width: float
    height: float

class TeamColor(Enum):
    BLUE = "BLUE"
    YELLOW = "YELLOW"

class Terrain:
    def __init__(self, team_color: str):
        try:
            self.team = TeamColor[team_color.upper()]
        except KeyError:
            self.team = TeamColor.BLUE
            
        self.WIDTH = FIELD_WIDTH_MM
        self.HEIGHT = FIELD_HEIGHT_MM
        self.ROBOT_RADIUS = ROBOT_RADIUS_MM
        self.obstacles = []
        
        self._load_static_obstacles()

    def _load_static_obstacles(self):
        """Définit les obstacles selon les règles (référentiel BLEU)."""
        self.add_obstacle("Grenier", x_blue=600, y_blue=1550, width=1800, height=450)
        self.add_obstacle("Nid Adverse", x_blue=0, y_blue=1550, width=600, height=450)

    def add_obstacle(self, name, x_blue, y_blue, width, height):
        """Ajoute un obstacle en appliquant la symétrie si on est JAUNE."""
        final_x = x_blue
        if self.team == TeamColor.YELLOW:
            final_x = self.WIDTH - (x_blue + width)
            
        self.obstacles.append(Obstacle(name, final_x, y_blue, width, height))