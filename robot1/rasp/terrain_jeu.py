"""
terrain_jeu.py
Gère les dimensions, la couleur d'équipe et les obstacles fixes du plateau.
Centralise aussi tous les paramètres du LiDAR réel et les positions des balises.
"""
import math
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


# ==============================================================================
# BALISES - Source unique pour positions (référentiel principal du plateau) ===
# ==============================================================================

class BeaconLayout:
    """
    Positions fixes des balises de localisation (calibré pour le terrain réel 3000x2000)
    Référentiel BLEU - symétrie appliquée automatiquement en JAUNE
    """
    BEACON_SIZE_MM = 100.0  # Taille des balises (10x10 cm)

    # Positions centre des balises en mm (x, y) pour l'équipe BLEUE (origine = 0,0 en bas à gauche)
    BEACONS = {
        1: (3050.0, 1950.0),    # Balise 1 - haut droite (côté départ)
        2: (3050.0, 50.0),      # Balise 2 - bas droite (côté départ)
        3: (-50.0, 1000.0)      # Balise 3 - milieu de l'autre segment
    }
    
    @classmethod
    def get_beacon(cls, beacon_id: int, team_color: str = "BLUE") -> tuple:
        """Retourne les coordonnées d'une balise après symétrie si nécessaire."""
        if beacon_id not in cls.BEACONS:
            raise ValueError(f"Balise {beacon_id} non trouvée")
        
        x, y = cls.BEACONS[beacon_id]
        
        # Appliquer symétrie si équipe JAUNE
        if team_color.upper() == "YELLOW":
            x = FIELD_WIDTH_MM - x
        
        return (x, y)