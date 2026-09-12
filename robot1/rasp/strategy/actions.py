"""
actions.py
Définition des types d'actions possibles pour le robot.
"""
from enum import Enum
from dataclasses import dataclass

class TypeAction(Enum):
    DEPLACEMENT = "DEPLACEMENT" # Utilise le PathFinder et les moteurs
    ACTIONNEUR = "ACTIONNEUR"   # Utilise les bras, pinces, pompes
    ATTENTE = "ATTENTE"         # Fait une pause (ex: attendre que le bras se lève)

@dataclass
class Action:
    type: TypeAction
    cible_x: float = 0.0
    cible_y: float = 0.0
    nom_actionneur: str = ""
    temps_attente: float = 0.0