#!/usr/bin/env python3
"""Programme de test simplifié - compatible simulation et hardware."""
import struct
import math
import logging
import sys
import time
from pathlib import Path
from loader import loader

# =================================================================
# --- AJOUT: Import de mes modules d'IA et de LiDAR ---
from terrain_jeu import Terrain
from pathfinder import PathFinder
from lidar import LidarInterface
# =================================================================

# Créer un logger simple
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

Messages = loader.load_class('usb_com', 'Messages')

#A mettre partout dans les codes python
# Initialisation automatique (simulation ou hardware)
sys.path.insert(0, str(Path(__file__).parent))
from utils import init_robot

com, mode = init_robot(logger)

# =================================================================
# --- AJOUT: Création de variables globales ---
# (Pour que mon PathFinder puisse savoir où se trouve le robot à tout moment)
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
# =================================================================


def handle_position(data: bytes) -> None:
    """Callback pour recevoir la position du robot."""
    global robot_x, robot_y, robot_theta 
    if len(data) >= 24:
        x, y, theta = struct.unpack('<ddd', data[:24])

        # =================================================================
        # --- AJOUT: Mise à jour de la position pour le cerveau ---
        robot_x = x
        robot_y = y
        robot_theta = theta
        # =================================================================

        logger.info(f" Position robot: X={x:.2f}mm, Y={y:.2f}mm, θ={theta:.4f}rad")
    else:
        logger.warning(f" Message trop court: {len(data)} bytes")

def send_position(x, y, theta, com, description=""):
    """Envoie une position cible au robot."""
    msg = Messages.SET_TARGET_POSITION.to_bytes()
    msg += struct.pack('<ddd', x, y, theta)
    com.send_bytes(msg)
    logger.info(f" {description}: X={x}mm, Y={y}mm, θ={theta}rad")

com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)


def main():
    logger.info("Démarrage du programme de test...")
    
    # Ligne droite horizontale
    send_position(200, 0, 0, com, "Ligne droite horizontale") 
    time.sleep(5)  # Attendre 5 secondes
    # Ligne droite verticale 
    send_position(0, 200, 0, com, "Ligne droite verticale")
    time.sleep(5)  # Attendre 5 secondes
    # Carré
    send_position(200, 0, 0, com, "Carré - Coin 1")
    send_position(200, -200, 0, com, "Carré - Coin 2")
    send_position(0, -200, 0, com, "Carré - Coin 3")
    send_position(0, 0, 0, com, "Carré - Retour origine")
    time.sleep(5)  # Attendre 5 secondes
    # Triangle rectangle
    send_position(200, 0, 0, com, "Triangle - Côté 1")
    send_position(200, 200, 0, com, "Triangle - Côté 2")
    send_position(0, 0, 0, com, "Triangle - Retour origine")
    time.sleep(5)  # Attendre 5 secondes
    # Cercle
    logger.info("Démarrage du cercle...")
    nb_pas = 1000
    rayon = 100

    """for i in range(nb_pas + 1):
        angle = i * 2 * math.pi / nb_pas
        x = rayon * math.cos(angle)
        y = rayon * math.sin(angle)
        theta = angle + math.pi / 2
        send_position(x, y, theta, com, f"Cercle - Point {i}/{nb_pas}")
        time.sleep(0.9)  # Petit délai pour ne pas saturer la communication"""
    send_position(0,0,90,com,"Orientation 90 degrés")

    # =================================================================
    # --- AJOUT: TEST DE L'INTELLIGENCE ET DE L'ESQUIVE ---
    # =================================================================
    logger.info("\nLancement de l'IA (Pathfinding A* + LiDAR USB)...")

    # 1. Initialisation de mes outils
    mon_terrain = Terrain("YELLOW")
    cerveau = PathFinder(mon_terrain)
    mon_lidar = GestionnaireLidar(port_usb='/dev/ttyUSB0')

    # 2. Définition d'un objectif pour tester l'esquive
    objectif = {'x': 1500, 'y': 1000} 

    try:
        # 3. Boucle temps réel
        while True:
            # Récupère les obstacles depuis mon code LiDAR
            obstacles = mon_lidar.obtenir_obstacles_absolus(robot_x, robot_y, robot_theta)

            # Calcule la trajectoire d'esquive avec mon code A*
            chemin = cerveau.get_path({'x': robot_x, 'y': robot_y}, objectif, obstacles)

            if chemin and len(chemin) > 1:
                # 1. LOOKAHEAD : On vise 4 cases plus loin (soit 20cm) pour éviter les saccades
                lookahead_index = min(4, len(chemin) - 1)
                cible_x, cible_y = chemin[lookahead_index]

                # 2. ORIENTATION : Calcul de l'angle vers le point visé
                dx = cible_x - robot_x
                dy = cible_y - robot_y
                angle_cible = math.atan2(dy, dx)

                # Envoie le prochain point d'esquive aux moteurs avec la bonne orientation
                send_position(cible_x, cible_y, angle_cible, com, "Esquive IA A*")
                
            elif chemin and len(chemin) <= 1:
                logger.info(" IA : Objectif atteint !")
                break

            time.sleep(0.1) # L'IA réfléchit 10 fois par seconde

    except KeyboardInterrupt:
        logger.info("Arrêt de l'IA par l'utilisateur.")
    finally:
        mon_lidar.arreter() # Je coupe proprement le moteur du capteur
    # =================================================================

    logger.info(" Programme terminé !")

if __name__ == "__main__":
    main()
