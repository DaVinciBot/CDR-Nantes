#!/usr/bin/env python3
"""
main.py
Point d'entrée principal du robot pour la Coupe de France de Robotique.
"""
import time
import logging
from robot import Robot # On importera votre future grande classe Robot

# Configuration du logger pour voir ce qui se passe dans la console
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("MAIN")

# --- PARAMÈTRES DU MATCH ---
TEMPS_MATCH_SECONDES = 90.0

def main():
    logger.info("Démarrage du système...")

    # =================================================================
    # 1. INITIALISATION (Avant de poser le robot sur la table)
    # =================================================================
    # Idéalement, on lit un switch physique sur le robot pour savoir si on est Bleu ou Jaune
    couleur_equipe = "YELLOW" 
    
    # On crée LE robot (qui va lui-même allumer le Lidar, la Teensy, etc.)
    mon_robot = Robot(couleur_equipe)
    
    logger.info(f"Robot initialisé pour l'équipe {couleur_equipe}.")
    logger.info("En attente de la tirette de départ...")

    # =================================================================
    # 2. ATTENTE DU DÉPART (Sur la table)
    # =================================================================
    # Le code reste bloqué ici tant que la tirette n'est pas retirée
    mon_robot.attendre_tirette() 
    
    # --- LE MATCH COMMENCE ICI ---
    heure_debut = time.time()
    logger.info("MATCH LANCÉ !")

    # =================================================================
    # 3. LA BOUCLE DES 90 SECONDES (Le Match)
    # =================================================================
    try:
        while True:
            temps_ecoule = time.time() - heure_debut
            
            # Vérification stricte du chronomètre
            if temps_ecoule >= TEMPS_MATCH_SECONDES:
                logger.info("FIN DU TEMPS RÉGLEMENTAIRE !")
                break # On sort de la boucle !

            # C'est ici que toute la magie opère. Le robot lit ses capteurs, 
            # consulte sa stratégie, calcule son Pathfinding et bouge.
            mon_robot.update()
            
            # Petite pause pour ne pas surcharger le processeur (ex: 20 Hz)
            time.sleep(0.05) 

    except KeyboardInterrupt:
        logger.warning("Arrêt d'urgence demandé par l'utilisateur (Ctrl+C).")
        
    finally:
        # =================================================================
        # 4. ARRÊT TOTAL (Sécurité vitale)
        # =================================================================
        logger.info("Désactivation des systèmes...")
        mon_robot.stopper_tout()
        logger.info("Extinction terminée. Bon match ! ")

if __name__ == "__main__":
    main()