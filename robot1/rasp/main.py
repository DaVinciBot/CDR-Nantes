"""
main.py
Point d'entrée principal du robot pour la Coupe de France de Robotique.
"""
import logging
import threading

from robot1.rasp.app import Robot, lire_couleur_equipe
from robot1.rasp.clock import RealClock
from robot1.rasp.telemetry.dashboard import RobotDashboard

# Configuration du logger pour voir ce qui se passe dans la console
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("MAIN")

# --- PARAMÈTRES DU MATCH ---
# 100 s: Eurobot regulation match duration. The code said 90 s until 11/09/2026.
TEMPS_MATCH_SECONDES = 100.0

def _executer_match(
    interface: RobotDashboard,
    stop_event: threading.Event,
) -> None:
    mon_robot = None
    erreur = False
    match_termine = False

    try:
        logger.info("Démarrage du système...")

        # Chemin de production : horloge murale. C'est le seul endroit qui la
        # choisit, et c'est ce qui permet à sim2d d'en fournir une autre sans
        # toucher au robot (voir robot1/rasp/clock.py).
        clock = RealClock()

        # =================================================================
        # 1. INITIALISATION (Avant de poser le robot sur la table)
        # =================================================================
        # Idéalement, on lit un switch physique sur le robot pour savoir si on est Bleu ou Jaune
        couleur_equipe = lire_couleur_equipe()

        # On crée LE robot (qui va lui-même allumer le Lidar, la Teensy, etc.)
        mon_robot = Robot(couleur_equipe, clock=clock)
        interface.set_data_provider(mon_robot.dashboard_snapshot)

        logger.info(f"Robot initialisé pour l'équipe {couleur_equipe}.")
        if stop_event.is_set():
            return

        # =================================================================
        # 2. ATTENTE DU DÉPART (Sur la table)
        # =================================================================
        # Le code reste bloqué ici tant que la tirette n'est pas retirée
        interface.set_status("En attente de la tirette")
        if not mon_robot.attendre_tirette(stop_event):
            return

        # --- LE MATCH COMMENCE ICI ---
        heure_debut = clock.now()
        logger.info("MATCH LANCÉ !")
        interface.set_status("Match en cours")

        # =================================================================
        # 3. LA BOUCLE DES 100 SECONDES (Le Match)
        # =================================================================
        while not stop_event.is_set():
            temps_ecoule = clock.now() - heure_debut
            interface.set_match_time(
                temps_ecoule,
                TEMPS_MATCH_SECONDES - temps_ecoule,
            )

            # Vérification stricte du chronomètre
            if temps_ecoule >= TEMPS_MATCH_SECONDES:
                logger.info("FIN DU TEMPS RÉGLEMENTAIRE !")
                match_termine = True
                break # On sort de la boucle !

            # C'est ici que toute la magie opère. Le robot lit ses capteurs,
            # consulte sa stratégie, calcule son Pathfinding et bouge.
            mon_robot.update()

            # Petite pause pour ne pas surcharger le processeur (ex: 20 Hz)
            clock.sleep(0.05)

        if stop_event.is_set():
            interface.set_status("Arrêt demandé")
        else:
            interface.set_status("Match terminé")

    except Exception:
        erreur = True
        interface.set_status("Erreur système")
        logger.exception("Erreur pendant l'exécution du robot.")
    finally:
        # =================================================================
        # 4. ARRÊT TOTAL (Sécurité vitale)
        # =================================================================
        if mon_robot is not None:
            logger.info("Désactivation des systèmes...")
            try:
                mon_robot.stopper_tout()
            except Exception:
                erreur = True
                interface.set_status("Erreur à l'arrêt")
                logger.exception("Erreur pendant l'arrêt des systèmes.")
            else:
                logger.info("Extinction terminée. Bon match ! ")
        if not erreur:
            interface.set_status(
                "Match terminé" if match_termine else "Système arrêté"
            )


def main():
    interface = RobotDashboard()
    stop_event = threading.Event()
    worker = threading.Thread(
        target=_executer_match,
        args=(interface, stop_event),
        name="boucle-robot",
    )
    worker.start()

    try:
        interface.root.mainloop()
    except KeyboardInterrupt:
        logger.warning("Arrêt demandé par l'utilisateur (Ctrl+C).")
    finally:
        stop_event.set()
        interface.close()
        worker.join()

if __name__ == "__main__":
    main()