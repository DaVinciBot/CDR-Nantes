import struct
import math
import logging
import threading
import time

from terrain_jeu import Terrain
from pathfinder import PathFinder
from lidar import GestionnaireLidar

from loader import loader
from utils import init_robot
from strategy.actions import TypeAction
from strategy.strat_manager import StratManager
class Robot:
    def __init__(self, couleur_equipe: str):
        """Initialise le robot, ses capteurs, son cerveau et son hardware."""
        self.logger = logging.getLogger("ROBOT")
        self.logger.info(f"Initialisation du robot en mode {couleur_equipe}...")

        # 1. MÉMOIRE PARTAGÉE (Position sécurisée par un Lock)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lock = threading.Lock() # Le cadenas contre les race conditions

        # 2. PERCEPTION ET NAVIGATION (Ton code !)
        self.terrain = Terrain(couleur_equipe)
        self.cerveau = PathFinder(self.terrain)
        self.lidar = GestionnaireLidar(port_usb='/dev/ttyUSB0')

        # 3. HARDWARE ET COMMUNICATION (Leur code !)
        self.Messages = loader.load_class('usb_com', 'Messages')
        self.com, self.mode = init_robot(self.logger)
        
        # On attache le callback : quand la Teensy parle, ça met à jour notre mémoire
        self.com.add_callback(self._handle_position, self.Messages.UPDATE_ROLLING_BASIS.value)

        # 4. STRATÉGIE 
        self.strategie = StratManager(couleur_equipe)
        
        self.logger.info("Robot entièrement initialisé.")

    # =================================================================
    # --- MÉTHODES DE BAS NIVEAU (Callbacks & Envois) ---
    # =================================================================
    def _handle_position(self, data: bytes) -> None:
        """Callback appelé automatiquement par l'USB quand la Teensy envoie sa position."""
        if len(data) >= 24:
            x, y, theta = struct.unpack('<ddd', data[:24])
            # On met le cadenas le temps de modifier les variables
            with self.lock:
                self.x = x
                self.y = y
                self.theta = theta
        else:
            self.logger.warning(f"Message odométrie trop court: {len(data)} bytes")

    def _envoyer_position_moteurs(self, x, y, theta, description=""):
        """Envoie l'ordre de mouvement physique à la Teensy."""
        msg = self.Messages.SET_TARGET_POSITION.to_bytes()
        msg += struct.pack('<ddd', x, y, theta)
        self.com.send_bytes(msg)
        self.logger.debug(f"{description}: X={x:.1f}, Y={y:.1f}, θ={theta:.2f}")

    # =================================================================
    # --- MÉTHODES DE HAUT NIVEAU (Utilisées par le main.py) ---
    # =================================================================
    def attendre_tirette(self):
        """Met le code en pause tant que la tirette n'est pas retirée."""
        self.logger.info("Veuillez retirer la tirette pour démarrer...")
        # TODO: Remplacer ce time.sleep par la vraie lecture du pin GPIO de la tirette
        time.sleep(3) 
        self.logger.info("Tirette retirée !")

    def update(self):
        """LA grande boucle de décision. Appelée 20 fois par seconde."""
        
        # 1. LECTURE DE LA RÉALITÉ (Copie sécurisée de la position)
        with self.lock:
            rx, ry, rtheta = self.x, self.y, self.theta

        # 2. PERCEPTION : Où sont les méchants ?
        obstacles = self.lidar.obtenir_obstacles_absolus(rx, ry, rtheta)

        # ==========================================================
        # 3. STRATÉGIE : VRAIE LECTURE DE LA MACHINE À ÉTATS
        # ==========================================================
        action = self.strategie.get_action_actuelle()

        # Si la liste des actions est vide, le match est fini !
        if action is None:
            self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (Fin des missions)")
            return 

        # ==========================================================
        # 4. EXÉCUTION DE L'ACTION EN COURS
        # ==========================================================
        
        # --- CAS A : C'EST UN DÉPLACEMENT ---
        if action.type == TypeAction.DEPLACEMENT:
            objectif = {'x': action.cible_x, 'y': action.cible_y}
            distance_restante = math.hypot(action.cible_x - rx, action.cible_y - ry)
            
            if distance_restante < 50.0: 
                self.logger.info(f"Objectif {action.cible_x}, {action.cible_y} atteint !")
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (Arrivé)")
                self.strategie.valider_action_terminee() 
                return 

            chemin = self.cerveau.get_path({'x': rx, 'y': ry}, objectif, obstacles)

            if chemin and len(chemin) > 1:
                # TODO: Rendre le lookahead_index dynamique proportionnellement 
                # à la vitesse lue sur la Teensy pour éviter les oscillations.
                lookahead_index = min(4, len(chemin) - 1)
                
                cible_x, cible_y = chemin[lookahead_index]
                angle_cible = math.atan2(cible_y - ry, cible_x - rx)
                self._envoyer_position_moteurs(cible_x, cible_y, angle_cible, "Esquive IA")
                    
            else:
                self.logger.warning("Chemin bloqué ! Arrêt d'urgence.")
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (Bloqué)")

        # --- CAS B : C'EST UNE ACTION MÉCANIQUE ---
        elif action.type == TypeAction.ACTIONNEUR:
            self.logger.info(f"Activation de l'actionneur : {action.nom_actionneur}")
            
            # TODO: Implémenter un vrai mécanisme de feedback (callback ou lecture d'état)
            # Ex: if self.actionneurs.bras_est_en_position(): valider_action_terminee()
            # Pour l'instant, on valide instantanément pour tester la boucle logicielle.
            self.strategie.valider_action_terminee()

        # --- CAS C : C'EST UNE PAUSE ---
        elif action.type == TypeAction.ATTENTE:
            # Encapsulation respectée : le Robot ne lit plus la variable chrono !
            if self.strategie.demarrer_chrono_si_necessaire():
                self.logger.info(f"Attente de {action.temps_attente}s...")
                
            elif self.strategie.chrono_ecoule(action.temps_attente):
                self.strategie.valider_action_terminee()

    def stopper_tout(self):
        """Procédure de sécurité à la fin des 90 secondes."""
        self.logger.info("Arrêt des moteurs...")
        with self.lock:
            self._envoyer_position_moteurs(self.x, self.y, self.theta, "ARRET MATCH")
        
        self.logger.info("Coupure du LiDAR...")
        self.lidar.arreter()
        
        # TODO: self.actionneurs.ranger_bras()