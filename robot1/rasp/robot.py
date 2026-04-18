import struct
import math
import logging
import threading
import time

from terrain_jeu import Terrain
from pathfinder import PathFinder
from lidar.lidar import LidarInterface
from lidar.lidar_logic import (
    update_teensy_pose,
    get_corrected_pose,
    should_send_correction_to_teensy,
)

from loader import loader
from utils import init_robot
from strategy.strategy_actions import TypeAction
from strategy.strategy_strat_manager import StratManager

class Robot:
    def __init__(self, couleur_equipe: str):
        """Initialise le robot, ses capteurs, son cerveau et son hardware."""
        self.logger = logging.getLogger("ROBOT")
        self.logger.info(f"Initialisation du robot en mode {couleur_equipe}...")
        
        self.couleur = couleur_equipe
        # 1. MÉMOIRE PARTAGÉE (Position sécurisée par un Lock)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lock = threading.Lock() # Le cadenas contre les race conditions
        
        # Timestamp pour throttle envoi correction vers Teensy
        self._last_correction_sent_time = 0.0

        # 2. PERCEPTION ET NAVIGATION (Ton code !)
        self.terrain = Terrain(couleur_equipe)
        self.cerveau = PathFinder(self.terrain)
        self.lidar = LidarInterface(team_color=couleur_equipe)

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
            
            # Signal LiDAR thread with current Teensy pose for SVD beacon window prediction
            update_teensy_pose(x, y, theta)
        else:
            self.logger.warning(f"Message odométrie trop court: {len(data)} bytes")

    def _envoyer_position_moteurs(self, x, y, theta, description=""):
        """Envoie l'ordre de mouvement physique à la Teensy."""
        msg = self.Messages.SET_TARGET_POSITION.to_bytes()
        msg += struct.pack('<ddd', x, y, theta)
        self.com.send_bytes(msg)
        self.logger.debug(f"{description}: X={x:.1f}, Y={y:.1f}, θ={theta:.2f}")

    def _apply_complementary_filter(self, lidar_x: float, lidar_y: float,
                                    teensy_x: float, teensy_y: float,
                                    lidar_confidence: float) -> tuple:
        """
        Filtre complémentaire adaptatif : fusion LiDAR + Teensy basée sur confiance.
        
        Alpha varie avec la confiance LiDAR :
        - Confiance basse (< 0.3) → 85% Teensy (alpha=0.85)
        - Confiance haute (> 0.8) → 75% LiDAR (alpha=0.25)
        - Transition linéaire entre les deux
        
        Args:
            lidar_x/y: Position LiDAR (m, corrigée par SVD)
            teensy_x/y: Position Teensy (mm, odométrie brute)
            lidar_confidence: Confiance du LiDAR [0.0, 1.0]
        
        Returns:
            (x_fused_mm, y_fused_mm, alpha)
        """
        # Calculer alpha adaptatif en fonction de la confiance
        if lidar_confidence < 0.2:
            alpha = 0.85  # Pas confiance → 85% Teensy, 15% LiDAR
        elif lidar_confidence > 0.8:
            alpha = 0.25  # Très confiance → 75% LiDAR, 25% Teensy
        else:
            # Transition linéaire entre 0.2 et 0.8
            alpha = 0.85 - (lidar_confidence - 0.2) / 0.6 * 0.60
        
        # Fusion simple blend
        x_fused = (1.0 - alpha) * lidar_x + alpha * teensy_x
        y_fused = (1.0 - alpha) * lidar_y + alpha * teensy_y
        
        return x_fused, y_fused, alpha

    def _send_odometry_correction(self, corrected_x: float, corrected_y: float,
                                  corrected_theta: float):
        """
        Envoie la pose corrigée vers Teensy pour reset odométrie.
        
        Args:
            corrected_x/y: Position corrigée en mm
            corrected_theta: Theta corrigé en rad
        """
        msg = self.Messages.SET_ODOMETRIE.to_bytes()
        msg += struct.pack('<ddd', corrected_x, corrected_y, corrected_theta)
        self.com.send_bytes(msg)
        self.logger.info(
            f"Correction odométrie envoyée: X={corrected_x:.1f}, "
            f"Y={corrected_y:.1f}, θ={corrected_theta:.3f}"
        )

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
        """La grande boucle de décision. Appelée 20 fois par seconde."""
        
        # 1. LECTURE DE LA POSITION TEENSY (Copie sécurisée)
        with self.lock:
            teensy_x, teensy_y, teensy_theta = self.x, self.y, self.theta

        # 1b. MISE À JOUR POSITION TEENSY POUR CORRECTION LIDAR
        update_teensy_pose(teensy_x, teensy_y, teensy_theta)
        
        # 2. FUSION LIDAR + TEENSY (SVD + Filtre complémentaire adaptatif)
        corrected_pose = get_corrected_pose()
        
        if corrected_pose is not None:
            # Position corrigée disponible depuis SVD
            lidar_x = corrected_pose.x
            lidar_y = corrected_pose.y
            lidar_confidence = corrected_pose.confidence
            
            # Appliquer filtre complémentaire adaptatif
            rx, ry, alpha = self._apply_complementary_filter(
                lidar_x, lidar_y, teensy_x, teensy_y, lidar_confidence
            )
            
            # Theta toujours du Teensy IMU (pas de correction de cap)
            rtheta = teensy_theta
            
            self.logger.debug(
                f"Pose fusionée (corrected): ({rx:.1f}, {ry:.1f}, {rtheta:.3f}) | "
                f"lidar_conf={lidar_confidence:.2f}, alpha={alpha:.2f}"
            )
            
            # VÉRIFIER SI CORRECTION DOIT ÊTRE RENVOYÉE À TEENSY
            if should_send_correction_to_teensy():
                # Vérifier throttle local (sécurité double)
                current_time = time.time()
                if current_time - self._last_correction_sent_time >= 1.0:
                    self._send_odometry_correction(rx, ry, rtheta)
                    self._last_correction_sent_time = current_time
        else:
            # Pas de position corrigée → utiliser fusion simple avec ancien code
            rx, ry, rtheta, lidar_confidence = self.lidar.get_fused_position(
                teensy_x, teensy_y, teensy_theta
            )
            self.logger.debug(
                f"Pose fused (simple): ({rx:.1f}, {ry:.1f}, {rtheta:.3f}) | "
                f"lidar_conf={lidar_confidence:.2f}"
            )

        # 3. PERCEPTION : Où sont les obstacles/adversaires ?
        obstacles = self.lidar.get_obstacles(rx, ry, rtheta)
        
        # 3b. INTÉGRATION ADVERSAIRE: Ajouter robot adverse aux obstacles
        opponent_data = self.lidar.get_opponent()
        if opponent_data is not None:
            opp_x, opp_y, opp_conf = opponent_data
            # Seuil de confiance pour intégrer aux obstacles
            if opp_conf > 0.1:
                obstacles.append({
                    'x': opp_x,
                    'y': opp_y,
                    'type': 'opponent',
                    'confidence': opp_conf,
                    'radius_mm': 110,  # Rayon nominal robot adverse
                })
                self.logger.debug(
                    f"Adversaire détecté: ({opp_x:.1f}, {opp_y:.1f}) "
                    f"confiance={opp_conf:.2f}"
                )

        # ==========================================================
        # 4. STRATÉGIE : VRAIE LECTURE DE LA MACHINE À ÉTATS
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
                # 9.1. LOOKAHEAD DYNAMIQUE: Adapter lookahead selon distance au but
                dist_to_target = math.sqrt((objectif['x'] - rx)**2 + (objectif['y'] - ry)**2)
                
                if dist_to_target > 500:  # Loin du but: lookahead court
                    lookahead_index = min(4, len(chemin) - 1)
                elif dist_to_target < 200:  # Près du but: lookahead très court
                    lookahead_index = min(1, len(chemin) - 1)
                else:  # Distance intermédiaire
                    lookahead_index = min(2, len(chemin) - 1)
                
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
        
        # 9.2. ACTUATEURS: Replier le bras (Si module actionneurs chargé)
        if hasattr(self, 'actionneurs'):
            try:
                self.actionneurs.ranger_bras()
                self.logger.info("Bras replié")
            except Exception as e:
                self.logger.error(f"Erreur fermeture bras: {e}")
        else:
            self.logger.warning("Module actionneurs non chargé")