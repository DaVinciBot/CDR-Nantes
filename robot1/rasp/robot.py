import struct
import math
import logging
import threading
import time

from terrain_jeu import Terrain
from pathfinder import PathFinder
from lidar.lidar import LidarInterface
from lidar.lidar_logic import (
    set_team_color,              # FIX : chargement balises dès __init__
    update_teensy_pose,
    get_corrected_pose,
    should_send_correction_to_teensy,
    stop_lidar_runtime,          # FIX : import pour stopper_tout
)

from loader import loader
from utils import init_robot
from strategy.strategy_actions import TypeAction
from strategy.strategy_strat_manager import StratManager


class Robot:
    def __init__(self, couleur_equipe: str):
        self.logger = logging.getLogger("ROBOT")
        self.logger.info(f"Initialisation du robot en mode {couleur_equipe}...")

        self.couleur = couleur_equipe
        self.x       = 0.0
        self.y       = 0.0
        self.theta   = 0.0
        self.lock    = threading.Lock()

        self._last_correction_sent_time = 0.0

        # 1. Charger les positions des balises AVANT tout le reste
        #    (avant LidarInterface, avant start_lidar_thread)
        set_team_color(couleur_equipe)
        self.logger.info(f"Balises LiDAR chargées pour équipe {couleur_equipe}.")

        # 2. Terrain et pathfinding
        self.terrain  = Terrain(couleur_equipe)
        self.cerveau  = PathFinder(self.terrain)

        # 3. Interface LiDAR (démarre le thread d'acquisition en interne)
        self.lidar    = LidarInterface(team_color=couleur_equipe)

        self.Messages      = loader.load_class('usb_com', 'Messages')
        self.com, self.mode = init_robot(self.logger)

        self.com.add_callback(
            self._handle_position,
            self.Messages.UPDATE_ROLLING_BASIS.value,
        )

        self.strategie = StratManager(couleur_equipe)
        self.logger.info("Robot entièrement initialisé.")

    # ── BAS NIVEAU ────────────────────────────────────────────────────────────

    def _handle_position(self, data: bytes) -> None:
        """Callback USB — mise à jour odométrie Teensy."""
        if len(data) >= 24:
            x, y, theta = struct.unpack('<ddd', data[:24])
            with self.lock:
                self.x     = x
                self.y     = y
                self.theta = theta
            # Signal LiDAR thread pour prédiction fenêtres SVD
            update_teensy_pose(x, y, theta)
        else:
            self.logger.warning(
                f"Message odométrie trop court: {len(data)} bytes"
            )

    def _envoyer_position_moteurs(
        self, x: float, y: float, theta: float, description: str = ""
    ) -> None:
        msg  = self.Messages.SET_TARGET_POSITION.to_bytes()
        msg += struct.pack('<ddd', x, y, theta)
        self.com.send_bytes(msg)
        self.logger.debug(f"{description}: X={x:.1f}, Y={y:.1f}, θ={theta:.2f}")

    def _apply_complementary_filter(
        self,
        lidar_x: float,
        lidar_y: float,
        teensy_x: float,
        teensy_y: float,
        lidar_confidence: float,
    ) -> tuple:
        """
        Filtre complémentaire adaptatif LiDAR + Teensy.

        Alpha varie avec la confiance :
          - conf < 0.2 → alpha = 0.85 (85 % Teensy)
          - conf > 0.8 → alpha = 0.25 (75 % LiDAR)
          - intermédiaire → transition linéaire

        Returns:
            (x_fused_mm, y_fused_mm, alpha)
        """
        if lidar_confidence < 0.2:
            alpha = 0.85
        elif lidar_confidence > 0.8:
            alpha = 0.25
        else:
            alpha = 0.85 - (lidar_confidence - 0.2) / 0.6 * 0.60

        x_fused = (1.0 - alpha) * lidar_x + alpha * teensy_x
        y_fused = (1.0 - alpha) * lidar_y + alpha * teensy_y

        return x_fused, y_fused, alpha

    def _send_odometry_correction(
        self,
        corrected_x: float,
        corrected_y: float,
        corrected_theta: float,
    ) -> None:
        """Envoie la pose corrigée vers la Teensy (reset odométrie)."""
        msg  = self.Messages.SET_ODOMETRIE.to_bytes()
        msg += struct.pack('<ddd', corrected_x, corrected_y, corrected_theta)
        self.com.send_bytes(msg)
        self.logger.info(
            f"Correction odométrie: X={corrected_x:.1f}, "
            f"Y={corrected_y:.1f}, θ={corrected_theta:.3f}"
        )

    # ── HAUT NIVEAU ───────────────────────────────────────────────────────────

    def attendre_tirette(self) -> None:
        """Met le code en pause jusqu'au retrait de la tirette."""
        self.logger.info("En attente de la tirette...")
        # TODO: remplacer par lecture GPIO réel
        time.sleep(3)
        self.logger.info("Tirette retirée !")

    def update(self) -> None:
        """Boucle de décision — appelée 20 fois/seconde."""

        # 1. Lecture position Teensy
        with self.lock:
            teensy_x, teensy_y, teensy_theta = self.x, self.y, self.theta

        # 2. Signal LiDAR (double appel intentionnel pour le cas où
        #    _handle_position n'a pas encore été déclenché ce cycle)
        update_teensy_pose(teensy_x, teensy_y, teensy_theta)

        # 3. Fusion SVD + filtre complémentaire
        corrected_pose = get_corrected_pose()

        if corrected_pose is not None and corrected_pose.is_localized:
            rx, ry, alpha = self._apply_complementary_filter(
                corrected_pose.x,
                corrected_pose.y,
                teensy_x,
                teensy_y,
                corrected_pose.confidence,
            )
            rtheta = teensy_theta  # Cap toujours depuis IMU

            self.logger.debug(
                f"Pose fusée (SVD): ({rx:.1f}, {ry:.1f}, {rtheta:.3f}) | "
                f"conf={corrected_pose.confidence:.2f} alpha={alpha:.2f}"
            )

            # Envoi correction vers Teensy si conditions réunies
            if should_send_correction_to_teensy():
                now = time.time()
                if now - self._last_correction_sent_time >= 1.0:
                    self._send_odometry_correction(rx, ry, rtheta)
                    self._last_correction_sent_time = now
        else:
            # Fallback : fusion simple avec ancien wrapper
            rx, ry, rtheta, _ = self.lidar.get_fused_position(
                teensy_x, teensy_y, teensy_theta
            )

        # 4. Obstacles du terrain (murs, caisses, structures statiques)
        # Le LiDAR ne détecte que balises (pour SVD) + adversaire (get_opponent)
        # Les vrais obstacles (terrain) viennent de terrain_jeu
        obstacles = self.terrain.get_static_obstacles() if hasattr(self.terrain, 'get_static_obstacles') else []

        # 5. Adversaire → obstacles (dynamique, détecté via LiDAR)
        opponent_data = self.lidar.get_opponent()
        if opponent_data is not None:
            opp_x, opp_y, opp_conf = opponent_data
            if opp_conf > 0.1:
                obstacles.append({
                    'x':            opp_x,
                    'y':            opp_y,
                    'type':         'opponent',
                    'confidence':   opp_conf,
                    'radius_mm':    110.0,
                })

        # 6. Stratégie
        action = self.strategie.get_action_actuelle()
        if action is None:
            self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (fin missions)")
            return

        # 7. Exécution action
        if action.type == TypeAction.DEPLACEMENT:
            objectif          = {'x': action.cible_x, 'y': action.cible_y}
            distance_restante = math.hypot(action.cible_x - rx, action.cible_y - ry)

            if distance_restante < 50.0:
                self.logger.info(
                    f"Objectif ({action.cible_x}, {action.cible_y}) atteint !"
                )
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (arrivé)")
                self.strategie.valider_action_terminee()
                return

            # Position avec theta pour pathfinding
            pos = {'x': rx, 'y': ry, 'theta': rtheta}
            
            # Debug: vérifier format uniformes
            self.logger.debug(
                f"Pathfinding input: pos={{'x':{rx:.1f}, 'y':{ry:.1f}, 'theta':{rtheta:.3f}}}, "
                f"target={objectif}, "
                f"obstacles={len(obstacles)} "
                f"(types={[o.get('type') for o in obstacles][:5]})"
            )
            
            chemin = self.cerveau.get_path(pos, objectif, obstacles)

            if chemin and len(chemin) > 1:
                dist_to_target = math.hypot(
                    objectif['x'] - rx, objectif['y'] - ry
                )
                if dist_to_target > 500:
                    lookahead_index = min(4, len(chemin) - 1)
                elif dist_to_target < 200:
                    lookahead_index = min(1, len(chemin) - 1)
                else:
                    lookahead_index = min(2, len(chemin) - 1)

                cible_x, cible_y = chemin[lookahead_index]
                angle_cible      = math.atan2(cible_y - ry, cible_x - rx)
                self._envoyer_position_moteurs(
                    cible_x, cible_y, angle_cible, "Esquive IA"
                )
            else:
                self.logger.warning("Chemin bloqué — arrêt d'urgence.")
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (bloqué)")

        elif action.type == TypeAction.ACTIONNEUR:
            self.logger.info(f"Actionneur : {action.nom_actionneur}")
            # TODO: feedback réel via callback Teensy
            self.strategie.valider_action_terminee()

        elif action.type == TypeAction.ATTENTE:
            if self.strategie.demarrer_chrono_si_necessaire():
                self.logger.info(f"Attente {action.temps_attente}s…")
            elif self.strategie.chrono_ecoule(action.temps_attente):
                self.strategie.valider_action_terminee()

    def stopper_tout(self) -> None:
        """Procédure de sécurité fin de match."""
        self.logger.info("Arrêt des moteurs…")
        with self.lock:
            self._envoyer_position_moteurs(
                self.x, self.y, self.theta, "ARRÊT MATCH"
            )

        # FIX : utiliser stop_lidar_runtime() et non lidar.arreter()
        self.logger.info("Coupure du LiDAR…")
        stop_lidar_runtime()

        if hasattr(self, 'actionneurs'):
            try:
                self.actionneurs.ranger_bras()
                self.logger.info("Bras replié.")
            except Exception as e:
                self.logger.error(f"Erreur fermeture bras: {e}")
        else:
            self.logger.warning("Module actionneurs non chargé.")