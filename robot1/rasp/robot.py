import struct
import math
import logging
import threading
import time

from terrain_jeu import Terrain
from pathfinder import PathFinder
from lidar.lidar import LidarInterface
from lidar.lidar_logic import (
    set_team_color,
    update_teensy_pose,
    get_corrected_pose,
    should_send_correction_to_teensy,
    stop_lidar_runtime,
)

from loader import loader
from utils import init_robot
from strategy.strategy_actions import TypeAction
from strategy.strategy_strat_manager import StratManager

# Import optionnel Rerun (pour visualisation monitoring)
HAS_RERUN = False
rerun_bridge = None
REQUIRED_RERUN_FUNCS = (
    "update_fused",
    "update_obstacles",
    "update_target",
    "update_trajectory",
)


def _is_valid_rerun_bridge(module) -> bool:
    if module is None:
        return False
    return all(callable(getattr(module, fn, None)) for fn in REQUIRED_RERUN_FUNCS)


try:
    import sys
    # Vérifier si rerun_bridge est déjà chargé (par test_sim_mode)
    if 'rerun_bridge' in sys.modules and _is_valid_rerun_bridge(sys.modules['rerun_bridge']):
        rerun_bridge = sys.modules['rerun_bridge']
        HAS_RERUN = True
    else:
        # Essayer de le charger via le loader
        if hasattr(loader, "load_rerun_bridge"):
            candidate = loader.load_rerun_bridge()
        else:
            candidate = None

        if _is_valid_rerun_bridge(candidate):
            rerun_bridge = candidate
            HAS_RERUN = True
except Exception as e:
    pass  # Silencieux si Rerun n'est pas disponible


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
        set_team_color(couleur_equipe)
        self.logger.info(f"Balises LiDAR chargées pour équipe {couleur_equipe}.")

        # 2. Terrain et pathfinding
        self.terrain = Terrain(couleur_equipe)
        self.cerveau = PathFinder(self.terrain)

        # 3. Interface LiDAR
        self.lidar = LidarInterface(team_color=couleur_equipe)

        # 4. Communication USB
        self.Messages       = loader.load_class('usb_com', 'Messages')
        self.com, self.mode = init_robot(self.logger)

        self.com.add_callback(
            self._handle_position,
            self.Messages.UPDATE_ROLLING_BASIS.value,
        )

        # 5. Envoi de la position de départ connue vers la Teensy
        #    (remplace l'odométrie Teensy à (0,0,0) par la vraie position initiale)
        self._initialiser_position_depart()

        # 6. Stratégie
        self.strategie = StratManager(couleur_equipe)
        self.logger.info("Robot entièrement initialisé.")

    # ── INITIALISATION ────────────────────────────────────────────────────────

    def _initialiser_position_depart(self) -> None:
        """
        Envoie SET_ODOMETRIE à la Teensy avec la position initiale connue
        (coin de départ selon la couleur d'équipe) et initialise les variables
        locales de position pour que le LiDAR puisse démarrer avec une pose de
        référence correcte.

        Doit être appelé APRÈS que self.com est initialisé.
        """
        start_x, start_y, start_theta = self.terrain.get_start_position()

        self.logger.info(
            f"Position de départ [{self.couleur}] : "
            f"X={start_x:.0f}mm  Y={start_y:.0f}mm  θ={math.degrees(start_theta):.1f}°"
        )

        # Mise à jour locale immédiate (avant le premier callback Teensy)
        with self.lock:
            self.x     = start_x
            self.y     = start_y
            self.theta = start_theta

        # Signal au thread LiDAR (fenêtres de prédiction SVD)
        update_teensy_pose(start_x, start_y, start_theta)

        # Envoi à la Teensy — reset complet de son odométrie ET de sa cible
        # (set_odometrie() côté Teensy appelle init_holonomic_basis() + target ← même pos)
        msg  = self.Messages.SET_ODOMETRIE.to_bytes()
        msg += struct.pack('<ddd', start_x, start_y, start_theta)
        self.com.send_bytes(msg)

        # Petite attente pour laisser la Teensy traiter le message avant le match
        time.sleep(0.3)
        self.logger.info("Position de départ transmise à la Teensy.")

    # ── BAS NIVEAU ────────────────────────────────────────────────────────────

    def _handle_position(self, data: bytes) -> None:
        """Callback USB — mise à jour odométrie Teensy."""
        if len(data) >= 24:
            x, y, theta = struct.unpack('<ddd', data[:24])
            with self.lock:
                self.x     = x
                self.y     = y
                self.theta = theta
            update_teensy_pose(x, y, theta)
        else:
            self.logger.warning(f"Message odométrie trop court: {len(data)} bytes")

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
        """Filtre complémentaire adaptatif LiDAR + Teensy."""
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
        # TODO: remplacer par lecture GPIO réel (pin à définir)
        time.sleep(3)
        self.logger.info("Tirette retirée !")

    def update(self) -> None:
        """Boucle de décision — appelée 20 fois/seconde."""

        # 1. Lecture position Teensy
        with self.lock:
            teensy_x, teensy_y, teensy_theta = self.x, self.y, self.theta

        # 2. Signal LiDAR
        update_teensy_pose(teensy_x, teensy_y, teensy_theta)

        # Données de visualisation (alimentent Rerun si actif)
        lidar_vis_x = teensy_x
        lidar_vis_y = teensy_y
        lidar_vis_theta = teensy_theta
        lidar_vis_conf = 0.0
        lidar_vis_ok = False

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

            lidar_vis_x = corrected_pose.x
            lidar_vis_y = corrected_pose.y
            lidar_vis_theta = corrected_pose.theta
            lidar_vis_conf = corrected_pose.confidence
            lidar_vis_ok = corrected_pose.is_localized

            self.logger.debug(
                f"Pose fusée (SVD): ({rx:.1f}, {ry:.1f}, {rtheta:.3f}) | "
                f"conf={corrected_pose.confidence:.2f} alpha={alpha:.2f}"
            )

            if should_send_correction_to_teensy():
                now = time.time()
                if now - self._last_correction_sent_time >= 1.0:
                    self._send_odometry_correction(rx, ry, rtheta)
                    self._last_correction_sent_time = now
        else:
            rx, ry, rtheta, _ = self.lidar.get_fused_position(
                teensy_x, teensy_y, teensy_theta
            )
            # En simulation, le mock lidar suit la position courante.
            lidar_vis_x = getattr(self.lidar, "x", rx)
            lidar_vis_y = getattr(self.lidar, "y", ry)
            lidar_vis_theta = rtheta
            lidar_vis_conf = float(getattr(self.lidar, "confidence", 0.0))
            lidar_vis_ok = lidar_vis_conf > 0.0

        # 4. Obstacles dynamiques — format (x, y) tuples pour PathFinder
        #    Les obstacles statiques sont déjà dans la grille PathFinder (pre-built).
        obstacles: list = []

        opponent_data = self.lidar.get_opponent()
        if opponent_data is not None:
            opp_x, opp_y, opp_conf = opponent_data
            if opp_conf > 0.1:
                # ← CORRECTION : tuple (x, y) attendu par create_dynamic_grid()
                obstacles.append((opp_x, opp_y))
                self.logger.debug(
                    f"Adversaire ajouté aux obstacles: ({opp_x:.0f}, {opp_y:.0f}) "
                    f"conf={opp_conf:.2f}"
                )

        # Publication Rerun — position fusionnée + obstacles
        if HAS_RERUN:
            rerun_bridge.update_odom(teensy_x, teensy_y, teensy_theta)
            rerun_bridge.update_lidar_pose(
                lidar_vis_x, lidar_vis_y, lidar_vis_theta, lidar_vis_conf, lidar_vis_ok
            )
            rerun_bridge.update_fused(rx, ry, rtheta)
            obs_list = [{"x": ox, "y": oy, "radius": 200} for ox, oy in obstacles]
            rerun_bridge.update_obstacles(obs_list)

        # 5. Stratégie
        action = self.strategie.get_action_actuelle()
        if action is None:
            self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (fin missions)")
            if HAS_RERUN:
                rerun_bridge.update_trajectory([])
            return

        # 6. Exécution action
        if action.type == TypeAction.DEPLACEMENT:
            objectif          = {'x': action.cible_x, 'y': action.cible_y}
            distance_restante = math.hypot(action.cible_x - rx, action.cible_y - ry)

            # Publication Rerun — cible
            if HAS_RERUN:
                rerun_bridge.update_target(action.cible_x, action.cible_y)

            if distance_restante < 50.0:
                self.logger.info(
                    f"Objectif ({action.cible_x}, {action.cible_y}) atteint !"
                )
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (arrivé)")
                self.strategie.valider_action_terminee()
                if HAS_RERUN:
                    rerun_bridge.update_trajectory([])
                return

            pos    = {'x': rx, 'y': ry, 'theta': rtheta}
            chemin = self.cerveau.get_path(pos, objectif, obstacles)

            # Publication Rerun — trajectoire calculée
            if HAS_RERUN and chemin:
                rerun_bridge.update_trajectory(chemin)

            if chemin and len(chemin) > 1:
                dist_to_target = math.hypot(objectif['x'] - rx, objectif['y'] - ry)
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
                if HAS_RERUN:
                    rerun_bridge.update_trajectory([])

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