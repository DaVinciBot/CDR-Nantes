import struct
import math
import logging
import threading
import time

from gpiozero import Button

from terrain_jeu import Terrain
from pathfinder import PathFinder
import lidar.lidar_detection as lidar

from loader import loader
from utils import init_robot
from strategy.strategy_actions import TypeAction
from strategy.strategy_strat_manager import StratManager

# ── PINS GPIO ─────────────────────────────────────────────────────────────────
PIN_COULEUR = 26
PIN_TIRETTE = 19

# ── IMPORT OPTIONNEL RERUN ────────────────────────────────────────────────────
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
    if 'rerun_bridge' in sys.modules and _is_valid_rerun_bridge(sys.modules['rerun_bridge']):
        rerun_bridge = sys.modules['rerun_bridge']
        HAS_RERUN = True
    else:
        if hasattr(loader, "load_rerun_bridge"):
            candidate = loader.load_rerun_bridge()
        else:
            candidate = None
        if _is_valid_rerun_bridge(candidate):
            rerun_bridge = candidate
            HAS_RERUN = True
except Exception:
    pass


# ── LECTURE COULEUR (avant instanciation Robot) ───────────────────────────────

def lire_couleur_equipe() -> str:
    """
    Lit le switch de couleur et retourne "BLUE" ou "YELLOW".
    À appeler dans main.py AVANT de créer Robot().

    Switch pressé (pin LOW) → BLUE
    Switch relâché (pin HIGH) → YELLOW
    """
    pin = Button(PIN_COULEUR, pull_up=True)
    couleur = "BLUE" if pin.is_pressed else "YELLOW"
    pin.close()
    return couleur


class Robot:
    def __init__(self, couleur_equipe: str):
        self.logger = logging.getLogger("ROBOT")
        self.logger.info(f"Initialisation du robot en mode {couleur_equipe}...")

        self.couleur = couleur_equipe.upper()
        self.x       = 0.0
        self.y       = 0.0
        self.theta   = 0.0
        self.lock    = threading.Lock()

        # GPIO tirette (kept open pendant tout le match)
        self._pin_tirette = Button(PIN_TIRETTE, pull_up=True)

        # 1. Terrain et pathfinding
        self.terrain = Terrain(self.couleur)
        self.cerveau = PathFinder(self.terrain)

        # 2. Démarrer LiDAR (détection adversaire)
        lidar.start()

        # 4. Communication USB
        self.Messages       = loader.load_class('usb_com', 'Messages')
        self.com, self.mode = init_robot(self.logger)

        self.com.add_callback(
            self._handle_position,
            self.Messages.UPDATE_ROLLING_BASIS.value,
        )

        # 5. Position de départ
        self._initialiser_position_depart()

        # 6. Stratégie
        self.strategie = StratManager(self.couleur)
        self.logger.info("Robot entièrement initialisé.")

    # ── INITIALISATION ────────────────────────────────────────────────────────

    def _initialiser_position_depart(self) -> None:
        start_x, start_y, start_theta = self.terrain.get_start_position()

        self.logger.info(
            f"Position de départ [{self.couleur}] : "
            f"X={start_x:.0f}mm  Y={start_y:.0f}mm  θ={math.degrees(start_theta):.1f}°"
        )

        with self.lock:
            self.x     = start_x
            self.y     = start_y
            self.theta = start_theta

        lidar.update_robot_pose(start_x, start_y, start_theta)

        msg  = self.Messages.SET_ODOMETRIE.to_bytes()
        msg += struct.pack('<ddd', start_x, start_y, start_theta)
        self.com.send_bytes(msg)

        time.sleep(0.3)
        self.logger.info("Position de départ transmise à la Teensy.")

    # ── TIRETTE ───────────────────────────────────────────────────────────────

    def attendre_tirette(self) -> None:
        """Boucle bloquante jusqu'au retrait de la tirette."""
        self.logger.info(
            f"[{self.couleur}] En attente du retrait de la tirette..."
        )
        self._pin_tirette.wait_for_release()
        self.logger.info("Tirette retirée — MATCH DÉMARRÉ !")

    # ── BAS NIVEAU ────────────────────────────────────────────────────────────

    def _handle_position(self, data: bytes) -> None:
        """Callback USB — mise à jour odométrie Teensy."""
        if len(data) >= 24:
            x, y, theta = struct.unpack('<ddd', data[:24])
            with self.lock:
                self.x     = x
                self.y     = y
                self.theta = theta
            lidar.update_robot_pose(x, y, theta)
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

    # ── BOUCLE PRINCIPALE ─────────────────────────────────────────────────────

    def update(self) -> None:
        """Boucle de décision — appelée 20 fois/seconde."""

        # 1. Lecture position Teensy
        with self.lock:
            teensy_x, teensy_y, teensy_theta = self.x, self.y, self.theta

        # 2. Mise à jour pose pour LiDAR
        lidar.update_robot_pose(teensy_x, teensy_y, teensy_theta)

        # 3. Position finale = odométrie Teensy (pas de correction LiDAR)
        rx, ry, rtheta = teensy_x, teensy_y, teensy_theta

        # 4. Obstacles dynamiques (adversaire détecté par LiDAR)
        obstacles: list = []
        opponent_data = lidar.get_opponent()
        if opponent_data is not None:
            opp_x, opp_y, opp_conf = opponent_data
            if opp_conf > 0.1:
                obstacles.append((opp_x, opp_y))
                self.logger.debug(
                    f"Adversaire: ({opp_x:.0f}, {opp_y:.0f}) conf={opp_conf:.2f}"
                )

        # 5. Publication Rerun
        if HAS_RERUN:
            rerun_bridge.update_odom(teensy_x, teensy_y, teensy_theta)
            rerun_bridge.update_lidar_pose(
                teensy_x, teensy_y, teensy_theta,
                0.0, False,  # pas de correction LiDAR
            )
            rerun_bridge.update_fused(rx, ry, rtheta)
            rerun_bridge.update_obstacles(
                [{"x": ox, "y": oy, "radius": 200} for ox, oy in obstacles]
            )

        # 6. Stratégie
        action = self.strategie.get_action_actuelle()
        if action is None:
            self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (fin missions)")
            if HAS_RERUN:
                rerun_bridge.update_trajectory([])
            return

        # 7. Exécution action
        if action.type == TypeAction.DEPLACEMENT:
            objectif          = {'x': action.cible_x, 'y': action.cible_y}
            distance_restante = math.hypot(action.cible_x - rx, action.cible_y - ry)

            if HAS_RERUN:
                rerun_bridge.update_target(action.cible_x, action.cible_y)

            if distance_restante < 80.0:
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

            elif not chemin:
                self.logger.warning(
                    f"Chemin bloqué — aucun chemin de ({rx:.0f},{ry:.0f}) "
                    f"vers ({objectif['x']},{objectif['y']}). Arrêt."
                )
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (bloqué)")
                if HAS_RERUN:
                    rerun_bridge.update_trajectory([])

            else:
                # len(chemin) == 1 → robot déjà dans la cellule cible
                self.logger.info("Cellule cible atteinte (path trivial).")
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (arrivé)")
                self.strategie.valider_action_terminee()
                if HAS_RERUN:
                    rerun_bridge.update_trajectory([])

        elif action.type == TypeAction.ACTIONNEUR:
            self.logger.info(f"Actionneur : {action.nom_actionneur}")
            self.strategie.valider_action_terminee()

        elif action.type == TypeAction.ATTENTE:
            if self.strategie.demarrer_chrono_si_necessaire():
                self.logger.info(f"Attente {action.temps_attente}s…")
            elif self.strategie.chrono_ecoule(action.temps_attente):
                self.strategie.valider_action_terminee()

    # ── ARRÊT ─────────────────────────────────────────────────────────────────

    def stopper_tout(self) -> None:
        """Procédure de sécurité fin de match."""
        self.logger.info("Arrêt des moteurs…")
        with self.lock:
            self._envoyer_position_moteurs(
                self.x, self.y, self.theta, "ARRÊT MATCH"
            )

        self.logger.info("Arrêt du LiDAR…")
        lidar.stop()

        self._pin_tirette.close()

        if hasattr(self, 'actionneurs'):
            try:
                self.actionneurs.ranger_bras()
                self.logger.info("Bras replié.")
            except Exception as e:
                self.logger.error(f"Erreur fermeture bras: {e}")