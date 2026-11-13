import struct
import math
import logging
import threading

from usb_com import Messages, pack_message, payload_size, unpack_message

from robot1.rasp.clock import RealClock
from robot1.rasp.comm import get_button, init_robot
from robot1.rasp.nav import Disc, PathFinder, Replanner
from robot1.rasp.strategy import StratManager, TypeAction
from robot1.rasp.vision import detection as lidar
from robot1.rasp.world import ROBOT_DEPLOYED_PERIMETER_MM, Terrain

# ── PINS GPIO ─────────────────────────────────────────────────────────────────
PIN_COULEUR = 26
PIN_TIRETTE = 19

# Opponent footprint for the planner: the circle of the largest deployed
# perimeter the rules allow (1400 mm), ~223 mm. The LiDAR cluster is the
# opponent's beacon mast, not its body: its size does not measure the robot.
OPPONENT_RADIUS_MM = ROBOT_DEPLOYED_PERIMETER_MM / (2.0 * math.pi)

# ── IMPORT OPTIONNEL RERUN ────────────────────────────────────────────────────
# La telemetrie est best-effort : son absence ne doit jamais empecher un match.
try:
    from robot1.rasp.telemetry import rerun_bridge
    HAS_RERUN = True
except Exception:
    rerun_bridge = None
    HAS_RERUN = False


# ── LECTURE COULEUR (avant instanciation Robot) ───────────────────────────────

def lire_couleur_equipe() -> str:
    """
    Lit le switch de couleur et retourne "BLUE" ou "YELLOW".
    À appeler dans main.py AVANT de créer Robot().

    Switch pressé (pin LOW) -> BLUE
    Switch relâché (pin HIGH) -> YELLOW

    Hors mode hardware, l'état du pin vient de comm.set_fake_button().
    """
    pin = get_button(PIN_COULEUR, pull_up=True)
    couleur = "BLUE" if pin.is_pressed else "YELLOW"
    pin.close()
    return couleur


class Robot:
    def __init__(self, couleur_equipe: str, clock=None):
        """Construit le robot et ouvre ses périphériques.

        Args:
            couleur_equipe: "BLUE" ou "YELLOW".
            clock: source de temps (voir robot1.rasp.clock). RealClock par
                défaut, donc comportement matériel inchangé ; sim2d passe une
                horloge virtuelle pour jouer un match plus vite que le temps
                réel, de façon reproductible.
        """
        self.logger = logging.getLogger("ROBOT")
        self.logger.info(f"Initialisation du robot en mode {couleur_equipe}...")

        self.couleur = couleur_equipe.upper()
        self.clock   = clock if clock is not None else RealClock()
        self.x       = 0.0
        self.y       = 0.0
        self.theta   = 0.0
        self.lock    = threading.Lock()

        # GPIO tirette (kept open pendant tout le match)
        self._pin_tirette = get_button(PIN_TIRETTE, pull_up=True)

        # 1. Terrain et pathfinding : neutres, en coordonnées table absolues.
        # Seule la stratégie connaît la couleur et applique la symétrie.
        self.terrain = Terrain()
        self.cerveau = PathFinder(
            self.terrain.WIDTH, self.terrain.HEIGHT,
            fixed=self.terrain.obstacles,
            robot_radius=self.terrain.ROBOT_RADIUS,
        )
        # The path is kept between ticks, the A* only runs on events.
        self.navigation = Replanner(self.cerveau)
        # No game state tracked yet: every quarry is assumed full.
        self.carrieres = [q.footprint() for q in self.terrain.quarries]

        # 2. Demarrer LiDAR (detection adversaire)
        # La detection doit partager l'horloge de la boucle : son gating de
        # vitesse compare des horodatages qu'elle produit elle-meme.
        lidar.set_clock(self.clock)
        # start() waits for the connection and returns False when the LiDAR
        # does not answer. Without this check the acquisition thread died on
        # its own and get_opponent() returned None for the whole match,
        # silently.
        if not lidar.start():
            _, erreur = lidar.get_status()
            self.logger.error(
                "Détection adversaire INDISPONIBLE (%s). "
                "Le robot roulera sans évitement dynamique.",
                erreur or "cause inconnue",
            )

        # 4. Communication USB
        self.Messages       = Messages
        self.com, self.mode = init_robot(self.logger)

        self.com.add_callback(
            self._handle_position,
            self.Messages.ODOMETRY.value,
        )

        # 5. Stratégie : avant la position de départ, qu'elle fournit
        # (pose JAUNE de référence passée par la symétrie d'équipe).
        self.strategie = StratManager(self.couleur, clock=self.clock)

        # 6. Position de départ
        self._initialiser_position_depart()
        self.logger.info("Robot entièrement initialisé.")

    # ── INITIALISATION ────────────────────────────────────────────────────────

    def _initialiser_position_depart(self) -> None:
        start_x, start_y, start_theta = self.strategie.get_start_position()

        self.logger.info(
            f"Position de départ [{self.couleur}] : "
            f"X={start_x:.0f}mm  Y={start_y:.0f}mm  theta={math.degrees(start_theta):.1f}°"
        )

        with self.lock:
            self.x     = start_x
            self.y     = start_y
            self.theta = start_theta

        lidar.update_robot_pose(start_x, start_y, start_theta)

        msg  = self.Messages.SET_ODOMETRIE.to_bytes()
        msg += struct.pack('<ddd', start_x, start_y, start_theta)
        self.com.send_bytes(msg)

        self.clock.sleep(0.3)
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
        if len(data) == payload_size(self.Messages.ODOMETRY):
            x, y, theta = unpack_message(self.Messages.ODOMETRY, data)[:3]
            with self.lock:
                self.x     = x
                self.y     = y
                self.theta = theta
            lidar.update_robot_pose(x, y, theta)
        else:
            self.logger.warning(
                f"Message odométrie de taille inattendue: {len(data)} bytes"
            )

    def _envoyer_position_moteurs(
        self, x: float, y: float, theta: float, description: str = ""
    ) -> None:
        # Plain pose target: no trajectory velocity yet.
        msg = pack_message(self.Messages.SET_TARGET, x, y, theta, 0.0, 0.0, 0.0)
        self.com.send_bytes(msg)
        self.logger.debug(f"{description}: X={x:.1f}, Y={y:.1f}, theta={theta:.2f}")

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
            f"Y={corrected_y:.1f}, theta={corrected_theta:.3f}"
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
        obstacles = self._obstacles_adverses()

        # 5. Publication Rerun
        if HAS_RERUN:
            rerun_bridge.update_odom(teensy_x, teensy_y, teensy_theta)
            rerun_bridge.update_lidar_pose(
                teensy_x, teensy_y, teensy_theta,
                0.0, False,  # pas de correction LiDAR
            )
            rerun_bridge.update_fused(rx, ry, rtheta)
            rerun_bridge.update_obstacles(
                [{"x": o.x, "y": o.y, "radius": o.radius} for o in obstacles]
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

            plans_avant = self.navigation.plans
            resultat = self.navigation.update(
                self.clock.now(), (rx, ry), (action.cible_x, action.cible_y),
                obstacles + self._carrieres_hors_cible(action),
            )

            if HAS_RERUN:
                rerun_bridge.update_trajectory(
                    resultat.path if resultat.ok else [])

            if not resultat.ok:
                # Logged once per attempt, not at every tick.
                if self.navigation.plans != plans_avant:
                    self.logger.warning(
                        f"Pas de chemin ({resultat.status.value} : "
                        f"{resultat.detail}). Arrêt."
                    )
                self._envoyer_position_moteurs(rx, ry, rtheta, "STOP (bloqué)")
                return

            # Lookahead along the path, as the A* cells used to give it.
            # Replaced by the path follower and its speed profile.
            if distance_restante > 500:
                lookahead_mm = 200.0
            elif distance_restante < 200:
                lookahead_mm = 50.0
            else:
                lookahead_mm = 100.0
            cible_x, cible_y = self.navigation.ahead((rx, ry), lookahead_mm)
            angle_cible = math.atan2(cible_y - ry, cible_x - rx)
            self._envoyer_position_moteurs(
                cible_x, cible_y, angle_cible, "Esquive IA"
            )

        elif action.type == TypeAction.ACTIONNEUR:
            self.logger.info(f"Actionneur : {action.nom_actionneur}")
            self.strategie.valider_action_terminee()

        elif action.type == TypeAction.ATTENTE:
            if self.strategie.demarrer_chrono_si_necessaire():
                self.logger.info(f"Attente {action.temps_attente}s…")
            elif self.strategie.chrono_ecoule(action.temps_attente):
                self.strategie.valider_action_terminee()

    # ── OBSTACLES ─────────────────────────────────────────────────────────────

    def _obstacles_adverses(self) -> list:
        """Opponent seen by the LiDAR, as planner obstacles.

        Provisional adapter: the only place that reads the detection output.
        It changes with the format agreed for the LiDAR, nothing else does.
        """
        opponent_data = lidar.get_opponent()
        if opponent_data is None:
            return []
        opp_x, opp_y, opp_conf = opponent_data
        if opp_conf <= 0.1:
            return []
        self.logger.debug(
            f"Adversaire: ({opp_x:.0f}, {opp_y:.0f}) conf={opp_conf:.2f}"
        )
        return [Disc(opp_x, opp_y, OPPONENT_RADIUS_MM, "adversaire")]

    def _carrieres_hors_cible(self, action) -> list:
        """Quarries to plan around: all of them but the one being reached.

        The target is an approach point next to a quarry, not a quarry: the
        quarry it serves is the one that would forbid it.
        """
        cible = (action.cible_x, action.cible_y)
        return [
            zone for zone in self.carrieres
            if self.cerveau.blocking_obstacle(*cible, [zone]) != zone.name
        ]

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

        # get_button() always returns an object, real or simulated, so this is
        # never None on the production path. The guard stays for the historical
        # tests that patch __init__ and create no pin at all.
        if self._pin_tirette is not None:
            self._pin_tirette.close()