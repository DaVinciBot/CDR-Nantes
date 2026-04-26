#!/usr/bin/env python3
"""
test_sim_mode.py
Mode simulation complet pour tester main.py sans hardware.

Lance avec:
    python test_sim_mode.py --sim
    python test_sim_mode.py --sim --with-rerun
"""
import sys
import argparse
import time
import logging
import math
import struct
import threading
import importlib.util
import subprocess
from pathlib import Path
from enum import Enum

# Configuration logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("SIM")

# ─────────────────────────────────────────────────────────────────────────────
# Classe Messages Mock (globale)
# ─────────────────────────────────────────────────────────────────────────────

class MockMessages(Enum):
    UPDATE_ROLLING_BASIS = 128
    SET_ODOMETRIE = 129
    SET_TARGET_POSITION = 130
    def to_bytes(self):
        return struct.pack('<B', self.value)


# ─────────────────────────────────────────────────────────────────────────────
# MOCKS pour remplacer le hardware
# ─────────────────────────────────────────────────────────────────────────────

class MockTeensyCOM:
    """Mock de la communication USB avec la Teensy."""
    
    def __init__(self):
        self.callbacks = {}
        self.sim_x = 1500.0
        self.sim_y = 1000.0
        self.sim_theta = 0.0
        self.target_x = self.sim_x
        self.target_y = self.sim_y
        self.target_theta = self.sim_theta
        self._lock = threading.Lock()
        self.running = False
        self.thread = None
        logger.info("✓ Mock Teensy COM initialisé")
    
    def add_callback(self, cb, msg_type):
        """Enregistre un callback pour un type de message."""
        self.callbacks[msg_type] = cb
        logger.info(f"  → Callback enregistré pour message type {msg_type}")
    
    def send_bytes(self, data):
        """Simule l'envoi de données à la Teensy.

        Protocole attendu:
          byte 0   : id message
          bytes 1+ : payload (<ddd) pour SET_ODOMETRIE / SET_TARGET_POSITION
        """
        if not data:
            return

        msg_id = data[0]

        # SET_ODOMETRIE: reset complet odométrie + cible
        if msg_id == MockMessages.SET_ODOMETRIE.value and len(data) >= 25:
            x, y, theta = struct.unpack('<ddd', data[1:25])
            with self._lock:
                self.sim_x, self.sim_y, self.sim_theta = x, y, theta
                self.target_x, self.target_y, self.target_theta = x, y, theta
            logger.info(f"[SIM] SET_ODOMETRIE reçu: X={x:.1f}, Y={y:.1f}, θ={theta:.3f}")

            # Publier immédiatement la nouvelle odométrie
            cb = self.callbacks.get(MockMessages.UPDATE_ROLLING_BASIS.value)
            if cb is not None:
                try:
                    cb(struct.pack('<ddd', x, y, theta))
                except Exception as e:
                    logger.error(f"Erreur callback immédiat SET_ODOMETRIE: {e}")
            return

        # SET_TARGET_POSITION: nouvelle consigne de déplacement
        if msg_id == MockMessages.SET_TARGET_POSITION.value and len(data) >= 25:
            x, y, theta = struct.unpack('<ddd', data[1:25])
            with self._lock:
                self.target_x, self.target_y, self.target_theta = x, y, theta
            logger.debug(f"[SIM] SET_TARGET_POSITION: X={x:.1f}, Y={y:.1f}, θ={theta:.3f}")
            return

        logger.debug(f"[SIM] Message Teensy ignoré: id={msg_id}, len={len(data)}")
    
    def start_sim(self):
        """Lance la boucle de simulation d'odométrie."""
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._sim_loop, daemon=True)
        self.thread.start()
    
    def _sim_loop(self):
        """Simule des updates odométrie à 20 Hz en suivant la consigne moteur."""
        speed = 200.0  # mm/s
        dt = 0.05  # 20 Hz
        
        while self.running:
            with self._lock:
                tx, ty, ttheta = self.target_x, self.target_y, self.target_theta
                sx, sy, stheta = self.sim_x, self.sim_y, self.sim_theta

            # Mouvement vers la consigne courante
            dx = tx - sx
            dy = ty - sy
            dist = math.hypot(dx, dy)

            if dist > 1.0:
                move = min(speed * dt, dist)
                sx += (dx / dist) * move
                sy += (dy / dist) * move
                stheta = math.atan2(dy, dx)
            else:
                # Cible atteinte: garder l'orientation cible commandée
                stheta = ttheta

            with self._lock:
                self.sim_x, self.sim_y, self.sim_theta = sx, sy, stheta
            
            # Envoyer callback UPDATE_ROLLING_BASIS (type 128)
            if 128 in self.callbacks:
                data = struct.pack('<ddd', sx, sy, stheta)
                try:
                    self.callbacks[128](data)
                except Exception as e:
                    logger.error(f"Erreur callback: {e}")
            
            time.sleep(dt)


class MockLidar:
    """Mock de l'interface Lidar."""
    
    def __init__(self, team_color="YELLOW"):
        self.team_color = team_color
        self.x = 1500.0
        self.y = 1000.0
        self.confidence = 0.85
        logger.info(f"✓ Mock Lidar initialisé ({team_color})")
    
    def get_opponent(self):
        """Retourne faux adversaire (ou None)."""
        return None  # Pas d'adversaire en test
    
    def get_fused_position(self, tx, ty, t_theta):
        """Retourne position fusionnée simulée.

        En mode test principal, on colle à l'odométrie pour démarrer
        exactement à la pose initiale et éviter les offsets artificiels.
        """
        self.x, self.y = tx, ty
        return tx, ty, t_theta, self.confidence


class MockStrategy:
    """Mock du manager de stratégie."""
    
    def __init__(self, team_color="YELLOW"):
        from strategy.strategy_actions import TypeAction
        self.TypeAction = TypeAction
        # Points choisis pour éviter les obstacles gonflés du PathFinder:
        # - Grenier gonflé approx: x in [440,2560], y in [1390,2160]
        # - Nid gonflé approx:     x in [0,760],   y in [1390,2160]
        # Depuis départ BLUE (2775,1700), on descend d'abord dans le couloir droit.
        self.actions = [
            {"type": "move", "x": 2775, "y": 1300},
            {"type": "wait", "duration": 1.0},
            {"type": "move", "x": 2300, "y": 1200},
            {"type": "wait", "duration": 1.0},
            {"type": "move", "x": 1600, "y": 900},
            {"type": "wait", "duration": 1.0},
            {"type": "move", "x": 900, "y": 900},
            {"type": "wait", "duration": 1.0},
            {"type": "move", "x": 2400, "y": 900},
        ]
        self.action_idx = 0
        self.start_time = None
        logger.info(f"✓ Mock Stratégie initialisée ({team_color})")
    
    def get_action_actuelle(self):
        """Retourne l'action courante."""
        if self.action_idx >= len(self.actions):
            return None
        
        action = self.actions[self.action_idx]
        
        # Créer un objet action fictif
        class MockAction:
            pass
        
        mock = MockAction()
        
        if action["type"] == "move":
            mock.type = self.TypeAction.DEPLACEMENT
            mock.cible_x = action["x"]
            mock.cible_y = action["y"]
        elif action["type"] == "wait":
            mock.type = self.TypeAction.ATTENTE
            mock.temps_attente = action["duration"]
        
        return mock
    
    def valider_action_terminee(self):
        """Passe à l'action suivante."""
        self.action_idx += 1
        logger.info(f"[SIM] Action {self.action_idx-1} validée, passage à {self.action_idx}")
    
    def demarrer_chrono_si_necessaire(self):
        if self.start_time is None:
            self.start_time = time.time()
            return True
        return False
    
    def chrono_ecoule(self, duration):
        if self.start_time is None:
            return False
        if time.time() - self.start_time >= duration:
            self.start_time = None
            return True
        return False


# ─────────────────────────────────────────────────────────────────────────────
# Injection des mocks
# ─────────────────────────────────────────────────────────────────────────────

def inject_mocks():
    """Remplace les imports réels par les mocks."""
    logger.info("\n" + "="*70)
    logger.info("INJECTION DES MOCKS - Mode simulation complet")
    logger.info("="*70 + "\n")
    
    # Mock du loader
    class MockLoader:
        def load_class(self, module, classname):
            if module == "usb_com" and classname == "Messages":
                return MockMessages
            return None
    
    class MockUtils:
        @staticmethod
        def init_robot(logger):
            com = MockTeensyCOM()
            com.start_sim()
            return com, "SIMULATION"
    
    # Remplacer dans sys.modules
    sys.modules['loader'] = type(sys)('loader')
    sys.modules['loader'].loader = MockLoader()
    
    sys.modules['utils'] = type(sys)('utils')
    sys.modules['utils'].init_robot = MockUtils.init_robot
    
    logger.info("✓ Mocks injectés dans sys.modules")


def patch_robot_for_sim(robot_class):
    """Patchs pour Robot.__init__ en mode simulation."""
    original_init = robot_class.__init__
    
    def patched_init(self, couleur_equipe):
        # Appel original jusqu'à la Teensy
        self.logger = logging.getLogger("ROBOT")
        self.logger.info(f"Initialisation du robot en mode {couleur_equipe}...")
        
        self.couleur = couleur_equipe
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lock = threading.Lock()
        self._last_correction_sent_time = 0.0
        
        # Terrain
        from terrain_jeu import Terrain
        from pathfinder import PathFinder
        self.terrain = Terrain(couleur_equipe)
        self.cerveau = PathFinder(self.terrain)
        
        # Lidar MOCK
        self.lidar = MockLidar(team_color=couleur_equipe)
        
        # Teensy MOCK — utiliser la classe MockMessages globale
        self.Messages = MockMessages
        
        self.com = MockTeensyCOM()
        self.com.add_callback(self._handle_position, 128)
        self.mode = "SIMULATION"
        
        # Stratégie MOCK
        self.strategie = MockStrategy(couleur_equipe)
        
        # Important: initialiser la pose avant de démarrer la boucle de simulation
        self._initialiser_position_depart()

        # Aligner le mock Lidar sur la pose de départ
        self.lidar.x = self.x
        self.lidar.y = self.y

        # Démarrer la simulation odométrique après init
        self.com.start_sim()
        self.logger.info("Robot entièrement initialisé (MODE SIMULATION).")
    
    robot_class.__init__ = patched_init


# ─────────────────────────────────────────────────────────────────────────────
# Main simulation
# ─────────────────────────────────────────────────────────────────────────────
def main_sim(with_rerun=False, rerun_mode="local", rerun_port=9876):
    """Lance main.py en mode simulation."""

    # ── Rerun AVANT inject_mocks ──────────────────────────────────────────────
    if with_rerun:
        try:
            logger.info("Initialisation de Rerun Bridge...")

            # Chercher rerun_bridge.py
            candidates = [
                Path(__file__).parent / "rerun" / "rerun_bridge.py",
                Path(__file__).parent / "rerun_bridge.py",
            ]
            bridge_path = next((p for p in candidates if p.exists()), None)
            if bridge_path is None:
                raise FileNotFoundError("rerun_bridge.py introuvable dans rerun/ et ./")

            spec   = importlib.util.spec_from_file_location("rerun_bridge", bridge_path)
            rb     = importlib.util.module_from_spec(spec)
            
            # IMPORTANT: injecter dans sys.modules AVANT exec_module pour que @dataclass trouve le module
            sys.modules["rerun_bridge"] = rb
            spec.loader.exec_module(rb)

            # Init Rerun
            rb.rr.init("eurobot_2026")
            
            if rerun_mode == "serve":
                # Mode serveur (Rasp → PC)
                rb.rr.serve_grpc(grpc_port=rerun_port, server_memory_limit="200MB")
                logger.info(f"✓ Rerun serveur lancé sur port {rerun_port}")
                logger.info(f"   Sur le PC : rerun --connect rerun+http://localhost:{rerun_port}/proxy")
            else:
                # Mode local (viewer spawné sur cette machine)
                rb.rr.spawn()                          # ouvre le viewer
                logger.info("✓ Rerun viewer spawné localement")
            
            rb.rr.send_blueprint(rb.create_blueprint())
            rb.log_static_map()

            # Boucle publication 20 Hz en thread daemon
            threading.Thread(
                target=rb.publish_loop,
                kwargs={"hz": 20.0, "lidar_poll": None},
                daemon=True,
            ).start()

            logger.info("✓ Rerun Bridge opérationnel")

        except Exception as e:
            logger.warning(f"⚠  Rerun désactivé : {e}")
            import traceback; traceback.print_exc()
            with_rerun = False

    # ── Injection des mocks ───────────────────────────────────────────────────
    inject_mocks()

    # ── Patch Robot ───────────────────────────────────────────────────────────
    from robot import Robot
    patch_robot_for_sim(Robot)

    logger.info("\n" + "="*70)
    logger.info("LANCEMENT DU MAIN.PY EN MODE SIMULATION")
    logger.info("="*70 + "\n")

    TEMPS_MATCH_SECONDES = 30.0

    logger.info("Démarrage du système...")
    couleur_equipe = "BLUE"
    mon_robot = Robot(couleur_equipe)

    logger.info(f"Robot initialisé pour l'équipe {couleur_equipe}.")
    logger.info("⏱️  Simulation sans tirette — démarrage immédiat...")

    heure_debut = time.time()
    logger.info("🚀 SIMULATION LANCÉE !\n")

    try:
        iteration = 0
        while True:
            temps_ecoule = time.time() - heure_debut

            if temps_ecoule >= TEMPS_MATCH_SECONDES:
                logger.info("\n✓ FIN DE LA SIMULATION !")
                break

            mon_robot.update()
            iteration += 1

            if iteration % 20 == 0:
                logger.info(f"[SIM] Temps écoulé: {temps_ecoule:.1f}s / {TEMPS_MATCH_SECONDES}s")

            time.sleep(0.05)

    except KeyboardInterrupt:
        logger.warning("\n⚠️  Arrêt d'urgence (Ctrl+C)")

    finally:
        logger.info("\nDésactivation des systèmes...")
        mon_robot.stopper_tout()
        logger.info("✓ Simulation terminée.")


if __name__ == "__main__":
    p = argparse.ArgumentParser(
        description="Test simulation complet",
        epilog="""
Exemples:
  python test_sim_mode.py --sim
  python test_sim_mode.py --sim --with-rerun
  python test_sim_mode.py --sim --with-rerun --mode serve --port 9876
        """
    )
    p.add_argument("--sim", action="store_true", help="Mode simulation", required=True)
    p.add_argument("--with-rerun", action="store_true", help="Avec Rerun visualization")
    p.add_argument("--mode", choices=["local", "serve"], default="local",
                   help="Mode Rerun (local=spawn viewer | serve=stream gRPC vers PC distant)")
    p.add_argument("--port", type=int, default=9876,
                   help="Port gRPC pour mode serve (défaut: 9876)")
    args = p.parse_args()
    
    if args.sim:
        main_sim(with_rerun=args.with_rerun, rerun_mode=args.mode, rerun_port=args.port)
