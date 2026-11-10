"""Classe les groupes du LiDAR et suit un adversaire possible.

Entree : (qualite, angle en degres, distance en mm). La convention historique
est conservee : angle 0 = +Y local, angle 90 = +X local.

Acquisition partagee : update_robot_pose(x, y, theta), puis
push_scan(scan.mesures). Ne pas appeler start() en mode materiel en parallele
avec acquisition.py. L'ancienne API start/stop reste compatible avec app.py.

get_clusters() donne les classes et les points de chaque groupe recent.
get_opponent() garde le format historique (x_mm, y_mm, score) ou None.
Essai sans materiel : python -m robot1.rasp.vision.detection --demo

Le code peut trouver plusieurs groupes adversaire_possible, mais il n’en propose qu’un seul au suivi : le plus proche de du robot.

Par exemple, si deux candidats sont à 700 mm et 900 mm, il propose celui à 700 mm. Les deux restent visibles dans get_clusters(), tandis que get_opponent() renvoie au maximum une cible suivie.

Retour : 
detection.get_clusters() -> Tuple d’objets `Cluster` -> Tous les groupes détectés 
groupe.categorie -> String -> mur_probable
detection.get_opponent() ->Tuple de trois nombres ou None -> (1120.0, 995.0, 1.0)
"""

"""Opponent detection from the A2M12 LiDAR.

Raw cluster centroid, exponential smoothing, speed gating. No geometric fit, so
it works whatever the shape of the object.

This is the only perception layer the match loop (app.py) uses. The beacon and
SVD stack lives in localization.py and is frozen, see vision/README.md.

Public API:
    start()                        start the acquisition thread, returns success
    stop()                         stop it cleanly
    get_opponent()                 (x_mm, y_mm, confidence) or None
    get_status()                   (connected, last_error)
    update_robot_pose(x, y, theta) called by app.py on every update()

Off-robot API (sim2d, replay):
    set_clock(clock)               replace the time source
    set_scan_source(factory)       replace the LiDAR by an iterator of scans
    use_pushed_scans()             take scans from push_scan(), no thread
    push_scan(scan)                process one scan synchronously
"""

import math
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, Tuple, List

import numpy as np

try:
    from robot1.rasp.clock import DEFAULT_CLOCK
except ModuleNotFoundError as exc:
    if exc.name != "robot1.rasp.clock":
        raise
    # Compatibilité avec l'ancien dépôt, sans clock.py.
    from types import SimpleNamespace
    DEFAULT_CLOCK = SimpleNamespace(now=time.monotonic, sleep=time.sleep)

from .lidar_config import BAUDRATE, PORT, TIMEOUT

logger = logging.getLogger("LIDAR_DETECT")

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
# Serial link: PORT / BAUDRATE / TIMEOUT come from config.json through
# lidar_config.py. Never hardcode a port here, it is what silently disabled
# opponent detection on the robot until 11/09/2026.

# Filters
MAX_SCAN_DIST_MM = 4000  # portée d’analyse des murs, distincte de la portée adverse
MIN_DIST_MM    = 200     # reliable dead zone of the A2M12
DETECT_DIST_MM = 1500    # < 520 mm hides the walls of the test field
                         # CDR value: 1500
MIN_QUALITY    = 5       # RPLidar quality (0-15), filters out noise
                         # lower to 5-7 if too many points are dropped

# Field (mm)
# Test field: 1040 x 1040   CDR field: 3000 x 2000
FIELD_W      = 3000
FIELD_H      = 2000
FIELD_MARGIN = 500       # CDR value: 500

# Clustering
CLUSTER_GAP_MM  = 80     # max distance between two consecutive points of a cluster
CLUSTER_MIN_PTS = 3      # a cluster this small is noise

# Classification : valeurs de départ à régler sur les scans réels.
CLUSTER_GAP_DEG = 6.0
WALL_MARGIN_MM = 55.0        # tolérance de pose et de mesure près d’un bord connu
WALL_MAP_MIN_LENGTH_MM = 100.0
WALL_MAP_MIN_FRACTION = 0.8
WALL_LONG_MIN_LENGTH_MM = 650.0
WALL_MAX_RMS_MM = 22.0        # dispersion perpendiculaire à la droite ajustée
OPP_MIN_SPAN_MM = 60.0       # taille observée de la face, pas du robot entier
OPP_MAX_SPAN_MM = 450.0

# Time tracking
MAX_MISSED_SCANS    = 5        # scans without detection before forgetting
ALPHA_SMOOTH        = 0.4      # exponential smoothing (0=very smooth, 1=raw)
MAX_OPP_SPEED_MM_S  = 2500     # plausible max opponent speed at the CDR
STALE_TIMEOUT_S     = 0.5      # freshness timeout

# LiDAR angular offset (degrees)
# 0   if the LiDAR cable points to the front of the robot
# 180 if it points to the rear
ANGLE_OFFSET_DEG = 0

# How long start() waits for the first successful connection.
CONNECT_TIMEOUT_S = 3.0

# ── SHARED STATE ──────────────────────────────────────────────────────────────

@dataclass
class _OpponentState:
    x:          float = 0.0
    y:          float = 0.0
    confidence: float = 0.0
    missed:     int   = 0
    timestamp:  Optional[float] = None

@dataclass(frozen=True)
class Cluster:
    """Groupe observé : centre et points dans le repère terrain si pose fournie."""
    categorie: str
    raison: str
    x_mm: float
    y_mm: float
    distance_mm: float
    longueur_mm: float
    largeur_mm: float
    rms_droite_mm: float
    points: Tuple[Tuple[float, float], ...]


@dataclass
class _RobotPose:
    x:     float = 0.0
    y:     float = 0.0
    theta: float = 0.0
    connue: bool = False

@dataclass
class _Status:
    connected:  bool = False
    last_error: str  = ""

_opponent      = _OpponentState()
_robot_pose    = _RobotPose()
_status        = _Status()
_opponent_lock = threading.Lock()
_pose_lock     = threading.Lock()
_status_lock   = threading.Lock()
_clusters_lock = threading.Lock()
_clusters: Tuple[Cluster, ...] = ()
_clusters_time: Optional[float] = None
_running       = False
_thread: Optional[threading.Thread] = None

# ── SCAN SOURCE AND TIME SOURCE ───────────────────────────────────────────────
# Defaults reproduce the historical behaviour exactly: the real LiDAR in its own
# thread, timestamped on the wall clock. Both are replaced by sim2d and replay.

_clock = DEFAULT_CLOCK
_scan_source_factory = None
_pushed_mode = False


def set_clock(clock) -> None:
    """Replace the time source used for timestamps and freshness.

    Must be called before start(). The speed gating below compares timestamps
    with a scan interval: mixing a virtual clock with wall-clock timestamps
    makes every detection look like an impossible jump, silently.

    Args:
        clock: object exposing now() and sleep(), see robot1.rasp.clock.Clock.
    """
    global _clock
    _clock = clock


def set_scan_source(factory) -> None:
    """Replace the LiDAR by any iterator of raw scans, kept in a thread.

    This is the streaming path, meant for replay: the source decides the pace,
    exactly as a serial port does.

    Args:
        factory: callable returning an iterable of scans. Each scan is a
            sequence of (quality, angle_deg, dist_mm) tuples, which is the
            exact shape the RPLidar SDK yields.

    Note:
        _process_scan() reads angle 0 as the +Y axis of the robot, with angles
        growing clockwise (x = d*sin, y = d*cos). A generator written with the
        usual convention produces mirrored scans and nothing reports it.
    """
    global _scan_source_factory, _pushed_mode
    _scan_source_factory = factory
    _pushed_mode = False


def use_pushed_scans() -> None:
    """Take scans from push_scan() instead of a LiDAR, with no thread.

    This is the deterministic path, meant for sim2d: the caller owns the loop,
    so a run is reproducible and can be stepped. start() then opens nothing and
    reports a connected LiDAR.
    """
    global _pushed_mode, _scan_source_factory
    _pushed_mode = True
    _scan_source_factory = None


def push_scan(raw_scan) -> None:
    """Process one raw scan synchronously, in the caller's thread.

    Args:
        raw_scan: sequence of (quality, angle_deg, dist_mm), see
            set_scan_source() for the angle convention.
    """
    _process_scan(raw_scan)

# ── PUBLIC API ────────────────────────────────────────────────────────────────

def update_robot_pose(x: float, y: float, theta: float) -> None:
    """Update the robot pose (Teensy odometry)."""
    with _pose_lock:
        _robot_pose.x     = x
        _robot_pose.y     = y
        _robot_pose.theta = theta
        _robot_pose.connue = True


def get_opponent() -> Optional[Tuple[float, float, float]]:
    """Return (x_mm, y_mm, confidence) in field coordinates, or None."""
    with _opponent_lock:
        if _opponent.confidence < 0.1 or _opponent.timestamp is None:
            return None
        if _clock.now() - _opponent.timestamp > STALE_TIMEOUT_S:
            return None
        return (_opponent.x, _opponent.y, _opponent.confidence)


def get_clusters() -> Tuple[Cluster, ...]:
    """Tous les groupes classés du dernier scan récent, murs compris."""
    with _clusters_lock:
        if _clusters_time is None or _clock.now() - _clusters_time > STALE_TIMEOUT_S:
            return ()
        return _clusters


def get_status() -> Tuple[bool, str]:
    """Return (connected, last_error) of the acquisition thread."""
    with _status_lock:
        return _status.connected, _status.last_error


def start(timeout_s: float = CONNECT_TIMEOUT_S) -> bool:
    """Start the acquisition thread and wait for the LiDAR to answer.

    Returns True once the LiDAR is connected. A failure is logged as an error
    and returned to the caller: an acquisition thread that dies on its own must
    never look like a working detection.
    """
    global _running, _thread
    if _thread and _thread.is_alive():
        logger.warning("Thread LiDAR deja actif.")
        return get_status()[0]

    with _status_lock:
        _status.connected  = False
        _status.last_error = ""

    _running = True

    if _pushed_mode:
        # Nothing to open and nobody to wait for: the caller pushes the scans.
        _set_status(True)
        logger.info("Scans pousses par l'appelant, aucun LiDAR ouvert.")
        return True

    _thread = threading.Thread(target=_lidar_loop, daemon=True, name="LidarDetect")
    _thread.start()

    deadline = _clock.now() + timeout_s
    while _clock.now() < deadline:
        connected, error = get_status()
        if connected:
            logger.info("Thread LiDAR demarre.")
            return True
        if error or not _thread.is_alive():
            break
        _clock.sleep(0.05)

    _, error = get_status()
    logger.error(
        "LiDAR indisponible sur %s : %s",
        PORT,
        error or f"aucune reponse en {timeout_s:.1f} s",
    )
    return False


def _clear_results():
    global _clusters, _clusters_time, _opponent
    with _clusters_lock:
        _clusters, _clusters_time = (), None
    with _opponent_lock:
        _opponent = _OpponentState()


def stop() -> None:
    """Stop the acquisition thread cleanly (waits up to 3 s)."""
    global _running
    _running = False
    if _pushed_mode:
        _set_status(False)
        _clear_results()
        logger.info("Detection arretee (scans pousses).")
        return
    if _thread:
        _thread.join(timeout=3.0)
    _clear_results()
    logger.info("Thread LiDAR arrete.")

# ── SCAN PROCESSING ───────────────────────────────────────────────────────────

def _classify(points, rx, ry, pose_known):
    """Mesure la forme d’un groupe puis donne une hypothèse et sa raison."""
    arr = np.asarray(points, dtype=float)
    centre = arr.mean(axis=0)
    centred = arr - centre
    # L’axe principal suit la direction la plus allongée du groupe.
    _, axes = np.linalg.eigh(centred.T @ centred)
    principal = centred @ axes[:, 1]
    lateral = centred @ axes[:, 0]
    longueur = float(np.ptp(principal))
    largeur = float(np.ptp(lateral))
    rms = float(np.sqrt(np.mean(lateral ** 2)))
    distance = math.hypot(centre[0] - rx, centre[1] - ry)

    # Distance aux quatre bords du terrain. Vérifier chaque bord séparément
    # évite de confondre deux côtés d’un coin avec un seul mur.
    near_wall = False
    if pose_known:
        x, y = arr.T
        distances = (abs(x), abs(x - FIELD_W), abs(y), abs(y - FIELD_H))
        near_wall = any(np.mean(d <= WALL_MARGIN_MM) >= WALL_MAP_MIN_FRACTION
                        for d in distances)
    if len(points) < CLUSTER_MIN_PTS:
        categorie, raison = "inconnu", "Trop peu de points pour estimer la forme"
    elif near_wall and longueur >= WALL_MAP_MIN_LENGTH_MM and rms <= WALL_MAX_RMS_MM:
        categorie, raison = "mur_probable", "Groupe rectiligne aligné avec un bord connu"
    elif longueur >= WALL_LONG_MIN_LENGTH_MM and rms <= WALL_MAX_RMS_MM:
        categorie, raison = "mur_probable", "Segment long et rectiligne (mur ou paroi fixe)"
    elif near_wall:
        categorie, raison = "inconnu", "Près d’un bord : mur et objet voisin difficiles à séparer"
    elif (pose_known and distance <= DETECT_DIST_MM
          and OPP_MIN_SPAN_MM <= longueur <= OPP_MAX_SPAN_MM
          and largeur <= OPP_MAX_SPAN_MM
          and 0 <= centre[0] <= FIELD_W and 0 <= centre[1] <= FIELD_H):
        categorie, raison = "adversaire_possible", "Taille compatible avec une face de robot"
    else:
        categorie, raison = "inconnu", "Pose, portée ou dimensions insuffisantes pour classer"
    return Cluster(categorie, raison, float(centre[0]), float(centre[1]),
                   distance, longueur, largeur, rms, tuple(points))


def _process_scan(raw_scan) -> None:
    """Filtre, projette, regroupe, classe et suit les candidats adverses."""
    global _clusters, _clusters_time
    with _pose_lock:
        rx, ry, rtheta = _robot_pose.x, _robot_pose.y, _robot_pose.theta
        pose_known = _robot_pose.connue
    cos_t, sin_t = math.cos(rtheta), math.sin(rtheta)
    samples = []
    for quality, angle_deg, dist_mm in raw_scan:
        if not all(math.isfinite(v) for v in (quality, angle_deg, dist_mm)):
            continue
        if not MIN_DIST_MM <= dist_mm <= MAX_SCAN_DIST_MM or quality < MIN_QUALITY:
            continue
        angle = (angle_deg + ANGLE_OFFSET_DEG) % 360.0
        rad = math.radians(angle)
        xr, yr = dist_mm * math.sin(rad), dist_mm * math.cos(rad)
        x_t = rx + xr * cos_t - yr * sin_t
        y_t = ry + xr * sin_t + yr * cos_t
        if pose_known and not (-FIELD_MARGIN <= x_t <= FIELD_W + FIELD_MARGIN
                              and -FIELD_MARGIN <= y_t <= FIELD_H + FIELD_MARGIN):
            continue
        samples.append((angle, x_t, y_t))
    samples.sort(key=lambda p: p[0])

    def proches(a, b):
        return ((b[0] - a[0]) % 360 <= CLUSTER_GAP_DEG
                and math.hypot(b[1] - a[1], b[2] - a[2]) <= CLUSTER_GAP_MM)

    # Séparer les surfaces aux ruptures de distance ou d’angle.
    groupes = []
    for point in samples:
        if not groupes or not proches(groupes[-1][-1], point):
            groupes.append([])
        groupes[-1].append(point)
    # Un objet traversant 359° -> 0° ne doit pas être coupé en deux.
    if len(groupes) > 1 and proches(groupes[-1][-1], groupes[0][0]):
        groupes[0] = groupes.pop() + groupes[0]

    groupes_classes = tuple(
        _classify([(x, y) for _, x, y in groupe], rx, ry, pose_known)
        for groupe in groupes
    )
    now = _clock.now()
    with _clusters_lock:
        _clusters = groupes_classes
        _clusters_time = now

    # Seuls les candidats compatibles sont transmis au suivi d’adversaire.
    candidats = [c for c in groupes_classes if c.categorie == "adversaire_possible"]
    choisi = min(candidats, key=lambda c: c.distance_mm, default=None)
    best = None if choisi is None else (choisi.x_mm, choisi.y_mm,
                                       min(1.0, len(choisi.points) / 8.0),
                                       len(choisi.points))
    best_dist = choisi.distance_mm if choisi else float("inf")

    # 5. Update with smoothing and gating
    now = _clock.now()
    with _opponent_lock:
        if best is None:
            _opponent.missed += 1
            if _opponent.missed > MAX_MISSED_SCANS:
                _opponent.confidence = 0.0
            return

        cx, cy, conf, npts = best

        # Gating: reject physically impossible jumps
        if (_opponent.confidence > 0.1 and _opponent.missed < MAX_MISSED_SCANS
                and _opponent.timestamp is not None
                and 0 <= now - _opponent.timestamp <= STALE_TIMEOUT_S):
            dt = max(now - _opponent.timestamp, 0.05)
            max_jump = MAX_OPP_SPEED_MM_S * dt
            jump = math.hypot(cx - _opponent.x, cy - _opponent.y)
            if jump > max_jump:
                logger.debug(f"GATING saut={jump:.0f}mm > max={max_jump:.0f}mm -> rejete")
                _opponent.missed += 1
                return
            # Exponential smoothing
            _opponent.x = ALPHA_SMOOTH * cx + (1 - ALPHA_SMOOTH) * _opponent.x
            _opponent.y = ALPHA_SMOOTH * cy + (1 - ALPHA_SMOOTH) * _opponent.y
        else:
            # New detection: no smoothing
            _opponent.x = cx
            _opponent.y = cy

        _opponent.confidence = conf
        _opponent.missed     = 0
        _opponent.timestamp  = now
        logger.debug(
            f"OPP ({_opponent.x:6.0f},{_opponent.y:6.0f}) "
            f"d={best_dist:5.0f}mm pts={npts} conf={conf:.2f}"
        )


def _mark_missed() -> None:
    """Increment the miss counter."""
    with _opponent_lock:
        _opponent.missed += 1
        if _opponent.missed > MAX_MISSED_SCANS:
            _opponent.confidence = 0.0

# ── ACQUISITION THREAD ────────────────────────────────────────────────────────

def _set_status(connected: bool, error: str = "") -> None:
    """Publish the state of the acquisition thread for get_status()."""
    with _status_lock:
        _status.connected = connected
        if error:
            _status.last_error = error


def _injected_scan_loop() -> None:
    """Consume scans from an injected source. Runs in a daemon thread."""
    try:
        scans = _scan_source_factory()
        _set_status(True)
        logger.info("Source de scans injectee, aucun LiDAR ouvert.")
        for scan in scans:
            if not _running:
                break
            _process_scan(scan)
    except Exception as exc:
        _set_status(False, str(exc))
        logger.error(f"Erreur de la source de scans : {exc}")
    finally:
        _set_status(False)
        logger.info("Thread source de scans termine.")


def _lidar_loop() -> None:
    """Acquisition loop. Runs in a daemon thread."""
    if _scan_source_factory is not None:
        _injected_scan_loop()
        return

    # Deferred on purpose: rplidar is a hardware driver, and importing it at
    # module level would make the whole perception layer unimportable on a
    # machine that has no LiDAR stack installed.
    from rplidar import RPLidar

    lidar = None
    try:
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        lidar._serial.flushInput()
        time.sleep(0.5)
        _set_status(True)
        logger.info(f"LiDAR connecte sur {PORT}.")

        try:
            scan_iter = lidar.iter_scans(max_buf_meas=1500, min_len=20)
        except TypeError:
            scan_iter = lidar.iter_scans()

        for scan in scan_iter:
            if not _running:
                break
            _process_scan(scan)

    except Exception as exc:
        _set_status(False, str(exc))
        logger.error(f"Erreur LiDAR sur {PORT} : {exc}")
    finally:
        _set_status(False)
        if lidar is not None:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
        logger.info("Thread LiDAR termine.")


def _afficher_groupes():
    for c in get_clusters():
        print(f"{c.categorie:22s} | centre=({c.x_mm:.0f}, {c.y_mm:.0f}) mm "
              f"| longueur={c.longueur_mm:.0f} mm | {len(c.points)} points | {c.raison}")
    print("Adversaire possible suivi :", get_opponent())


def _demo():
    """Scan simulé : un bord du terrain, une face de robot et un petit objet."""
    rx, ry = 700.0, 1000.0
    update_robot_pose(rx, ry, 0.0)
    points = ([(0, y) for y in range(720, 1281, 20)]
              + [(1120, y) for y in range(920, 1081, 20)]
              + [(1050, y) for y in range(1290, 1331, 10)])
    scan = [(15, math.degrees(math.atan2(x - rx, y - ry)) % 360,
             math.hypot(x - rx, y - ry)) for x, y in points]
    push_scan(scan)
    _afficher_groupes()


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Classe les groupes de points LiDAR.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--demo", action="store_true", help="Scan simulé, sans capteur")
    mode.add_argument("--live", action="store_true", help="Scans réels via acquisition.py")
    parser.add_argument("--x", type=float, help="X du robot immobile en mm")
    parser.add_argument("--y", type=float, help="Y du robot immobile en mm")
    parser.add_argument("--theta-deg", type=float, help="Orientation du robot en degrés")
    args = parser.parse_args()
    if args.demo:
        _demo()
        return
    if any(v is None or not math.isfinite(v) for v in (args.x, args.y, args.theta_deg)):
        parser.error("--live exige --x, --y et --theta-deg pour comparer les points au terrain")
    from . import acquisition
    update_robot_pose(args.x, args.y, math.radians(args.theta_deg))
    try:
        if not acquisition.start():
            raise RuntimeError(acquisition.get_status()[1] or "Aucun scan LiDAR récent")
        dernier_numero = None
        prochain_affichage = 0.0
        while True:
            _, erreur = acquisition.get_status()
            if erreur:
                raise RuntimeError(erreur)
            scan = acquisition.get_latest_scan()
            if scan is not None and scan.numero != dernier_numero:
                push_scan(scan.mesures)
                dernier_numero = scan.numero
                if time.monotonic() >= prochain_affichage:
                    print(f"\nScan {scan.numero}")
                    _afficher_groupes()
                    prochain_affichage = time.monotonic() + 1.0
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        acquisition.stop()


if __name__ == "__main__":
    main()
