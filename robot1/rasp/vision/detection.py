# ============================================================================
# SEUILS ET CONTRAINTES - VALEURS INITIALES A CALIBRER SUR LE ROBOT
#  1) MIN_DIST_MM=200 : distance 150 mm rejetee ; 300 mm conservee.
#  2) MAX_OBJECT_RANGE_MM=2500 : distance 2600 mm rejetee.
#  3) MIN_QUALITY=5 : qualite 4 rejetee ; qualite 5 autorisee.
#  4) CLUSTER_GAP_MM=55 : 50 mm peuvent relier ; 60 mm separent.
#  5) CLUSTER_GAP_DEG=6 : 5 deg peuvent relier ; 7 deg separent.
#  6) CLUSTER_MIN_PTS=4 : 3 points -> bruit (meme objet reel possible).
#  7) OBJECT_MIN_SPAN_MM=20 : contour de 15 mm -> bruit.
#  8) OBJECT_MAX_SPAN_MM=650 : contour de 700 mm -> ignore.
#  9) STONE_MATCH_RADIUS_MM=120 : distance au plan 100 mm ->
#     association possible ; 130 mm -> non associee.
# 10) STONE_MAX_SPAN_MM=180 : contour pierre 200 mm -> non pierre.
# 11) OPP_MIN_SPAN_MM=100 et OPP_MAX_SPAN_MM=450 :
#     face de 90 mm non candidate ; 200 mm candidate en taille ; 470 mm non.
# 12) DETECT_DIST_MM=1500 : cible a 1400 mm dans la portee adverse ;
#     cible a 1600 mm hors portee adverse, mais peut rester obstacle.
# 13) STALE_TIMEOUT_S=0.5 : scan vieux de 0.6 s non exploitable.
# 14) _jump_limit(0.01)=85 mm : saut 1000 mm en 10 ms rejete.
# 15) R_evitement=max(R_observe,R_physique)+R_robot+MARGE.
#     Ex.: max(30,50)+150+50 = 250 mm ; R_robot doit etre configure.
# IMPORTANT : LES COMPARAISONS SONT CONDITIONNELLES : plusieurs criteres
# doivent souvent etre vrais a la fois. Les tailles sont vues en COUPE 2D
# a la hauteur du faisceau, pas des dimensions physiques garanties.
# ============================================================================

"""Classe les groupes du LiDAR et suit un adversaire possible.

Entree : (qualite, angle en degres, distance en mm). La convention historique
est conservee : angle 0 = +Y local, angle 90 = +X local.

Acquisition partagee : update_robot_pose(x, y, theta), puis
push_scan(scan.mesures). Ne pas appeler start() en mode materiel en parallele
avec acquisition.py. L'ancienne API start/stop reste compatible avec app.py.

get_clusters() donne les classes et les points de chaque groupe recent.
get_opponent() garde le format historique (x_mm, y_mm, score) ou None.
get_opponent_position() renvoie (x_mm, y_mm) ou None.
Le suivi ignore les sauts trop grands et confirme une nouvelle cible
sur deux scans cohérents. Le score est un indice, pas une probabilité.
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

try:
    from .lidar_config import BAUDRATE, PORT, TIMEOUT
except ModuleNotFoundError as exc:
    if exc.name != __package__ + '.lidar_config':
        raise
    # Les tests hors capteur restent possibles dans cette archive autonome.
    # La connexion reelle impose de copier ces fichiers dans le depot CDR
    # qui contient lidar_config.py ; aucune valeur de port n'est inventee.
    BAUDRATE, PORT, TIMEOUT = None, None, 3.0

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
CLUSTER_GAP_MM  = 55     # max distance between two consecutive points of a cluster
CLUSTER_MIN_PTS = 4      # a cluster this small is noise

# Classification : valeurs de départ à régler sur les scans réels.
CLUSTER_GAP_DEG = 6.0
WALL_MARGIN_MM = 55.0        # tolérance de pose et de mesure près d’un bord connu
WALL_MAP_MIN_LENGTH_MM = 100.0
WALL_MAP_MIN_FRACTION = 0.8
WALL_LONG_MIN_LENGTH_MM = 650.0
WALL_MAX_RMS_MM = 22.0        # dispersion perpendiculaire à la droite ajustée
OPP_MIN_SPAN_MM = 100.0       # taille observée de la face, pas du robot entier
OPP_MAX_SPAN_MM = 450.0
# Les tailles sont celles de la section visible a la hauteur du LiDAR.
# Une pierre ne peut etre distinguee par sa taille seule de facon fiable.
OBJECT_MIN_SPAN_MM = 20.0
OBJECT_MAX_SPAN_MM = 650.0
STONE_MATCH_RADIUS_MM = 120.0  # association avec les pierres connues du terrain
STONE_MAX_SPAN_MM = 180.0  # a mesurer sur le scan reel, pas la taille au sol
MAX_OBJECT_RANGE_MM = 2500.0
IGNORE_OUTSIDE_FIELD = True
# Positions des pierres visibles a la hauteur du scan : a renseigner si connues.
STONE_POSITIONS_MM = ()  # plan du terrain, coordonnees X,Y en mm
LIDAR_HEIGHT_MM = None  # mesure a definir; la vue est une coupe 2D horizontale
ROBOT_RADIUS_MM = None  # OBLIGATOIRE pour produire un rayon d evitement
AVOIDANCE_MARGIN_MM = 50.0  # marge de securite a calibrer
STONE_PHYSICAL_RADIUS_MM = None  # rayon reel connu par le plan, si disponible
OPP_PHYSICAL_RADIUS_MM = None  # rayon physique adverse, si connu



# Time tracking
MAX_MISSED_SCANS    = 5        # scans without detection before forgetting
ALPHA_SMOOTH        = 0.4      # exponential smoothing (0=very smooth, 1=raw)
MAX_OPP_SPEED_MM_S  = 2500     # plausible max opponent speed at the CDR
POSITION_NOISE_MM   = 60.0     # marge de mesure à calibrer sur le robot
CONFIRM_SCANS       = 2        # mesures consécutives pour confirmer une cible
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
    raw_x: float = 0.0
    raw_y: float = 0.0
    hits: int = 0
    accepted: bool = False
    pending: Optional[Tuple[float, float, float, int]] = None
    taille_mm: float = 0.0
    rayon_mm: float = 0.0

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
    rayon_mm: float = 0.0  # rayon englobant observe, pas rayon physique exact
    distance_min_mm: float = 0.0  # distance au point visible le plus proche


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


# ----------------------------------------------------------------------
# FONCTION : set_clock()
# ROLE    : Remplace l horloge utilisee par la detection.
# ENTREES : clock : objet fournissant now() et sleep().
# RETOUR  : None.
# A SAVOIR: Utile pour simulation ; ne pas melanger horloges reelles et virtuelles.
# ----------------------------------------------------------------------
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


# ----------------------------------------------------------------------
# FONCTION : set_scan_source()
# ROLE    : Configure une source de scans en replay.
# ENTREES : factory : fonction qui fournit un iterable de scans bruts.
# RETOUR  : None.
# A SAVOIR: N ouvre pas directement le RPLIDAR ; desactive le mode push.
# ----------------------------------------------------------------------
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


# ----------------------------------------------------------------------
# FONCTION : use_pushed_scans()
# ROLE    : Active l injection synchrone de scans.
# ENTREES : Aucune.
# RETOUR  : None.
# A SAVOIR: Pour simulation ou acquisition partagee ; start() ne doit pas ouvrir de second port.
# ----------------------------------------------------------------------
def use_pushed_scans() -> None:
    """Take scans from push_scan() instead of a LiDAR, with no thread.

    This is the deterministic path, meant for sim2d: the caller owns the loop,
    so a run is reproducible and can be stepped. start() then opens nothing and
    reports a connected LiDAR.
    """
    global _pushed_mode, _scan_source_factory
    _pushed_mode = True
    _scan_source_factory = None


# ----------------------------------------------------------------------
# FONCTION : push_scan()
# ROLE    : Envoie un scan a la detection de maniere synchrone.
# ENTREES : raw_scan : sequence de (qualite, angle_deg, distance_mm).
# RETOUR  : None ; les resultats sont stockes dans l etat du module.
# A SAVOIR: Consulter ensuite get_clusters(), get_opponent() et get_obstacles().
# ----------------------------------------------------------------------
# CONTRAINTES : Ne pas appeler en doublon avec process_scan pour le meme scan.
# COMPARAISON CHIFFREE : 1 scan transmis 2 fois peut artificiellement confirmer une piste en 2 scans.
def push_scan(raw_scan) -> None:
    """Process one raw scan synchronously, in the caller's thread.

    Args:
        raw_scan: sequence of (quality, angle_deg, dist_mm), see
            set_scan_source() for the angle convention.
    """
    _process_scan(raw_scan)

# ── PUBLIC API ────────────────────────────────────────────────────────────────

# ----------------------------------------------------------------------
# FONCTION : update_robot_pose()
# ROLE    : Met a jour la position connue du robot pour projeter les retours LiDAR.
# ENTREES : x, y : coordonnees terrain en mm ; theta : orientation en radians.
# RETOUR  : None.
# A SAVOIR: Une pose erronee fausse les coordonnees des obstacles et leur association au plan.
# ----------------------------------------------------------------------
# CONTRAINTES : x/y en mm, theta en radians ; sans pose connue, association au plan incertaine.
# COMPARAISON CHIFFREE : theta=pi/2 correspond a 90 deg, pas theta=90.
def update_robot_pose(x: float, y: float, theta: float) -> None:
    """Update the robot pose (Teensy odometry)."""
    with _pose_lock:
        _robot_pose.x     = x
        _robot_pose.y     = y
        _robot_pose.theta = theta
        _robot_pose.connue = True


# ----------------------------------------------------------------------
# FONCTION : get_opponent()
# ROLE    : Recupere la derniere cible adverse validee.
# ENTREES : Aucune.
# RETOUR  : (x_mm, y_mm, score) ou None.
# A SAVOIR: Le score n est pas une probabilite ; cible absente signifie absence de mesure validee.
# ----------------------------------------------------------------------
# CONTRAINTES : Retourne None lorsque cible non acceptee ou derniere mesure trop ancienne (>0.5 s).
# COMPARAISON CHIFFREE : Mesure age 0.2 s : renvoyee si acceptee ; 0.7 s : None.
def get_opponent() -> Optional[Tuple[float, float, float]]:
    """Return (x_mm, y_mm, confidence) in field coordinates, or None."""
    with _opponent_lock:
        if not _opponent.accepted or _opponent.timestamp is None:
            return None
        if not 0 <= _clock.now() - _opponent.timestamp <= STALE_TIMEOUT_S:
            return None
        return (_opponent.x, _opponent.y, _opponent.confidence)


# ----------------------------------------------------------------------
# FONCTION : get_opponent_details()
# ROLE    : Recupere cible suivie et dimensions visibles.
# ENTREES : Aucune.
# RETOUR  : (x_mm, y_mm, score, taille_visible_mm, rayon_visible_mm) ou None.
# A SAVOIR: Dimensions de coupe LiDAR, pas dimensions du robot entier.
# ----------------------------------------------------------------------
# CONTRAINTES : Les dimensions sont celles de la section visible, non du volume entier ; suit la fraicheur du suivi.
# COMPARAISON CHIFFREE : Face visible 200 mm : taille=200 mm meme si chassis complet plus large.
def get_opponent_details():
    """Cible validee: x_mm, y_mm, score, taille_visible_mm, rayon_visible_mm.

    La taille est une mesure 2D du contour LiDAR, pas le gabarit total du robot.
    """
    opponent = get_opponent()
    if opponent is None:
        return None
    with _opponent_lock:
        return (*opponent, _opponent.taille_mm, _opponent.rayon_mm)


# ----------------------------------------------------------------------
# FONCTION : get_obstacles()
# ROLE    : Recupere les groupes pertinents pour l evitement.
# ENTREES : include_opponent : bool, False par defaut.
# RETOUR  : tuple de Cluster (pierres et obstacles ; eventuellement candidats adverses).
# A SAVOIR: Les candidats de taille adverse sont exclus par defaut ; activer le parametre pour tous.
# ----------------------------------------------------------------------
# CONTRAINTES : N expose que les groupes exploitables ; include_opponent controle la presence des candidats adverses.
# COMPARAISON CHIFFREE : include_opponent=False : obstacle et pierre seulement ; True : inclut adversaire_possible.
def get_obstacles(include_opponent: bool = False):
    """Objets utiles a l'evitement, hors bruit et murs non interpretes.

    Retourne des Cluster avec centre, rayon/longueur/largeur observes.
    """
    allowed = ('pierre_possible', 'obstacle', 'adversaire_possible') if include_opponent else ('pierre_possible', 'obstacle')
    return tuple(c for c in get_clusters() if c.categorie in allowed)


# ----------------------------------------------------------------------
# FONCTION : configure()
# ROLE    : Regle les seuils et les tailles de reference de la detection.
# ENTREES : stone_positions ; cluster_gap_mm ; opponent_min_span_mm ; opponent_max_span_mm ; lidar_height_mm ; robot_radius_mm ; avoidance_margin_mm ; stone_physical_radius_mm ; opponent_physical_radius_mm (tous optionnels).
# RETOUR  : None ; leve ValueError pour des valeurs invalides.
# A SAVOIR: A appeler avant la capture : modifie des constantes globales. Les tailles physiques sont des valeurs a mesurer.
# ----------------------------------------------------------------------
# CONTRAINTES : Seuils de distance et de taille >0 et finis ; min adversaire <= max ; rayon robot >0.
# COMPARAISON CHIFFREE : min=100, max=450 : valide ; min=500, max=450 : ValueError.
def configure(stone_positions=None, cluster_gap_mm=None, opponent_min_span_mm=None,
              opponent_max_span_mm=None, lidar_height_mm=None, robot_radius_mm=None,
              avoidance_margin_mm=None, stone_physical_radius_mm=None,
              opponent_physical_radius_mm=None):
    """Configure la detection avant le lancement, pas pendant un traitement."""
    global STONE_POSITIONS_MM, CLUSTER_GAP_MM, OPP_MIN_SPAN_MM, OPP_MAX_SPAN_MM
    global LIDAR_HEIGHT_MM, ROBOT_RADIUS_MM, AVOIDANCE_MARGIN_MM
    global STONE_PHYSICAL_RADIUS_MM, OPP_PHYSICAL_RADIUS_MM
    if stone_positions is not None:
        positions = tuple((float(x), float(y)) for x, y in stone_positions)
        if not all(math.isfinite(v) for point in positions for v in point):
            raise ValueError('Positions des pierres invalides')
        STONE_POSITIONS_MM = positions
    for name, value in (('CLUSTER_GAP_MM', cluster_gap_mm),
                        ('OPP_MIN_SPAN_MM', opponent_min_span_mm),
                        ('OPP_MAX_SPAN_MM', opponent_max_span_mm)):
        if value is not None:
            value = float(value)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} doit etre positif et fini')
            globals()[name] = value
    if OPP_MIN_SPAN_MM > OPP_MAX_SPAN_MM:
        raise ValueError('Intervalle de taille adversaire invalide')
    for name, value, allow_zero in (
        ('LIDAR_HEIGHT_MM', lidar_height_mm, True),
        ('ROBOT_RADIUS_MM', robot_radius_mm, False),
        ('AVOIDANCE_MARGIN_MM', avoidance_margin_mm, True),
        ('STONE_PHYSICAL_RADIUS_MM', stone_physical_radius_mm, False),
        ('OPP_PHYSICAL_RADIUS_MM', opponent_physical_radius_mm, False),
    ):
        if value is not None:
            value = float(value)
            if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
                raise ValueError(f'{name} invalide')
            globals()[name] = value


# ----------------------------------------------------------------------
# FONCTION : load_stones_from_plan()
# ROLE    : Charge les coordonnees des pierres depuis le plan.
# ENTREES : path : chemin vers JSON {"stones": [{"x_mm":..., "y_mm":...}]}.
# RETOUR  : int : nombre de positions chargees.
# A SAVOIR: Une pierre sur le plan n est pas consideree comme vue sans mesures LiDAR correspondantes.
# ----------------------------------------------------------------------
# CONTRAINTES : Le JSON doit fournir une liste stones et des champs x_mm/y_mm ; le plan ne prouve pas la presence.
# COMPARAISON CHIFFREE : 2 entrees dans stones => retourne 2 ; stones non-liste => ValueError.
def load_stones_from_plan(path):
    """Lit un fichier JSON {"stones": [{"x_mm": ..., "y_mm": ...}]}.

    Les pierres non visibles ne sont pas declarees detectees. Le plan sert
    uniquement a associer un groupe effectivement observe au bon objet.
    """
    import json
    from pathlib import Path
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    stones = data['stones']
    if not isinstance(stones, list):
        raise ValueError('stones doit etre une liste')
    positions = [(stone['x_mm'], stone['y_mm']) for stone in stones]
    configure(stone_positions=positions)
    return len(positions)


# ----------------------------------------------------------------------
# FONCTION : get_avoidance_objects()
# ROLE    : Prepare des enveloppes d objets pour la navigation.
# ENTREES : include_opponent : bool, True par defaut.
# RETOUR  : tuple de dictionnaires : categorie, centre X/Y, longueur, largeur, rayon observe, rayon evitement, indicateur de rayon physique connu.
# A SAVOIR: Necessite robot_radius_mm configure ; ne commande pas PathFinder, et rayons a valider sur le robot.
# ----------------------------------------------------------------------
# CONTRAINTES : Le rayon reel du robot est obligatoire ; rayon = max(rayon observe, connu) + rayon robot + marge.
# COMPARAISON CHIFFREE : Rayons 30/50 mm, robot 150 mm, marge 50 mm => rayon evitement 250 mm.
def get_avoidance_objects(include_opponent=True):
    """Contours pour navigation avec rayon observe ET marge conservative.

    Retourne des dictionnaires simples. Le rayon de securite est une estimation,
    pas une garantie de collision impossible. Les rayons physiques fournis sont
    preferes quand ils sont connus. La projection actuelle repose sur des cercles;
    longueur/largeur restent disponibles pour une future inflation orientee.
    """
    if ROBOT_RADIUS_MM is None:
        raise ValueError('Configurer robot_radius_mm (rayon du robot reel) avant evitement')
    objects = []
    for c in get_obstacles(include_opponent=include_opponent):
        physical = (STONE_PHYSICAL_RADIUS_MM if c.categorie == 'pierre_possible'
                    else OPP_PHYSICAL_RADIUS_MM if c.categorie == 'adversaire_possible'
                    else None)
        radius = max(c.rayon_mm, physical or 0.0)
        objects.append({
            'categorie': c.categorie, 'x_mm': c.x_mm, 'y_mm': c.y_mm,
            'longueur_mm': c.longueur_mm, 'largeur_mm': c.largeur_mm,
            'rayon_observe_mm': c.rayon_mm,
            'rayon_evitement_mm': radius + ROBOT_RADIUS_MM + AVOIDANCE_MARGIN_MM,
            'rayon_physique_connu': physical is not None,
        })
    return tuple(objects)


# ----------------------------------------------------------------------
# FONCTION : get_opponent_position()
# ROLE    : Expose uniquement les coordonnees de la cible suivie.
# ENTREES : Aucune.
# RETOUR  : (x_mm, y_mm) ou None.
# A SAVOIR: Forme simplifiee de get_opponent().
# ----------------------------------------------------------------------
# CONTRAINTES : Retour x/y uniquement lorsque get_opponent fournit un suivi recent.
# COMPARAISON CHIFFREE : get_opponent=None => get_opponent_position=None.
def get_opponent_position() -> Optional[Tuple[float, float]]:
    """Position validée dans le repère terrain, en mm, ou None."""
    result = get_opponent()
    return None if result is None else (result[0], result[1])


# ----------------------------------------------------------------------
# FONCTION : get_clusters()
# ROLE    : Lit tous les groupes utiles du dernier scan recent.
# ENTREES : Aucune.
# RETOUR  : tuple de Cluster ou tuple vide si perime.
# A SAVOIR: Les groupes etiquetes bruit/ignore ne sont pas publies.
# ----------------------------------------------------------------------
# CONTRAINTES : Ne renvoie pas les classes bruit/ignore et expire si dernier scan >0.5 s.
# COMPARAISON CHIFFREE : Scan age 0.3 s : groupes ; scan age 0.8 s : tuple vide.
def get_clusters() -> Tuple[Cluster, ...]:
    """Groupes d'objets observables du dernier scan recent (sans bruit)."""
    with _clusters_lock:
        if _clusters_time is None or _clock.now() - _clusters_time > STALE_TIMEOUT_S:
            return ()
        return _clusters


# ----------------------------------------------------------------------
# FONCTION : get_status()
# ROLE    : Lit l etat de connexion de la detection.
# ENTREES : Aucune.
# RETOUR  : (connected: bool, last_error: str).
# A SAVOIR: En mode push, l etat depend de la gestion des scans pousses.
# ----------------------------------------------------------------------
# CONTRAINTES : En acquisition, actif implique thread vivant, arret non demande et scan recent.
# COMPARAISON CHIFFREE : Thread vivant mais scan age de 0.7 s > 0.5 : non actif.
def get_status() -> Tuple[bool, str]:
    """Return (connected, last_error) of the acquisition thread."""
    with _status_lock:
        return _status.connected, _status.last_error


# ----------------------------------------------------------------------
# FONCTION : start()
# ROLE    : Demarre la detection en mode materiel ou source configuree.
# ENTREES : timeout_s : delai de connexion autorise.
# RETOUR  : bool : True si source active, sinon False.
# A SAVOIR: Avec acquisition.py deja actif, preferer push_scan et ne pas demarrer deux lecteurs.
# ----------------------------------------------------------------------
# CONTRAINTES : Attend le premier scan pendant TIMEOUT+3 s par defaut ; ne cree pas un second lecteur actif.
# COMPARAISON CHIFFREE : TIMEOUT=3 s => attente maximale par defaut de 6 s.
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


# ----------------------------------------------------------------------
# FONCTION : _clear_results()
# ROLE    : Remet a zero les resultats partages et le suivi adverse.
# ENTREES : Aucune.
# RETOUR  : None.
# A SAVOIR: Efface toute cible et tous les groupes memorises.
# ----------------------------------------------------------------------
def _clear_results():
    global _clusters, _clusters_time, _opponent
    with _clusters_lock:
        _clusters, _clusters_time = (), None
    with _opponent_lock:
        _opponent = _OpponentState()


# ----------------------------------------------------------------------
# FONCTION : stop()
# ROLE    : Arrete la detection et libere les ressources eventuelles.
# ENTREES : Aucune.
# RETOUR  : None.
# A SAVOIR: En mode pousse, ne ferme pas le capteur geré par acquisition.py.
# ----------------------------------------------------------------------
# CONTRAINTES : Attend le thread au plus TIMEOUT+2 s ; une lecture bloquee declenche TimeoutError.
# COMPARAISON CHIFFREE : TIMEOUT=3 s => join de 5 s maximum.
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

# ----------------------------------------------------------------------
# FONCTION : _classify()
# ROLE    : Mesure et classe un groupe de points deja constitue.
# ENTREES : points : liste de (x,y) terrain ; rx, ry : position du robot ; pose_known : bool.
# RETOUR  : Cluster : categorie, justification, centre, dimensions, points et rayon.
# A SAVOIR: Ordre des regles : bruit / hors zone / trop grand / pierre proche du plan / adversaire possible / obstacle.
# ----------------------------------------------------------------------
# CONTRAINTES : Bruit si <4 points OU taille <20 mm ; ignorer >650 mm ; pierre <=180 mm et proche du plan <=120 mm ; adversaire 100..450 mm et <=1500 mm.
# COMPARAISON CHIFFREE : 3 points : bruit ; 4 points et 50 mm : possible objet ; contour adverse 90 mm : hors seuil.
def _classify(points, rx, ry, pose_known):
    """Classe les groupes sans chercher des murs (situes sous le faisceau).

    Une pierre n'est etiquetee pierre_possible que par rapprochement d'une
    position de depart connue ; sinon un petit groupe reste 'obstacle'.
    """
    arr = np.asarray(points, dtype=float)
    centre = arr.mean(axis=0)
    centred = arr - centre
    _, axes = np.linalg.eigh(centred.T @ centred)
    principal = centred @ axes[:, 1]
    lateral = centred @ axes[:, 0]
    longueur = float(np.ptp(principal))
    largeur = float(np.ptp(lateral))
    rms = float(np.sqrt(np.mean(lateral ** 2)))
    rayon = float(np.max(np.linalg.norm(centred, axis=1)))
    distance = math.hypot(centre[0] - rx, centre[1] - ry)
    distance_min = float(np.min(np.linalg.norm(arr - np.array([rx, ry]), axis=1)))
    taille = max(longueur, largeur)
    valid_field = (not pose_known or 0 <= centre[0] <= FIELD_W
                   and 0 <= centre[1] <= FIELD_H)
    if len(points) < CLUSTER_MIN_PTS or taille < OBJECT_MIN_SPAN_MM:
        categorie, raison = 'bruit', 'Points insuffisants ou contour trop petit'
    elif not valid_field or distance_min > MAX_OBJECT_RANGE_MM:
        categorie, raison = 'ignore', 'Hors terrain ou hors portee objets'
    elif taille > OBJECT_MAX_SPAN_MM:
        categorie, raison = 'ignore', 'Surface trop etendue : non exploitable'
    elif (pose_known and taille <= STONE_MAX_SPAN_MM and any(
          math.hypot(centre[0] - sx, centre[1] - sy) <= STONE_MATCH_RADIUS_MM
          for sx, sy in STONE_POSITIONS_MM)):
        categorie, raison = 'pierre_possible', 'Proche d une pierre connue au depart'
    elif (pose_known and distance <= DETECT_DIST_MM and
          OPP_MIN_SPAN_MM <= longueur <= OPP_MAX_SPAN_MM and
          largeur <= OPP_MAX_SPAN_MM):
        categorie, raison = 'adversaire_possible', 'Contour compatible, identification non certaine'
    else:
        categorie, raison = 'obstacle', 'Objet reel vu mais nature indeterminee'
    return Cluster(categorie, raison, float(centre[0]), float(centre[1]),
                   distance, longueur, largeur, rms, tuple(points), rayon, distance_min)


# ----------------------------------------------------------------------
# FONCTION : process_scan()
# ROLE    : Interface publique synchrone de traitement de scan.
# ENTREES : raw_scan : sequence des mesures brutes.
# RETOUR  : None.
# A SAVOIR: Appelle _process_scan et met a jour les resultats consultables.
# ----------------------------------------------------------------------
# CONTRAINTES : Meme format que acquisition.Scan.mesures ; traite une seule fois chaque scan pour eviter les doublons.
# COMPARAISON CHIFFREE : Un scan de 100 mesures peut produire 0, 1 ou plusieurs groupes.
def process_scan(raw_scan):
    """API synchrone publique pour scans issus d acquisition.py."""
    _process_scan(raw_scan)


# ----------------------------------------------------------------------
# FONCTION : _process_scan()
# ROLE    : Filtre, projette, regroupe et classe les retours du capteur.
# ENTREES : raw_scan : triplets (qualite, angle_deg, distance_mm).
# RETOUR  : None ; met a jour _clusters et _opponent.
# A SAVOIR: 0 deg = +Y local et 90 deg = +X ; controle des transitions 359/0 deg.
# ----------------------------------------------------------------------
# CONTRAINTES : Qualite >=5 ; distance 200..2500 mm ; points hors terrain rejetes si pose connue ; liaison <=6 deg ET <=55 mm.
# COMPARAISON CHIFFREE : Qualite 4 : rejet ; qualite 5 : possible ; voisins 40 mm et 4 deg : meme groupe ; 70 mm : separes.
def _process_scan(raw_scan) -> None:
    """Filtre les points, regroupe les contours, renvoie seulement objets utiles."""
    global _clusters, _clusters_time
    with _pose_lock:
        rx, ry, rtheta = _robot_pose.x, _robot_pose.y, _robot_pose.theta
        pose_known = _robot_pose.connue
    cos_t, sin_t = math.cos(rtheta), math.sin(rtheta)
    samples = []
    for quality, angle_deg, dist_mm in raw_scan:
        if not all(math.isfinite(v) for v in (quality, angle_deg, dist_mm)):
            continue
        if not MIN_DIST_MM <= dist_mm <= MAX_OBJECT_RANGE_MM or quality < MIN_QUALITY:
            continue
        angle = (angle_deg + ANGLE_OFFSET_DEG) % 360.0
        rad = math.radians(angle)
        xr, yr = dist_mm * math.sin(rad), dist_mm * math.cos(rad)
        x_t = rx + xr * cos_t - yr * sin_t
        y_t = ry + xr * sin_t + yr * cos_t
        # N'interprete pas les retours hors terrain comme des obstacles.
        if pose_known and IGNORE_OUTSIDE_FIELD and not (0 <= x_t <= FIELD_W and 0 <= y_t <= FIELD_H):
            continue
        samples.append((angle, x_t, y_t))
    samples.sort(key=lambda p: p[0])

    # ----------------------------------------------------------------------
    # FONCTION : proches()
    # ROLE    : Decide si deux points LiDAR voisins appartiennent au meme groupe.
    # ENTREES : a, b : tuples (angle_deg, x_mm, y_mm).
    # RETOUR  : bool : True si ecart angulaire et distance dans les deux seuils.
    # A SAVOIR: Fonction locale a _process_scan ; 55 mm est le pas de liaison, pas le rayon physique.
    # ----------------------------------------------------------------------
    # CONTRAINTES : DEUX conditions simultanees : difference d angle <=6 deg et distance euclidienne <=55 mm.
    # COMPARAISON CHIFFREE : 5 deg + 54 mm => True ; 7 deg + 30 mm => False ; 4 deg + 56 mm => False.
    def proches(a, b):
        return ((b[0] - a[0]) % 360 <= CLUSTER_GAP_DEG
                and math.hypot(b[1] - a[1], b[2] - a[2]) <= CLUSTER_GAP_MM)

    groupes = []
    for point in samples:
        if not groupes or not proches(groupes[-1][-1], point):
            groupes.append([])
        groupes[-1].append(point)
    if len(groupes) > 1 and proches(groupes[-1][-1], groupes[0][0]):
        groupes[0] = groupes.pop() + groupes[0]

    classes = tuple(_classify([(x, y) for _, x, y in group], rx, ry, pose_known)
                    for group in groupes)
    # Ne publie pas de bruit, murs supposes ou points sans objet exploitable.
    groupes_classes = tuple(c for c in classes if c.categorie not in ('bruit', 'ignore'))
    now = _clock.now()
    with _clusters_lock:
        _clusters, _clusters_time = groupes_classes, now
    candidats = [c for c in groupes_classes if c.categorie == 'adversaire_possible']
    _update_opponent(candidats, now)


# ----------------------------------------------------------------------
# FONCTION : _jump_limit()
# ROLE    : Calcule la distance maximale plausible de deplacement adverse.
# ENTREES : dt : temps depuis une mesure validee en secondes.
# RETOUR  : float : limite en mm.
# A SAVOIR: Vitesse maximale * dt + marge de bruit ; seuil a calibrer.
# ----------------------------------------------------------------------
# CONTRAINTES : Distance maximale en mm = 60 mm + 2500 mm/s * dt en s.
# COMPARAISON CHIFFREE : dt=0.01 s => 85 mm ; saut=1000 mm >85 mm => rejete.
def _jump_limit(dt: float) -> float:
    # Distance possible depuis la dernière mesure acceptée, plus marge de bruit.
    return POSITION_NOISE_MM + MAX_OPP_SPEED_MM_S * dt


# ----------------------------------------------------------------------
# FONCTION : _update_opponent()
# ROLE    : Suit une cible et rejette les changements incoherents.
# ENTREES : candidats : liste de Cluster adversaire_possible ; now : horodatage de l horloge choisie.
# RETOUR  : None ; met a jour l etat _opponent.
# A SAVOIR: Exige deux scans pour acquisition ; mesure l ecart entre positions brutes, pas lissees.
# ----------------------------------------------------------------------
# CONTRAINTES : Nouveau suivi valide apres 2 scans coherents ; suivi encore plausible si dt<=0.5 s et moins de 5 echecs ; lissage alpha=0.4.
# COMPARAISON CHIFFREE : Ancien x=1000, observation x=1100 => nouveau x lisse=1040 ; saut hors limite : refuse.
def _update_opponent(candidats, now: float) -> None:
    with _opponent_lock:
        state = _opponent
        # Une mesure refusée ne doit pas être présentée comme une mesure fraîche.
        # On conserve néanmoins le dernier point accepté pour comparer le scan suivant.
        state.accepted = False
        dt = None if state.timestamp is None else now - state.timestamp
        tracking = (dt is not None and 0 <= dt <= STALE_TIMEOUT_S
                    and state.missed < MAX_MISSED_SCANS)

        if tracking:
            # Compare au point BRUT précédent : le lissage introduit un retard.
            limite = _jump_limit(dt)
            choix = min(candidats, key=lambda c: math.hypot(
                c.x_mm - state.raw_x, c.y_mm - state.raw_y), default=None)
            saut = (math.hypot(choix.x_mm - state.raw_x, choix.y_mm - state.raw_y)
                    if choix is not None else float("inf"))
            if choix is None or saut > limite:
                state.missed += 1
                state.hits = 0
                logger.debug("Scan ignoré : saut %.0f mm, limite %.0f mm", saut, limite)
                return
            state.x += ALPHA_SMOOTH * (choix.x_mm - state.x)
            state.y += ALPHA_SMOOTH * (choix.y_mm - state.y)
            state.hits += 1
        else:
            # Au départ ou après une perte, exiger deux scans cohérents.
            # Une position isolée ne suffit pas à créer une nouvelle cible.
            state.timestamp = None
            state.confidence = 0.0
            state.hits = 0
            if not candidats:
                state.pending = None
                return
            choix = None
            hits = 1
            if state.pending is not None:
                px, py, ptime, phits = state.pending
                pdt = now - ptime
                if 0 < pdt <= STALE_TIMEOUT_S:
                    proche = min(candidats, key=lambda c: math.hypot(
                        c.x_mm - px, c.y_mm - py))
                    if math.hypot(proche.x_mm - px, proche.y_mm - py) <= _jump_limit(pdt):
                        choix, hits = proche, phits + 1
            if choix is None:
                choix = min(candidats, key=lambda c: c.distance_mm)
            if hits < CONFIRM_SCANS:
                state.pending = (choix.x_mm, choix.y_mm, now, hits)
                return
            state.x, state.y = choix.x_mm, choix.y_mm
            state.hits = hits

        state.pending = None
        state.taille_mm = max(choix.longueur_mm, choix.largeur_mm)
        state.rayon_mm = choix.rayon_mm
        state.raw_x, state.raw_y = choix.x_mm, choix.y_mm
        # Le score mélange densité du groupe et stabilité sur plusieurs scans.
        densite = min(1.0, len(choix.points) / 8.0)
        stabilite = min(1.0, state.hits / 4.0)
        state.confidence = 0.6 * densite + 0.4 * stabilite
        state.missed = 0
        state.timestamp = now
        state.accepted = True


# ── ACQUISITION THREAD ────────────────────────────────────────────────────────

# ----------------------------------------------------------------------
# FONCTION : _set_status()
# ROLE    : Publie le statut interne du lecteur de detection.
# ENTREES : connected : bool ; error : message optionnel.
# RETOUR  : None.
# A SAVOIR: Protege par verrou ; un message erreur peut persister.
# ----------------------------------------------------------------------
def _set_status(connected: bool, error: str = "") -> None:
    """Publish the state of the acquisition thread for get_status()."""
    with _status_lock:
        _status.connected = connected
        if error:
            _status.last_error = error


# ----------------------------------------------------------------------
# FONCTION : _injected_scan_loop()
# ROLE    : Consomme une source de scans injectee dans un thread.
# ENTREES : Aucune ; utilise _scan_source_factory.
# RETOUR  : None.
# A SAVOIR: Pour replay et simulation ; aucune ouverture du port LiDAR.
# ----------------------------------------------------------------------
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


# ----------------------------------------------------------------------
# FONCTION : _lidar_loop()
# ROLE    : Lit le materiel directement dans le mode historique.
# ENTREES : Aucune ; utilise PORT, BAUDRATE, TIMEOUT et _running.
# RETOUR  : None.
# A SAVOIR: Ne jamais activer en meme temps qu acquisition.py sur le meme port.
# ----------------------------------------------------------------------
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
        if PORT is None or BAUDRATE is None:
            raise RuntimeError('lidar_config.py absent : installer dans le depot robot')
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


# ----------------------------------------------------------------------
# FONCTION : _afficher_groupes()
# ROLE    : Affiche les groupes et la cible pour debug.
# ENTREES : Aucune.
# RETOUR  : None ; ecrit dans le terminal.
# A SAVOIR: Pas d effet sur le robot.
# ----------------------------------------------------------------------
def _afficher_groupes():
    for c in get_clusters():
        print(f"{c.categorie:22s} | centre=({c.x_mm:.0f}, {c.y_mm:.0f}) mm "
              f"| longueur={c.longueur_mm:.0f} mm | {len(c.points)} points | {c.raison}")
    print("Adversaire possible suivi :", get_opponent_details())
    print("Obstacles (dont pierres) :", [(c.categorie, round(c.x_mm), round(c.y_mm),
          round(c.rayon_mm)) for c in get_obstacles()])


# ----------------------------------------------------------------------
# FONCTION : _demo()
# ROLE    : Fabrique des scans artificiels pour illustrer les categories.
# ENTREES : Aucune.
# RETOUR  : None ; injecte deux scans et affiche les resultats.
# A SAVOIR: Demonstration et non validation de precision du capteur reel.
# ----------------------------------------------------------------------
def _demo():
    """Scan simulé : un bord du terrain, une face de robot et un petit objet."""
    rx, ry = 700.0, 1000.0
    update_robot_pose(rx, ry, 0.0)
    points = ([(0, y) for y in range(720, 1281, 20)]
              + [(1120, y) for y in range(920, 1081, 20)]
              + [(1050, y) for y in range(1290, 1331, 10)])
    scan = [(15, math.degrees(math.atan2(x - rx, y - ry)) % 360,
             math.hypot(x - rx, y - ry)) for x, y in points]
    # Deux scans simulés successifs pour confirmer la cible.
    push_scan(scan)
    time.sleep(0.02)
    push_scan(scan)
    _afficher_groupes()


# ----------------------------------------------------------------------
# FONCTION : main()
# ROLE    : Interface en ligne de commande de la detection.
# ENTREES : Arguments CLI : --demo ou --live, pose x/y/theta selon le mode.
# RETOUR  : None ; affiche les resultats, ou une erreur en cas de probleme.
# A SAVOIR: Le mode --live passe par acquisition.py ; robot immobile pour la pose manuelle.
# ----------------------------------------------------------------------
# CONTRAINTES : Respecter les options CLI et arreter la lecture a la fin ; eviter un second processus sur le port serie.
# COMPARAISON CHIFFREE : --level 9 : test synthetique sans LiDAR ; --level 7 : test materiel avec pose.
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Classe les groupes de points LiDAR.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--demo", action="store_true", help="Scan simulé, sans capteur")
    mode.add_argument("--live", action="store_true", help="Scans réels via acquisition.py")
    parser.add_argument("--x", type=float, help="X du robot immobile en mm")
    parser.add_argument("--y", type=float, help="Y du robot immobile en mm")
    parser.add_argument("--theta-deg", type=float, help="Orientation du robot en degrés")
    parser.add_argument("--cluster-gap-mm", type=float, default=CLUSTER_GAP_MM)
    parser.add_argument("--opp-min-span-mm", type=float, default=OPP_MIN_SPAN_MM)
    args = parser.parse_args()
    if args.demo:
        _demo()
        return
    if any(v is None or not math.isfinite(v) for v in (args.x, args.y, args.theta_deg)):
        parser.error("--live exige --x, --y et --theta-deg pour comparer les points au terrain")
    configure(cluster_gap_mm=args.cluster_gap_mm, opponent_min_span_mm=args.opp_min_span_mm)
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
