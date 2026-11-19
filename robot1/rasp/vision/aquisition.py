""" ============================================================================
CONTRAINTES NUMERIQUES - ACQUISITION
- Distance <= 0 ou non finie : mesure ecartee. Qualite faible : conservee.
- Scan recent : age <= 0.5 s par defaut ; au-dela get_latest_scan() -> None.
- TIMEOUT + 3 s pour start ; TIMEOUT + 2 s pour stop.
- ATTENTION : seuil de qualite et reconnaissance des objets = detection.py.
============================================================================"""

#Acquisition unique du RPLIDAR, partagee entre localisation et detection. Chaque mesure est (qualite, angle_deg, distance_mm). Aucun filtrage metier.

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    from .lidar_config import BAUDRATE, PORT, TIMEOUT
except ModuleNotFoundError as exc:
    if exc.name != __package__ + '.lidar_config':
        raise
    # Les tests hors capteur restent possibles dans cette archive autonome.
    # La connexion reelle impose de copier ces fichiers dans le depot CDR
    # qui contient lidar_config.py ; aucune valeur de port n'est inventee.
    BAUDRATE, PORT, TIMEOUT = None, None, 3.0

logger = logging.getLogger('LIDAR_ACQUISITION')

@dataclass(frozen=True)
class Scan:
    numero: int
    timestamp: float
    mesures: tuple[tuple[int, float, float], ...]
    # ----------------------------------------------------------------------
    # FONCTION : age()
    # ROLE    : Calcule l anciennete du scan.
    # ENTREES : self : objet Scan contenant timestamp (horloge monotone).
    # RETOUR  : float : age en secondes.
    # A SAVOIR: Une grande anciennete rend le scan inutilisable pour la navigation.
    # ----------------------------------------------------------------------
    # CONTRAINTES : La duree est calculee avec time.monotonic() ; unite : secondes.
    # COMPARAISON CHIFFREE : Si timestamp=100.0 et horloge=100.2, age=0.2 s.
    def age(self) -> float:
        return time.monotonic() - self.timestamp

_lock = threading.Lock()
_stop = threading.Event()
_ready = threading.Event()
_thread: Optional[threading.Thread] = None
_latest: Optional[Scan] = None
_error: Optional[str] = None

# ----------------------------------------------------------------------
# FONCTION : get_latest_scan()
# ROLE    : Lit le dernier scan publie sans le consommer.
# ENTREES : max_age_s : age maximal accepte, par defaut 0,5 s.
# RETOUR  : Scan ou None si aucun scan suffisamment recent.
# A SAVOIR: Lecture protegee par verrou ; plusieurs modules peuvent utiliser le meme scan.
# ----------------------------------------------------------------------
# CONTRAINTES : Un scan est utilisable uniquement si son age ne depasse pas max_age_s (0.5 s par defaut).
# COMPARAISON CHIFFREE : Age 0.4 s <= 0.5 : accepte ; age 0.6 s > 0.5 : None.
def get_latest_scan(max_age_s: float = 0.5) -> Optional[Scan]:
    with _lock:
        scan = _latest
    return scan if scan is not None and 0 <= scan.age() <= max_age_s else None

# ----------------------------------------------------------------------
# FONCTION : get_status()
# ROLE    : Indique si le lecteur tourne et fournit des mesures recentes.
# ENTREES : Aucune.
# RETOUR  : tuple (actif: bool, erreur: str ou None).
# A SAVOIR: Un thread vivant sans scan recent ne signifie pas une acquisition fonctionnelle.
# ----------------------------------------------------------------------
# CONTRAINTES : En acquisition, actif implique thread vivant, arret non demande et scan recent.
# COMPARAISON CHIFFREE : Thread vivant mais scan age de 0.7 s > 0.5 : non actif.
def get_status():
    with _lock:
        error = _error
    alive = bool(_thread and _thread.is_alive() and not _stop.is_set())
    return alive and get_latest_scan() is not None, error

# ----------------------------------------------------------------------
# FONCTION : start()
# ROLE    : Demarre le lecteur materiel dans un thread dedie.
# ENTREES : timeout_s : temps maximal d attente du premier scan valide, en secondes.
# RETOUR  : bool : True si les scans sont disponibles ; sinon False.
# A SAVOIR: Ouvre le materiel indirectement via _read_loop ; ne pas ouvrir un second lecteur.
# ----------------------------------------------------------------------
# CONTRAINTES : Attend le premier scan pendant TIMEOUT+3 s par defaut ; ne cree pas un second lecteur actif.
# COMPARAISON CHIFFREE : TIMEOUT=3 s => attente maximale par defaut de 6 s.
def start(timeout_s: float = TIMEOUT + 3.0) -> bool:
    global _thread, _latest, _error
    if _thread and _thread.is_alive():
        return get_status()[0]
    with _lock:
        _latest, _error = None, None
    _stop.clear()
    _ready.clear()
    _thread = threading.Thread(target=_read_loop, name='LidarAcquisition', daemon=True)
    _thread.start()
    if not _ready.wait(timeout_s):
        with _lock:
            _error = 'Delai depasse : aucun scan valide recu.'
        _stop.set()
        return False
    return get_status()[0]

# ----------------------------------------------------------------------
# FONCTION : stop()
# ROLE    : Demande la fermeture du capteur et attend le thread.
# ENTREES : timeout_s : delai maximal avant erreur.
# RETOUR  : None ; peut lever TimeoutError si le thread reste actif.
# A SAVOIR: Efface le dernier scan ; appeler dans finally.
# ----------------------------------------------------------------------
# CONTRAINTES : Attend le thread au plus TIMEOUT+2 s ; une lecture bloquee declenche TimeoutError.
# COMPARAISON CHIFFREE : TIMEOUT=3 s => join de 5 s maximum.
def stop(timeout_s: float = TIMEOUT + 2.0) -> None:
    _stop.set()
    if _thread:
        _thread.join(timeout=timeout_s)
        if _thread.is_alive():
            raise TimeoutError('Lecture LiDAR encore bloquee : port occupe')
    with _lock:
        global _latest
        _latest = None

# ----------------------------------------------------------------------
# FONCTION : _clean_measurements()
# ROLE    : Nettoie un scan brut sans interpretation des objets.
# ENTREES : raw_scan : iterable de triplets (qualite, angle_deg, distance_mm).
# RETOUR  : tuple de triplets valides ; peut etre vide.
# A SAVOIR: Supprime NaN, infini, qualite negative, distance <= 0 ; normalise angle sur [0,360[.
# ----------------------------------------------------------------------
# CONTRAINTES : Rejette les valeurs non finies et les distances <= 0 ; ne rejette pas la qualite faible.
# COMPARAISON CHIFFREE : Distance 0 mm : rejetee ; 800 mm : gardee si qualite et angle finis.
def _clean_measurements(raw_scan):
    """Filtrage exclusivement physique/structurel, sans seuil de qualite."""
    cleaned = []
    for quality, angle, distance in raw_scan:
        if not all(math.isfinite(v) for v in (quality, angle, distance)):
            continue
        if quality < 0 or distance <= 0:
            continue
        cleaned.append((int(quality), float(angle) % 360.0, float(distance)))
    return tuple(cleaned)

# ----------------------------------------------------------------------
# FONCTION : _read_loop()
# ROLE    : Lit le port serie, construit et publie des objets Scan.
# ENTREES : Aucune entree explicite ; utilise PORT, BAUDRATE, TIMEOUT et les evenements globaux.
# RETOUR  : None ; met a jour _latest, _error et _ready.
# A SAVOIR: Seul endroit ouvrant le RPLIDAR dans ce module ; ferme moteur et port en sortie.
# ----------------------------------------------------------------------
# CONTRAINTES : Publie uniquement des scans non vides ; numerotation commence a 1 ; verrous pour le partage.
# COMPARAISON CHIFFREE : 0 mesure valide : scan ignore ; 12 mesures valides : scan publie.
def _read_loop():
    global _latest, _error
    lidar = None
    numero = 0
    try:
        if PORT is None or BAUDRATE is None:
            raise RuntimeError('lidar_config.py absent : installer dans le depot robot')
        from rplidar import RPLidar
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        for raw_scan in lidar.iter_scans(max_buf_meas=1500, min_len=20):
            if _stop.is_set():
                break
            mesures = _clean_measurements(raw_scan)
            if not mesures:
                continue
            numero += 1
            scan = Scan(numero, time.monotonic(), mesures)
            with _lock:
                _latest = scan
            _ready.set()
        if not _stop.is_set():
            raise RuntimeError('Fin inattendue du flux de scans')
    except Exception as exc:
        with _lock:
            _error = str(exc)
        logger.exception('Erreur d acquisition')
    finally:
        with _lock:
            _latest = None
        if lidar is not None:
            for fn in (lidar.stop, lidar.stop_motor, lidar.disconnect):
                try:
                    fn()
                except Exception:
                    logger.exception('Erreur de fermeture LiDAR')
        _ready.set()

# ----------------------------------------------------------------------
# FONCTION : main()
# ROLE    : Point d entree pour le diagnostic en console.
# ENTREES : Aucune ; configuration provenant du depot.
# RETOUR  : None ; tourne jusqu a interruption ou erreur.
# A SAVOIR: Affiche numeros et ages de scans ; aucun ordre aux moteurs du robot.
# ----------------------------------------------------------------------
# CONTRAINTES : Respecter les options CLI et arreter la lecture a la fin ; eviter un second processus sur le port serie.
# COMPARAISON CHIFFREE : --level 9 : test synthetique sans LiDAR ; --level 7 : test materiel avec pose.
def main():
    logging.basicConfig(level=logging.INFO)
    try:
        if not start():
            raise RuntimeError(get_status()[1] or 'Aucun scan')
        last = 0
        while True:
            _, err = get_status()
            if err:
                raise RuntimeError(err)
            scan = get_latest_scan()
            if scan and scan.numero != last:
                last = scan.numero
                print(f'Scan {scan.numero}: {len(scan.mesures)} points; age={scan.age():.3f}s; exemple={scan.mesures[0]}')
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop()

if __name__ == '__main__':
    main()
