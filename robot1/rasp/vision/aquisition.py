"""Acquisition partagee du RPLIDAR, sans detection ni localisation.

Ne pas lancer en meme temps que les anciens lecteurs de detection/localization.
"""

"""
Chaque mesure conserve ce format : (qualite, angle_en_degres, distance_en_millimetres)
Le programme élimine seulement les mesures invalides, par exemple une distance nulle. Les filtres spécifiques aux balises ou à l’adversaire seront appliqués dans leurs traitements respectifs.

Le programme lit les mesures du LiDAR une seule fois et met le dernier scan à disposition des autres modules. La détection de l’adversaire et la localisation peuvent ainsi travailler sur les mêmes mesures, sans ouvrir chacune une connexion au capteur.
Pour chaque point, il fournit (qualité, angle en degrés, distance en millimètres). Les autres modules utilisent ces informations pour regrouper les points et identifier les objets : robot adverse, mur, balise…
Donc, il n’identifie pas lui-même les objets et n’élimine pas encore les points de faible qualité : ces traitements sont effectués ensuite.
Il élimine également les mesures invalides, notamment les distances nulles ou négatives et les valeurs numériques non finies.
Pour chaque scan, le programme :
1.	Parcourt les points un par un.
2.	Vérifie que leurs valeurs sont valides.
3.	Conserve les points valides avec leur qualité, angle et distance.
4.	Met le scan complet filtré à disposition des autres modules.

Exemple de sortie :
(
    (12, 30.0, 800.0),
    (11, 31.0, 810.0),
    (12, 32.0, 805.0),
)
-> (qualité, angle en degrés, distance en millimètres)
-> Liste de tuples, chaque tuple représentant un point mesuré par le LiDAR.
"""

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

from rplidar import RPLidar

from .lidar_config import BAUDRATE, PORT, TIMEOUT

logger = logging.getLogger("LIDAR_ACQUISITION")
@dataclass(frozen=True)


# Regroupe le numéro du scan, son horodatage et ses mesures.
class Scan:
    numero: int
    # Instant de reception du scan complet, pas celui de chaque point.
    timestamp: float
    # Format brut conserve : (qualite, angle_degres, distance_mm).
    mesures: tuple[tuple[int, float, float], ...]

    def age(self) -> float:
        return time.monotonic() - self.timestamp


_lock = threading.Lock()
_stop = threading.Event()
_ready = threading.Event()
_thread: Optional[threading.Thread] = None
_latest: Optional[Scan] = None
_error: Optional[str] = None


# Renvoie le dernier scan récent, ou None si aucune donnée récente n’est disponible.
def get_latest_scan(max_age_s: float = 0.5) -> Optional[Scan]:
    """Dernier scan recent, ou None. Lecture non destructive, partageable."""
    with _lock:
        scan = _latest
    if scan is None or scan.age() > max_age_s:
        return None
    return scan


#  Renvoie l’état de l’acquisition et la dernière erreur éventuelle.
def get_status():
    """Retourne (acquisition_active, derniere_erreur).

    Active exige un thread vivant ET un scan recent, pas juste un port ouvert.
    """
    with _lock:
        error = _error
    active = bool(_thread and _thread.is_alive() and not _stop.is_set())
    return active and get_latest_scan() is not None, error


# Démarre la lecture et attend le premier scan valide. Renvoie True si elle fonctionne.
def start(timeout_s: float = TIMEOUT + 3.0) -> bool:
    """A appeler depuis le thread principal ; attend le premier scan valide."""
    global _thread, _latest, _error
    if _thread and _thread.is_alive():
        return get_status()[0]

    with _lock:
        _latest = None
        _error = None
    _stop.clear()
    _ready.clear()
    _thread = threading.Thread(target=_read_loop, name="LidarAcquisition", daemon=True)
    _thread.start()
    if not _ready.wait(timeout_s):
        with _lock:
            _error = "Delai depasse : aucun scan valide recu."
        _stop.set()
        return False
    return get_status()[0]


# Demande l’arrêt, attend la fin de la lecture et libère le capteur.
def stop(timeout_s: float = TIMEOUT + 2.0) -> None:
    """Demande l'arret et attend la liberation du capteur."""
    _stop.set()
    if _thread:
        _thread.join(timeout=timeout_s)
        if _thread.is_alive():
            raise TimeoutError("Lecture LiDAR encore bloquee ; ne pas ouvrir un autre lecteur.")


# Ouvre le port du LiDAR et récupère les scans en continu.
def _read_loop():
    global _latest, _error
    lidar = None
    numero = 0
    try:
        lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        for raw_scan in lidar.iter_scans(max_buf_meas=1500, min_len=20):
            if _stop.is_set():
                break
            # Retirer seulement les mesures invalides. Les filtres metier
            # (distance/qualite) appartiennent aux traitements suivants.
            mesures = tuple(
                (int(q), float(a), float(d))
                for q, a, d in raw_scan
                if math.isfinite(q) and math.isfinite(a)
                and math.isfinite(d) and d > 0
            )
            if not mesures:
                continue
            numero += 1
            scan = Scan(numero, time.monotonic(), mesures)
            with _lock:
                _latest = scan
            _ready.set()
        if not _stop.is_set():
            raise RuntimeError("Le flux de scans s'est termine de facon inattendue.")
    except Exception as exc:
        with _lock:
            _error = str(exc)
        logger.exception("Erreur d'acquisition LiDAR")
    finally:
        with _lock:
            _latest = None
        if lidar is not None:
            # Une erreur d'arret moteur ne doit pas empecher la deconnexion.
            for operation in (lidar.stop, lidar.stop_motor, lidar.disconnect):
                try:
                    operation()
                except Exception:
                    logger.exception("Erreur pendant la fermeture du LiDAR")
        _ready.set()


# Permet de tester le fichier directement dans la console.
def main():
    """Test console uniquement : aucune commande des moteurs du robot."""
    logging.basicConfig(level=logging.INFO)
    try:
        if not start():
            raise RuntimeError(get_status()[1] or "Aucun scan disponible.")
        dernier_numero = 0
        while True:
            active, error = get_status()
            if error:
                raise RuntimeError(error)
            scan = get_latest_scan()
            if scan and scan.numero != dernier_numero:
                dernier_numero = scan.numero
                print(f"Scan {scan.numero} : {len(scan.mesures)} points, "
                      f"age {scan.age():.3f} s ; exemple {scan.mesures[0]}")
            elif not active:
                logger.warning("En attente de mesures recentes...")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Arret demande.")
    finally:
        stop()


if __name__ == "__main__":
    main()
