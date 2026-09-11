# vision/ - perception LiDAR (RPLIDAR A2M12)

> Etat au 11/09/2026. Ce README remplace une version qui annoncait « RPLidar A1 », « Fully
> Operational » et documentait un `lidar_navigation_bridge.py` qui n'a jamais existe dans le
> depot. En cas de doute, le code fait foi, pas ce fichier.

Le dossier porte **deux piles LiDAR independantes**, dont **une seule est dans le chemin du
match**. C'est la chose a comprendre avant de toucher quoi que ce soit ici.

| Fichier | Role | Dans le match ? |
| --- | --- | --- |
| `detection.py` | detection d'adversaire : thread, clustering, lissage, gating de vitesse | **Oui**, consomme par `app.py` |
| `localization.py` | extraction de balises + recalage de pose SVD Umeyama 2D | Non, **module gele** |
| `lidar_config.py` | liaison serie partagee par les deux (port, baudrate, timeout) | Oui, indirectement |
| `gui.py` | vue de debug tkinter/matplotlib de `localization.py` | Non, outil de mise au point |
| `__init__.py` | **volontairement vide** de tout import | - |

Les deux piles ouvrent chacune le port serie du LiDAR : elles ne peuvent pas tourner en meme
temps. Un match et la GUI de debug sur la meme machine, c'est l'un ou l'autre.

## Liaison serie

Le port n'est plus ecrit en dur. Il se resout dans cet ordre :

1. la variable d'environnement `LIDAR_PORT`,
2. la section `lidar_config` de `robot1/rasp/config.json`,
3. le defaut `/dev/ttyUSB0` (valeur Raspberry Pi).

Le port retenu est journalise a l'import. Jusqu'au 11/09/2026 `detection.py` portait
`PORT = 'COM5'` en dur : sur la Raspberry Pi le thread mourait a la connexion, l'exception etait
avalee, et le match tournait **sans aucune detection d'adversaire** sans que rien ne le signale.

```bash
python main.py                          # port de config.json
LIDAR_PORT=COM5 python -m vision        # surcharge ponctuelle (PC Windows)
```

## `detection.py` - la couche de production

```python
from robot1.rasp.vision import detection

detection.start()                          # -> bool, False si le LiDAR ne repond pas
detection.get_status()                     # -> (connecte, derniere_erreur)
detection.update_robot_pose(x, y, theta)   # pose odometrique, appelee a chaque update()
detection.get_opponent()                   # -> (x_mm, y_mm, confiance) ou None
detection.stop()
```

`start()` attend la connexion et renvoie `False` en cas d'echec : `app.py` s'en sert pour logger
une erreur explicite au demarrage plutot que de laisser croire a une detection qui fonctionne.

Principe : centroide brut du cluster le plus proche, lissage exponentiel, rejet des sauts plus
rapides que `MAX_OPP_SPEED_MM_S`. Pas de fit geometrique, donc insensible a la forme de l'objet.

**Limite connue, non corrigee** : rien ne distingue un mur, un mat de balise ou un obstacle fixe
du robot adverse. Le cluster le plus proche dans `DETECT_DIST_MM` gagne. Seuls ce rayon et la
boite englobante du terrain limitent les faux positifs. A valider en conditions de match avant
la reecriture (`doc_ref/TODO.md` section 6).

## `localization.py` - module gele

Le calcul SVD tourne dans `lidar_thread()` et est expose par `get_corrected_pose()`, mais
**`app.py::update()` ne l'applique pas** : la pose du match reste l'odometrie Teensy. Ses seuls
consommateurs sont `gui.py` et le pont Rerun (`--with-lidar`). Ni en attente ni mort : calcule,
non branche, jamais valide sur robot.

« Gele » veut dire qu'on ne le corrige plus au coup par coup : le layout des balises et la table
sont du CDR 2026, le reglement 2027 n'est pas sorti, et sa quarantaine de seuils empiriques a ete
reglee sur un terrain de test qui n'existe plus. La decision - reecrire ou supprimer - se prend
apres la campagne de logs et `replay/` (`doc_ref/TODO.md` sections 3 et 5).

Ce qui vaudra d'etre repris, **comme mathematiques et non comme code** : le SVD Umeyama
(`_compute_corrected_pose`), l'extraction de candidats (`_extract_beacon_candidates_fast`) et
l'association hongroise (`_hungarian_assign`).

Le module contient aussi sa propre detection d'adversaire (`_detect_opponent_fast`,
`get_latest_opponent`) : c'est un doublon de `detection.py`, utilise seulement par la GUI.

## Importer

`__init__.py` ne contient qu'un docstring, aucun import. C'est volontaire : `app.py` ne veut que
`detection`, et le package tirait avec lui les 1000 lignes de `localization` plus une tentative
d'import de tkinter.

```python
from robot1.rasp.vision import detection                       # match
from robot1.rasp.vision.localization import get_corrected_pose  # debug / Rerun
```

## Lancer

```bash
cd robot1/rasp
python -m vision        # GUI de debug (tkinter + matplotlib, pas sur Rasp headless)
```

## Ce qui a disparu le 11/09/2026

- `interface.py` (`LidarInterface`) : wrapper objet sans aucun consommateur de production. Son
  `get_fused_position()` implementait un melange adaptatif LiDAR/odometrie que personne
  n'appelait. Detail : `doc_ref/CHANGELOG.md`.
- La branche « fusion » de `gui.py` : elle importait `lidar_processor` et `fusion_layer`, absents
  de toutes les branches du depot. Son drapeau valait donc toujours `False` et les ~35 lignes de
  configuration qu'elle gardait etaient inatteignables.
- Les exports du `__init__.py`.
