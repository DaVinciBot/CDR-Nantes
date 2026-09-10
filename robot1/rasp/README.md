# robot1/rasp - haut niveau Python (Raspberry Pi)

Boucle de match, navigation, LiDAR, strategie et visualisation, cote Raspberry Pi 5.

> Etat : base issue de la CDR 2026. Fonctionnelle par morceaux, pas de bout en bout.
> Le haut niveau (`app.py`, `vision/`, `nav/`, `strategy/`) sera **reecrit a partir
> de la refonte** (voir `doc_ref/PLAN_REFONTE_HAUT_NIVEAU_2027.md` §10 et §13, `doc_ref/TODO.md` §5).
> Bugs connus non corriges et calibrations en attente : `doc_ref/TODO.md`,
> audit complet : `doc_ref/AUDIT_CODE_CDR_NANTES_2026.md`.

## Installation (une fois)

```bash
# depuis la racine du depot
pip install -e .                  # rend usb_com / teensy importables partout
pip install -r requirements.txt   # dependances tierces (gpiozero : Raspberry Pi seulement)
```

`pip install -e .` remplace l'ancien `loader.py` et ses `sys.path.insert`. Sans lui,
`from usb_com import Com, Messages` echoue.

## Structure

```text
robot1/rasp/
├── main.py             point d'entree du match
├── app.py              boucle de match / machine a etats (classe Robot)
├── world.py            geometrie table, balises, symetrie BLEU/JAUNE
├── config.json         identifiants Teensy (lu par comm/)
│
├── comm/               lien Teensy : mode d'execution + creation du Com
├── nav/                navigation A*
├── vision/             LiDAR : detection adversaire, recalage balises, GUI debug
├── strategy/           machine a etats strategie (stub hardcode)
├── telemetry/          pont de visualisation Rerun
│
├── tools/bringup/      scripts de bring-up materiel (liaison serie, moteurs)
├── test/               scripts de test historiques, hors chemin de prod
├── doc/                notes de travail
├── test_simple_traj.py test de deplacement interactif (garde a la racine)
├── move_robot.py       deplacement manuel x/y/theta
└── code_changement_couleur.py
```

Chaque sous-dossier est un package avec son `__init__.py` : les symboles publics
sont ceux qu'il exporte.

| Module | Import | Role |
| -------- | -------- | ------ |
| `comm` | `from comm import init_robot` | ouvre le lien Teensy, lit `ROBOT_MODE`. Point d'insertion du futur mode `sim2d`. |
| `nav` | `from nav import PathFinder` | A* sur grille 50 mm, inflation d'obstacles. |
| `vision` | `from vision import detection` | detection adversaire (la seule couche utilisee par `app.py`). |
| `vision` | `from vision import get_corrected_pose` | recalage SVD balises : calcule et expose, **pas applique** par la boucle de match. |
| `strategy` | `from strategy import StratManager, TypeAction` | sequence d'actions (stub hardcode). |
| `telemetry` | `from telemetry import rerun_bridge` | publication Rerun, best-effort. |
| `world` | `from world import Terrain, BeaconLayout` | dimensions table, obstacles fixes, balises. |

## Lancer

Toutes les commandes se lancent **depuis `robot1/rasp/`**.

```bash
cd robot1/rasp

python main.py                                  # match (robot reel)
python -m vision                                # GUI debug LiDAR
python -m telemetry.rerun_bridge --mode serve --with-lidar --port 9876
python -m tools.bringup.test_communication      # bring-up liaison serie
```

Le `python -m` n'est pas cosmetique : il place `robot1/rasp` en tete de `sys.path`,
ce qui rend `world`, `comm` et `vision` importables depuis un sous-package sans
toucher a `sys.path`.

## Mode d'execution

Un seul transport : le lien USB vers la Teensy. Le mode est explicite, jamais devine,
et se lit dans la variable `ROBOT_MODE`.

```bash
python main.py                             # robot reel (defaut)

ROBOT_MODE=dummy python main.py            # poste de dev sans Teensy (Linux / Raspberry Pi)
$env:ROBOT_MODE='dummy'; python main.py    # idem sous Windows PowerShell
```

En mode `hardware`, l'echec d'ouverture du port leve `ComError` : c'est voulu. Un robot
qui ne parle pas a sa Teensy ne doit pas sembler demarrer. Toute autre valeur de
`ROBOT_MODE` leve `ValueError`, pour qu'une faute de frappe ne degrade pas silencieusement.

Les identifiants de la Teensy (`serial_number`, `vid`, `pid`, `baudrate`) vivent dans
`config.json`, section `serial_config` : c'est la seule source de verite.

## doc/

- `TESTING_LIDAR_INTEGRATION.md` - approche de test LiDAR par niveaux (partiellement perime,
  bandeau en tete).
- `../ODOMETRY_CORRECTION_IMPLEMENTATION.md` - algo SVD Umeyama + filtre complementaire
  (le filtre decrit n'est pas branche sur la boucle de match - cf. TODO §5).

Les analyses d'etat des lieux (`CODEBASE_ANALYSIS_2026*`, `ETAT_DES_LIEUX_PYTHON`,
`PLAN_REDUCTION_RASP`) ont ete retirees le 09/09/2026 : perimees et redondantes avec le
suivi `doc_ref/` (voir `doc_ref/CHANGELOG.md`).

## Deux emplacements de scripts, deux usages

| Dossier | Usage | Lancer |
| --- | --- | --- |
| `tools/bringup/` | bring-up **materiel** : liaison serie, detection USB, un moteur. Outils qu'on lance a la main face au robot. | `python -m tools.bringup.<script>` |
| `test/` | scripts de test **du code** (CDR 2026), historiques. Pas de framework, pas d'assertions collectees. Trois sur cinq sont casses. | `python -m test.<script>` |

Detail et etat de chacun : [test/README.md](test/README.md) et
[tools/bringup/README.md](tools/bringup/README.md).

`test_simple_traj.py`, `move_robot.py` et `code_changement_couleur.py` restent a la racine :
ce sont des utilitaires de pilotage manuel, pas des tests.

> `test/` masque le package `test` de la bibliotheque standard quand le repertoire courant est
> `robot1/rasp`. Sans consequence ici (rien n'importe le `test` du stdlib), mais c'est la raison
> pour laquelle la convention Python est plutot `tests/` au pluriel.
