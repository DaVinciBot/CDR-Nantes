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

`pip install -e .` remplace l'ancien `loader.py` et ses `sys.path.insert`. Il rend importables,
depuis n'importe quel repertoire courant, `usb_com` / `teensy` (depuis `common/`) **et
`robot1.rasp`** (depuis la racine du depot). Sans lui, `from usb_com import Com, Messages` et
`from robot1.rasp.nav import PathFinder` echouent tous les deux.

## Structure

```text
robot1/rasp/
├── __init__.py         vide : fait de `rasp` le package `robot1.rasp`
├── main.py             point d'entree du match
├── app.py              boucle de match / machine a etats (classe Robot)
├── world.py            geometrie table, balises, symetrie BLEU/JAUNE
├── config.json         identifiants Teensy + port LiDAR (comm/, vision/)
│
├── comm/               lien Teensy : mode d'execution + creation du Com
├── nav/                navigation A*
├── vision/             LiDAR : detection adversaire (production) + recalage balises (gele)
├── strategy/           machine a etats strategie (stub hardcode)
├── telemetry/          pont de visualisation Rerun
│
│
├── tools/bringup/      bring-up materiel (liaison serie, moteurs)   HORS package
├── tools/manual/       pilotage manuel du robot                     HORS package
└── tests/              scripts de test historiques                  HORS package
```

Chaque module du haut (`comm`, `nav`, `strategy`, `telemetry`) est un package avec son
`__init__.py` : les symboles publics sont ceux qu'il exporte. `vision/` fait exception depuis le
11/09/2026 : son `__init__.py` n'exporte **rien**, pour qu'importer la detection d'adversaire ne
tire pas avec elle les 1000 lignes du recalage balises et une tentative d'import de tkinter.
On y importe le sous-module voulu, jamais le package.

`tools/` et `tests/` n'ont **volontairement pas** d'`__init__.py` : ce ne sont pas des
dependances du robot, juste des namespaces de lancement. Consequence pratique, ils se lancent en
`-m` **depuis `robot1/rasp/`**, alors que `robot1.rasp.*` s'importe de partout.

| Module | Import | Role |
| -------- | -------- | ------ |
| `comm` | `from robot1.rasp.comm import init_robot` | ouvre le lien Teensy, lit `ROBOT_MODE`. Point d'insertion du futur mode `sim2d`. |
| `nav` | `from robot1.rasp.nav import PathFinder` | A* sur grille 50 mm, inflation d'obstacles. |
| `vision` | `from robot1.rasp.vision import detection` | detection adversaire (la seule couche utilisee par `app.py`). |
| `vision` | `from robot1.rasp.vision.localization import get_corrected_pose` | recalage SVD balises : calcule et expose, **pas applique** par la boucle de match. Module **gele**, voir [vision/README.md](vision/README.md). |
| `strategy` | `from robot1.rasp.strategy import StratManager, TypeAction` | sequence d'actions (stub hardcode). |
| `telemetry` | `from robot1.rasp.telemetry import rerun_bridge` | publication Rerun, best-effort. |
| `world` | `from robot1.rasp.world import Terrain, BeaconLayout` | dimensions table, obstacles fixes, balises. |

## Lancer

Toutes les commandes se lancent **depuis `robot1/rasp/`**.

```bash
cd robot1/rasp

python main.py                                  # match (robot reel)
python -m vision                                # GUI debug LiDAR
python -m telemetry.rerun_bridge --mode serve --with-lidar --port 9876
python -m tools.bringup.test_communication      # bring-up liaison serie
python -m tools.manual.move_robot               # pilotage manuel x/y/theta
```

Le `-m` sert ici a resoudre `vision`, `telemetry`, `tools` et `tests` **en tant que chemins**,
via le repertoire courant : d'ou l'obligation d'etre dans `robot1/rasp/`. Le code du robot,
lui, ne depend plus du repertoire courant depuis l'etape B (11/09/2026) — il s'importe en
`robot1.rasp.<module>` et fonctionne de partout :

```bash
cd n-importe-ou
python -m robot1.rasp.main                      # le match, sans se soucier du cwd
python -c "from robot1.rasp.nav import PathFinder"
```

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

Le port du LiDAR suit la meme regle, section `lidar_config`, avec une surcharge ponctuelle par
`LIDAR_PORT` :

```bash
LIDAR_PORT=COM5 python -m vision           # PC Windows, sans toucher config.json
```

Il etait ecrit en dur dans `vision/detection.py` (`COM5`) jusqu'au 11/09/2026, ce qui privait le
match de toute detection d'adversaire sur la Raspberry Pi, sans message d'erreur autre qu'une
ligne noyee dans le log. Detail : [vision/README.md](vision/README.md).

## Ou est passee la doc de `rasp/`

Il n'y a plus de dossier `doc/` : ce README et les README de chaque module sont la seule doc du
code. Le reste vit dans `doc_ref/` (non commite) et `DocVB/cdr/nantes/Info/` (publie).

- `doc/TESTING_LIDAR_INTEGRATION.md` et `ODOMETRY_CORRECTION_IMPLEMENTATION.md` : **supprimes**
  (09/09 et 11/09/2026). Ce qui valait d'etre garde - seuils d'acceptation du LiDAR, courbe alpha
  du filtre complementaire adaptatif - a ete extrait dans `doc_ref/PLAN_REFONTE` §7 avant
  suppression. Etat du SVD : `doc_ref/TODO.md` §5.
- Les 4 `telemetry/RERUN_*.md` (96 Ko) : **supprimes** le 11/09/2026, voir
  [telemetry/README.md](telemetry/README.md).
- Les analyses d'etat des lieux (`CODEBASE_ANALYSIS_2026*`, `ETAT_DES_LIEUX_PYTHON`,
  `PLAN_REDUCTION_RASP`) : retirees le 09/09/2026, perimees et redondantes avec `doc_ref/`.

Detail de chaque suppression et de ce qui a ete extrait : `doc_ref/CHANGELOG.md`.

## Trois emplacements de scripts, trois usages

| Dossier | Usage | Lancer |
| --- | --- | --- |
| `tools/bringup/` | bring-up **materiel** : liaison serie, detection USB, un moteur. Outils qu'on lance a la main face au robot. | `python -m tools.bringup.<script>` |
| `tools/manual/` | **pilotage manuel** du robot complet : envoi de consignes x/y/theta. | `python -m tools.manual.<script>` |
| `tests/` | scripts de test **du code** (CDR 2026), historiques. Pas de framework, pas d'assertions collectees. Il en reste deux : les trois casses ont ete supprimes le 11/09/2026. | `python -m tests.<script>` |

Detail et etat de chacun : [tests/README.md](tests/README.md) et
[tools/bringup/README.md](tools/bringup/README.md).

La racine de `rasp/` ne porte plus que du chemin de production. `move_robot.py` et
`test_simple_traj.py` sont descendus dans `tools/manual/` le 11/09/2026.

`code_changement_couleur.py` y est aussi, mais **mis de cote**, pas comme un outil : il double le
mecanisme de couleur d'equipe deja en production (`app.py::lire_couleur_equipe` -> `Robot` ->
`Terrain` / `StratManager`, symetrie dans `world.py`), avec une symetrie **contradictoire** - Y
(`y = 2000 - y`) la ou `world.py` fait X (`x = FIELD_WIDTH_MM - x`). Ne pas le rebrancher sans
avoir tranche lequel des deux axes est le bon : la reponse ira dans `world.py`, pas ici
(`doc_ref/TODO.md` §2).

> Le dossier s'appelait `test/` jusqu'au 10/09/2026 : il masquait alors le package `test` de la
> bibliotheque standard quand le repertoire courant est `robot1/rasp`. Renomme `tests/`, au
> pluriel, conformement a la convention Python.
