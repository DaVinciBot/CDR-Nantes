# robot1/rasp - haut niveau Python (Raspberry Pi)

Boucle de match, navigation, LiDAR, strategie et visualisation, cote Raspberry Pi 5.

> Etat : base issue de la CDR 2026. Fonctionnelle par morceaux, pas de bout en bout.
> Le haut niveau (`robot.py`, `lidar_*`, `pathfinder.py`, `strategy/`) sera **reecrit a partir
> de la refonte** (voir `doc_ref/PLAN_REFONTE_HAUT_NIVEAU_2027.md` §10 et `doc_ref/TODO.md` §5).
> Bugs connus non corriges et calibrations en attente : `doc_ref/TODO.md`,
> audit complet : `doc_ref/AUDIT_CODE_CDR_NANTES_2026.md`.

## Structure

```text
robot1/rasp/
├── main.py                     # point d'entree
├── robot.py                    # boucle de match / machine a etats
├── terrain_jeu.py              # geometrie table + balises
├── pathfinder.py               # navigation A*
├── loader.py                   # chargement dynamique de modules (a jeter, cf. PLAN_REFONTE §10)
├── config.json                 # config USB (serial_config = source unique des ids Teensy)
├── lidar/                      # sous-systeme LiDAR (voir lidar/README.md)
├── strategy/                   # machine a etats strategie (stub hardcode)
├── utils/                      # robot_context : mode d'execution + creation du lien Com
├── rerun/                      # pont de visualisation Rerun (voir rerun/README.md)
├── test/                       # bring-up materiel (scripts serie)
└── doc/                        # notes de travail (voir ci-dessous)
```

## Lancer

```bash
cd robot1/rasp

python -m lidar.main                                          # GUI debug LiDAR
python rerun/rerun_bridge.py --mode serve --with-lidar --port 9876   # visualisation Rerun
```

## Mode d'execution

Un seul transport : le lien USB vers la Teensy. Le mode est explicite, jamais devine,
et se lit dans la variable `ROBOT_MODE`.

```bash
cd robot1/rasp

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
