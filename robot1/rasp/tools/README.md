# tools/

Outillage, **hors chemin de production**. Rien ici n'est importe par `main.py` /
`app.py` : ce sont des scripts qu'on lance a la main.

```text
tools/
├── bringup/        bring-up materiel : liaison serie, detection USB, moteurs
├── manual/         pilotage manuel du robot complet : consignes x/y/theta
└── run_test.py     lance un essai enregistre et fait saisir sa verite terrain
```

Aucun de ces dossiers n'a d'`__init__.py`, volontairement : ce ne sont pas des packages
du robot, juste des namespaces pour pouvoir les lancer avec `-m` depuis `robot1/rasp/`.

```bash
cd robot1/rasp
python -m tools.bringup.test_usb_detection
python -m tools.bringup.test_communication
python -m tools.manual.move_robot
```

Le `-m` est requis : c'est ce qui met `robot1/rasp` dans `sys.path`, donc ce qui rend
`tools.bringup.<script>` resolvable. Le code du robot, lui, s'importe en
`from robot1.rasp.comm import get_com_config` : ces scripts ne
dependent plus du repertoire courant pour ca, uniquement pour se trouver eux-memes.

Voir [bringup/README.md](bringup/README.md) pour le detail de chaque script de bring-up.

## run_test.py

Lance **un essai** : un run enregistré pour calibrer ou valider, avec son type, sa note et sa
vérité terrain (ce qui s'est vraiment passé, mesuré sur la table).

```bash
cd robot1/rasp
python -m tools.run_test --list                              # les types d'essai
python -m tools.run_test --type check --duration 10 --yes    # contrôle avant une séance
python -m tools.run_test --type otos_lin --note "1 m le long de x, 200 mm/s" \
    --start-pose "250 1000 0" --target "1250 1000 0"
python -m tools.run_test --type otos_ang --note "10 tours à gauche, lent"
```

Déroulé : le lien est ouvert, tout est enregistré (le log est toujours actif ici, quel que soit
`config.json`, et les deux messages par tick de la Teensy sont demandés), l'opérateur démarre,
le robot est bougé à la main ou par les cibles `--target`, `STOP` est envoyé, la vérité terrain
est demandée, puis le dossier est relu et l'outil rend son verdict.

L'outil est strict là où un match ne l'est pas :

- un essai de calibration **ne démarre pas sans `LOG_TICK`** (`--allow-partial` pour forcer, le
  dossier le dit) ;
- l'essai **n'est pas clos sans vérité terrain** : une pose finale (`x y theta`, en mm et en
  **degrés**, comme on la lit sur la table), un nombre de tours, ou `aucune` avec sa raison ;
- le verdict `ESSAI : OK` demande un log complet, la vérité du bon type, et des `LOG_TICK`
  réellement reçus. Sinon l'outil dit ce qui manque et sort en code retour 1.

Quand la vérité est une pose, l'écart entre la pose estimée par la Teensy et la pose mesurée est
affiché tout de suite.

| Option | Rôle |
| --- | --- |
| `--type T` | type d'essai (`--list`) : il fixe la vérité terrain attendue |
| `--note`, `--operator` | écrits dans `meta.json` |
| `--start-pose "X Y DEG"` | pose imposée à la Teensy avant l'essai |
| `--target "X Y DEG"` | cible de pose, répétable ; sans cible, le robot est bougé à la main |
| `--target-timeout S` | attente maximale par cible (20 s) |
| `--duration S` | durée d'enregistrement après les cibles ; sans elle, Entrée arrête l'essai |
| `--yes` | démarrer sans attendre Entrée |
| `--truth "X Y DEG"`, `--turns N`, `--no-truth RAISON` | vérité terrain donnée d'avance, sans question |
| `--uncertainty-mm`, `--uncertainty-deg` | précision de la mesure (2 mm, 0,5°) |
| `--no-log-tick`, `--no-pid-trace` | ne pas demander ces messages à la Teensy |
| `--allow-partial` | lancer un essai de calibration sans `LOG_TICK` |
| `--log-dir D` | dossier des runs (défaut : `log_config` de `config.json`) |

Il suit `ROBOT_MODE` : en `dummy` il sert à répéter une séance sans robot. Types d'essai, format
de `truth.json` et verdict : `telemetry/trial.py`. Format du dossier :
[../telemetry/README.md](../telemetry/README.md).

## manual/

- `move_robot.py` - deplacement manuel x/y/theta, interactif ou par liste de points.
  Descendu de la racine de `rasp/` le 11/09/2026 : c'est de l'outillage, pas du chemin
  de production. `test_simple_traj.py` est reste a la racine pour l'instant.
