# Robot1 Raspberry Pi - Python

Ce dossier contient la partie Python active du robot.

## Structure utile

```text
robot1/rasp/
├── main.py
├── robot.py
├── terrain_jeu.py
├── pathfinder.py
├── loader.py
├── switch_mode.py
├── config.json
├── lidar/
├── strategy/
├── utils/
├── rerun/
├── test/
├── simu/
└── doc/
```

## Flux lidar actif (debug)

```bash
cd robot1/rasp

# Interface lidar (lidar_logic + lidar_gui)
python -m lidar.main

# Visualisation Rerun avec polling lidar
python rerun/rerun_bridge.py --mode serve --with-lidar --port 9876
```

Les scripts de test lidar historiques ont ete archives.

## Lancement module lidar GUI

```bash
cd robot1/rasp
python -m lidar.main
```

## Bascule simulation/hardware

```bash
cd robot1/rasp
python switch_mode.py simulation
python switch_mode.py hardware
```

## Documentation de reference

- Etat global Python: `doc/ETAT_DES_LIEUX_PYTHON_2026-04-18.md`
- Plan de reduction: `doc/PLAN_REDUCTION_RASP.md`

## Archives (cleanup)

Les fichiers retires de la surface active sont conserves ici:

- `robot1/archive_rasp/tests_lidar/`
- `robot1/archive_rasp/docs_obsoletes/`
- `robot1/archive_rasp/rerun_docs/`
- `robot1/archive_rasp/rerun_tools/`
