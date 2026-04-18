# Plan de reduction rasp (Python + docs)

Date: 2026-04-18

## Objectif

Reduire la surface active sous robot1/rasp pour garder les fichiers utiles au developpement courant, tout en conservant l historique dans une archive.

## Vague 1 appliquee

Fichiers Python testes lidar archives:
- robot1/rasp/test_lidar_bridge.py
- robot1/rasp/rerun/test_teensy_lidar_simple.py

Documents archives:
- robot1/rasp/lidar/ETAT_DU_CODE.md
- robot1/rasp/LIDAR_INTEGRATION_GUIDE.md
- robot1/rasp/rerun/TEST_TEENSY_LIDAR_SIMPLE_README.md

Destination archive:
- robot1/archive_rasp/tests_lidar/
- robot1/archive_rasp/docs_obsoletes/
- robot1/archive_rasp/rerun_docs/

## Vague 2 appliquee

Fichiers Python supplementaires retires de la surface active:
- robot1/rasp/test_lidar_only.py (script non present dans le workspace actuel)
- robot1/rasp/test_live_hardware.py (script non present dans le workspace actuel)
- robot1/rasp/rerun_init.py (archive dans robot1/archive_rasp/rerun_tools/)

Destination archive utilisee:
- robot1/archive_rasp/rerun_tools/

## Surface active cible (lidar)

Flux a garder en priorite:
- python -m lidar.main
- python rerun/rerun_bridge.py --mode serve --with-lidar --port 9876

Module coeur a garder:
- robot1/rasp/lidar/lidar_logic.py
- robot1/rasp/lidar/lidar_gui.py
- robot1/rasp/lidar/lidar_navigation_bridge.py
- robot1/rasp/lidar/main.py
- robot1/rasp/rerun/rerun_bridge.py
- robot1/rasp/lidar/launch_rerun.py

## Vague 3 proposee (non appliquee ici)

- Fusionner la doc rerun en un seul guide principal.
- Archiver les docs rerun secondaires une fois references migrees.
- Reduire les tests simulation redondants en conservant un test unifie + un test bas niveau.

## Regle de cleanup

- Deplacer d abord en archive (pas de suppression directe).
- Garder les scripts utilises en execution quotidienne dans robot1/rasp.
- Mettre a jour les README avant toute vague suivante.
