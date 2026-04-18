# Etat des lieux Python - CDR Nantes

Date: 2026-04-18
Perimetre: partie Python du workspace CDR-Nantes
Objectif: photographie complete de l etat actuel (imports, versions, coherence structure/doc), sans proposition de code executable.

Note: ce rapport correspond a la photo initiale de la journee. Une premiere vague de cleanup a ensuite archive certains tests lidar et documents vers `robot1/archive_rasp/`.

## 1) Resume executif

Etat global: PARTIELLEMENT FONCTIONNEL.

Points bloquants identifies:
- Imports internes casses (noms de modules/fichiers non alignes).
- Environnement Python actif incomplet (venv avec seulement pip).
- Deux erreurs de syntaxe/indentation qui empechent une execution fiable de tout le perimetre.
- Documentation dispersee et partiellement obsolete par rapport au code reel.

Points positifs:
- Architecture Python globalement claire par domaines (communication, lidar, rerun, tests, utils, strategy).
- requirements.txt unique a la racine.
- Module lidar moderne present (lidar_logic + lidar_gui + bridge navigation).

## 2) Inventaire Python (photo actuelle)

Total fichiers .py: 58

Repartition:
- common/usb_com/python: 8
- common/teensy: 7
- robot1/rasp/lidar: 6
- robot1/rasp/rerun: 3
- robot1/rasp/simu: 8
- robot1/rasp/test: 6
- robot1/rasp/utils: 3
- autres sous robot1/rasp: 16

Documentation .md detectee: 22 fichiers.

## 3) Environnements Python et versions

requirements.txt (attendu projet):
- pyserial==3.5
- crc8==0.1.0
- loggerplusplus==1.7.0
- pyusb==1.2.1
- numpy>=1.26
- matplotlib>=3.8
- rplidar-roboticia==0.9.5
- rerun-sdk>=0.31.0
- pygltflib>=0.7.0

Environnement actif dans l IDE (Pylance):
- Interpreter: .venv/Scripts/python.exe
- Version: Python 3.14.3
- Packages installes: pip==26.0.1 uniquement

Environnement systeme detecte en parallele:
- Interpreter: pythoncore-3.14-64/python.exe
- Version: Python 3.14.3
- Packages confirms: pyserial 3.5, numpy 2.4.3, matplotlib 3.10.8, rplidar-roboticia 0.9.5
- Packages manquants: crc8, loggerplusplus, pyusb, rerun-sdk, pygltflib

Conclusion versions/environnement:
- Il existe un decalage entre l environnement IDE (venv vide) et l environnement systeme (partiellement equipe).
- Cela explique une partie majeure des imports non resolus dans l editeur.

## 4) Probleme d imports - detail

### 4.1 Imports internes casses (structure projet)

Blocants:
- robot1/rasp/main.py importe core.robot, mais aucun package core n existe dans le workspace.
- robot1/rasp/robot.py importe strategy.actions et strategy.strat_manager.
- Les fichiers reels sont robot1/rasp/strategy/strategy_actions.py et robot1/rasp/strategy/strategy_strat_manager.py (noms non alignes avec les imports).
- robot1/rasp/strategy/strategy_strat_manager.py importe .actions, fichier absent (nom reel: strategy_actions.py).
- robot1/rasp/robot.py et robot1/rasp/test_program.py importent GestionnaireLidar depuis lidar, mais aucune definition de classe GestionnaireLidar n a ete trouvee.
- robot1/rasp/lidar/lidar_gui.py tente d importer lidar_processor et fusion_layer, fichiers absents du workspace.

### 4.2 Imports dependants du contexte de lancement (fragiles)

Observations:
- Plusieurs scripts de test injectent des chemins dynamiquement via sys.path.insert.
- Exemple typique: imports directs rerun_bridge et lidar_logic dans test_lidar_bridge.py.
- Ces imports peuvent marcher en execution locale precise, mais restent non resolus en analyse statique IDE.

Impact:
- Forte sensibilite au repertoire courant.
- Comportements differents entre execution manuelle, IDE, CI, et lancement module.

### 4.3 Imports externes manquants dans le venv actif

Dans le venv actif de l IDE, modules non disponibles:
- serial
- numpy
- matplotlib
- rplidar
- rerun
- PIL
- crc8
- loggerplusplus

Impact:
- Les diagnostics IDE sont majoritairement rouges meme sur des modules potentiellement valides en environnement systeme.

## 5) Erreurs de syntaxe/indentation (compileall)

Verification compileall sur tout le repo:
- robot1/rasp/pathfinder.py: IndentationError (unexpected indent) ligne 70.
- robot1/rasp/rerun/webots_map_exporter.py: SyntaxError (unterminated string literal) ligne 289.

Impact:
- Ces fichiers ne peuvent pas etre importes/executables proprement tant que ces erreurs persistent.

## 6) Incoherences de code supplementaires

Detecte via diagnostics:
- robot1/rasp/strategy/strategy_strat_manager.py utilise time.time() sans import time.

Impact:
- Meme si les imports de module etaient corriges, execution runtime en erreur probable sur la gestion chrono.

## 7) Etat des "versions" fonctionnelles internes

### 7.1 Lidar

Version active dans le code:
- robot1/rasp/lidar/lidar_logic.py
- robot1/rasp/lidar/lidar_gui.py
- robot1/rasp/lidar/lidar_navigation_bridge.py
- robot1/rasp/lidar/main.py

Version documentee mais obsolete dans certaines docs:
- references a test_lidar.py et lidar_runtime.py encore presentes dans la documentation lidar.

Conclusion:
- Le code semble avoir migre vers une version modulaire, mais une partie de la doc est encore sur une ancienne version de nommage.

### 7.2 Strategy

Version code actuelle:
- dossier robot1/rasp/strategy avec noms prefixes (strategy_actions.py, strategy_strat_manager.py)

Version attendue par imports:
- structure de type strategy/actions.py et strategy/strat_manager.py

Conclusion:
- migration de nommage inachevee entre fichiers et imports.

### 7.3 Communication COM

Version API reel (common/usb_com/python/com/com.py):
- constructeur Com axe sur logger, serial_number, vid, pid, baudrate, enable_crc, enable_dummy
- pas de start_listening expose

Version API presente dans une partie de la doc:
- exemples avec parametre port, use_crc et start_listening

Conclusion:
- documentation de l API Com pas synchronisee avec l implementation actuelle.

## 8) Etat de la documentation Python

Constat general:
- Documentation abondante mais dispersee (22 fichiers .md), avec plusieurs zones non synchronisees au code.

Incoherences majeures relevees:
- robot1/rasp/README.md et robot1/rasp/utils/README.md referencent des fichiers absents ou deplaces (robot_context.py, webots_com.py au mauvais niveau, lidar_manager.py, test_lidar_simple.py, etc.).
- robot1/rasp/lidar/ETAT_DU_CODE.md documente une generation ancienne basee sur test_lidar.py / lidar_runtime.py.
- simulation/README.md decrit des fichiers Python a la racine simulation qui n existent pas dans cette arborescence.
- common/usb_com/python/README.md contient des exemples d API qui ne correspondent pas a la signature actuelle de Com.

Conclusion doc:
- Necessite une source unique de verite pour la partie Python. Ce document joue ce role de reference a date.

## 9) Niveau de risque (exploitation)

Risque eleve:
- Imports internes casses (main/robot/strategy/lidar fusion).
- Environnement IDE non provisionne.
- Erreurs syntaxe compileall.

Risque moyen:
- Scripts relies a sys.path insert (dependants du contexte de lancement).
- Documentation obsolescente conduisant a des faux pas de lancement.

Risque faible:
- Organisation des dossiers globalement lisible et coherent par domaine.

## 10) Priorites de remediation (sans code ici)

Priorite 1:
- Stabiliser la couche structure/import interne (main, robot, strategy, lidar).

Priorite 2:
- Unifier l environnement Python de reference (venv projet) et l aligner avec requirements.txt.

Priorite 3:
- Corriger les erreurs de syntaxe/indentation compileall.

Priorite 4:
- Harmoniser la documentation autour d un seul referentiel Python puis mettre a jour les READMEs obsoletes.

---

Source de verite creee: robot1/rasp/doc/ETAT_DES_LIEUX_PYTHON_2026-04-18.md
