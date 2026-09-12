# tests/

Scripts de test **historiques** (CDR 2026), hors chemin de production. Ce ne sont pas des tests
automatises : aucun framework, aucune assertion collectee, ils s'executent et impriment.
Le haut niveau sera reecrit (`doc_ref/PLAN_REFONTE_HAUT_NIVEAU_2027.md` §10) : ces scripts seront
alors soit reecrits, soit supprimes. Ne pas investir dedans.

Le bring-up materiel (liaison serie, detection USB, moteurs) est ailleurs :
[../tools/bringup/](../tools/bringup/).

## Lancer

Depuis `robot1/rasp/`, au choix :

```bash
python -m tests.test_sim_mode --sim     # recommande
python tests/test_sim_mode.py --sim     # marche aussi
```

Les bootstraps `sys.path.insert` en tete de ces fichiers ont ete **retires le 11/09/2026** :
depuis l'etape B, le code du robot s'importe en `robot1.rasp.<module>` et `pip install -e .`
suffit. Le `cd robot1/rasp` ne sert plus qu'a resoudre `tests.<script>` lui-meme.

## Etat

| Script | Etat |
| --- | --- |
| `test_sim_mode.py` | **Fonctionne de bout en bout.** Simulation complete par injection de mocks (odometrie, `Robot.__init__`). Match de 30 s joue jusqu'au `stopper_tout()` final depuis le 11/09/2026. Base de depart annoncee pour `sim2d` (`doc_ref/TODO.md` §3). |
| `test_complet_lidar.py` | 7 tests par niveaux (world, pathfinder, strategie, SVD, pipeline). Les niveaux 6 et 8 exigent le LiDAR branche. L'ancien niveau 7 (`LidarInterface`) est parti avec le wrapper, le 8 garde son numero. |

Les trois scripts casses ont ete **supprimes le 11/09/2026** plutot que reecrits : ils testaient
`Robot._apply_complementary_filter` et `GestionnaireLidar`, disparus tous les deux, et le haut
niveau sera de toute facon reecrit. Detail dans `doc_ref/CHANGELOG.md`.

- `test_program.py` - `NameError` sur `GestionnaireLidar`.
- `test_lidar_correction_integration.py` - reference `Robot._apply_complementary_filter`.
- `test_complementary_filter.py` - meme cause.

## Limites connues

Tous ces scripts importent `app.py`, donc `gpiozero`, donc ils ne tournent pas hors Raspberry Pi
sans stub. Pour `test_sim_mode.py` c'est un vrai probleme, puisqu'il est cense servir de base a la
simulation : a corriger dans `sim2d` en passant les GPIO derriere une abstraction (`TODO.md` §3).

L'`AttributeError: '_pin_tirette'` qui cassait la toute fin du match simule est **corrige**
(11/09/2026) : `patched_init` cree les deux pins a `None` et `app.py::stopper_tout()` ferme la
tirette sous condition.

Reste une limite de fond : `test_sim_mode.py` mocke le LiDAR dans `self.lidar`, alors qu'`app.py`
utilise le module importe (`from robot1.rasp.vision import detection as lidar`). **Le mock
n'intercepte donc rien**, c'est le vrai `detection` qui est appele. Sans LiDAR branche il ne
renvoie jamais d'adversaire : la simulation tourne, mais elle ne teste pas l'evitement.
A traiter dans `sim2d`.
