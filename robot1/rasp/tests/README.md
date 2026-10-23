# tests/

Scripts de test **historiques** (CDR 2026), hors chemin de production. Ce ne sont pas des tests
automatises : aucun framework, aucune assertion collectee, ils s'executent et impriment.
Le haut niveau sera reecrit : ces scripts seront alors soit reecrits, soit supprimes. Ne pas
investir dedans.

Le bring-up materiel (liaison serie, detection USB, moteurs) est ailleurs :
[../tools/bringup/](../tools/bringup/).

## Lancer

Depuis `robot1/rasp/` :

```bash
python -m tests.test_complet_lidar --no-lidar   # niveaux sans LiDAR branche
python -m tests.test_complet_lidar              # tous les niveaux, LiDAR branche
```

Les bootstraps `sys.path.insert` en tete de ces fichiers ont ete **retires le 11/09/2026** :
le code du robot s'importe en `robot1.rasp.<module>` et `pip install -e .` suffit. Le
`cd robot1/rasp` ne sert plus qu'a resoudre `tests.<script>` lui-meme.

## Etat

| Script | Etat |
| --- | --- |
| `test_complet_lidar.py` | 7 tests par niveaux (world, pathfinder, strategie, SVD, pipeline). Les niveaux 6 et 8 exigent le LiDAR branche. L'ancien niveau 7 (`LidarInterface`) est parti avec le wrapper, le 8 garde son numero. |

Les trois scripts casses ont ete **supprimes le 11/09/2026** plutot que reecrits : ils testaient
`Robot._apply_complementary_filter` et `GestionnaireLidar`, disparus tous les deux, et le haut
niveau sera de toute facon reecrit.

- `test_program.py` - `NameError` sur `GestionnaireLidar`.
- `test_lidar_correction_integration.py` - reference `Robot._apply_complementary_filter`.
- `test_complementary_filter.py` - meme cause.

`test_sim_mode.py` (simulation d'un match par injection de mocks) a ete **supprime** : il est
remplace par `robot1/sim2d/`, qui fait tourner la boucle de match sans mock. Voir
[../../sim2d/README.md](../../sim2d/README.md).
