# test/

Scripts de test **historiques** (CDR 2026), hors chemin de production. Ce ne sont pas des tests
automatises : aucun framework, aucune assertion collectee, ils s'executent et impriment.
Le haut niveau sera reecrit (`doc_ref/PLAN_REFONTE_HAUT_NIVEAU_2027.md` §10) : ces scripts seront
alors soit reecrits, soit supprimes. Ne pas investir dedans.

Le bring-up materiel (liaison serie, detection USB, moteurs) est ailleurs :
[../tools/bringup/](../tools/bringup/).

## Lancer

Depuis `robot1/rasp/`, au choix :

```bash
python -m test.test_sim_mode --sim      # recommande
python test/test_sim_mode.py --sim      # marche aussi (bootstrap sys.path en tete de fichier)
```

## Etat

| Script | Etat |
| --- | --- |
| `test_sim_mode.py` | **Fonctionne.** Simulation complete par injection de mocks (odometrie, `Robot.__init__`). Base de depart annoncee pour `sim2d` (`doc_ref/TODO.md` §3). |
| `test_complet_lidar.py` | 8 tests par niveaux (world, pathfinder, strategie, SVD, pipeline). Les niveaux LiDAR exigent le materiel. |
| `test_program.py` | **Casse** : `GestionnaireLidar` non defini L109 alors que l'import L15 est `LidarInterface`. `NameError` garanti (`TODO.md` §2). |
| `test_lidar_correction_integration.py` | **Casse** : reference `Robot._apply_complementary_filter`, qui n'existe plus (`TODO.md` §5). |
| `test_complementary_filter.py` | **Casse** : meme cause. |

## Limite connue

Tous ces scripts importent `app.py`, donc `gpiozero`, donc ils ne tournent pas hors Raspberry Pi
sans stub. Pour `test_sim_mode.py` c'est un vrai probleme, puisqu'il est censé servir de base a la
simulation : a corriger dans `sim2d` en passant les GPIO derriere une abstraction (`TODO.md` §3).
