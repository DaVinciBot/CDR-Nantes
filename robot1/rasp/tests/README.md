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
