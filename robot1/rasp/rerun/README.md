# Pont Rerun - visualisation temps reel

`rerun_bridge.py` publie l'etat du robot (odometrie, nuage LiDAR, balises, pose corrigee,
cible, trajectoire, position fusionnee) vers le Rerun SDK a 20 Hz, pour monitoring et
rejeu avec timeline.

Rerun ne fait **aucun calcul de fusion** : il recoit des donnees deja calculees par le
haut niveau (`robot.py` / pathfinder) et les affiche.

> Etat : pont fonctionnel sur `main`, non branche sur la boucle de match, jamais teste sur
> robot. Sort a trancher (TODO §7 : stress test CPU Rasp 5). Le bas + haut niveau seront
> reecrits a partir de novembre 2026 ; `rerun_bridge.py` sera repris elague (PLAN_REFONTE §10).

---

## Lancer

```bash
cd robot1/rasp

# Viewer local (dev, machine avec affichage), donnees fictives
python rerun/rerun_bridge.py --mode local --sim

# Stream gRPC vers un viewer distant (Rasp -> PC)
python rerun/rerun_bridge.py --mode serve --port 9876 --with-lidar

# Se connecter a un viewer Rerun deja lance ailleurs
python rerun/rerun_bridge.py --mode connect --host <IP_PC> --port 9876
```

Arguments (`argparse` dans `rerun_bridge.py`) :

| Flag | Defaut | Role |
|------|--------|------|
| `--mode` | `local` | `local` = viewer spawne (`rr.spawn`) · `serve` = `rr.serve_grpc` · `connect` = `connect_grpc` vers viewer externe |
| `--host` | `0.0.0.0` | hote pour `serve` / `connect` |
| `--port` | `9876` | port gRPC |
| `--sim` | off | simulation interne (robot en cercle, nuage synthetique) |
| `--with-lidar` | off | polling du LiDAR hardware via `lidar/lidar_logic.py` |

En mode `serve`, on **attache le viewer Rerun** au flux gRPC (viewer natif ou
`rerun --connect rerun+http://<IP_RASP>:9876/proxy`), ce n'est pas une page web a ouvrir
dans un navigateur.

---

## API publique

Appelee depuis `robot.py::update()` / pathfinder / strategie. Toutes thread-safe
(`_State` + `RLock`).

```python
update_odom(x, y, theta)                     # odometrie Teensy brute       -> cylindre bleu
update_lidar_cloud(pts)                       # [[angle_rad, dist_mm, qual]] -> nuage 3D + vue polaire
update_lidar_beacons(cands)                   # [{"x_r", "y_r", "id"}]       -> diamants oranges
update_lidar_pose(x, y, theta, conf, ok)      # pose SVD balises             -> cylindre rouge (si ok)
update_target(x, y, theta=None)              # cible navigation            -> point jaune
update_trajectory(pts)                        # [(x, y), ...] (vide = clear) -> ligne pointillee
update_obstacles(obstacles)                   # [{"x", "y", "radius"}]       -> spheres rouges
update_fused(x, y, theta)                     # meilleure estimation        -> cylindre vert
```

Toutes les positions en mm, angles en radians, repere table `(0,0)` en bas-gauche,
`(3000, 2000)` en haut-droite.

---

## Fichiers

| Fichier | Role |
|---------|------|
| `rerun_bridge.py` | pont principal (etat partage, API, boucle de publication, modes reseau) |
| `table_map_exporter.py` | publie le terrain statique (playmat, balises, zones, murs, caisses), geometrie codee en dur, aucune dependance simulateur |
| `map_assets/eurobot2026/` | textures + `.proto` de la table |
| `RERUN_QUICK_START.md` | prise en main |
| `RERUN_DATA_FLOW_INTEGRATION.md` | flux de donnees, integration `robot.py`, API detaillee |
| `RERUN_ARCHITECTURE_TECHNICAL.md` | architecture, threads, perf |
| `RERUN_INTEGRATION_EXAMPLES.md` | exemples d'integration copier-coller |

> Les 4 `RERUN_*.md` sont anterieurs au passage en `serve_grpc` : la ou ils disent
> "ouvrir http://RaspIP:9876 dans un navigateur", voir la section "Lancer" ci-dessus.
> Ils seront consolides a la refonte.
