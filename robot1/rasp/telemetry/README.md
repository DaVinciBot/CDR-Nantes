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
python -m telemetry.rerun_bridge --mode local --sim

# Stream gRPC vers un viewer distant (Rasp -> PC)
python -m telemetry.rerun_bridge --mode serve --port 9876 --with-lidar

# Se connecter a un viewer Rerun deja lance ailleurs
python -m telemetry.rerun_bridge --mode connect --host <IP_PC> --port 9876
```

Arguments (`argparse` dans `rerun_bridge.py`) :

| Flag | Defaut | Role |
|------|--------|------|
| `--mode` | `local` | `local` = viewer spawne (`rr.spawn`) · `serve` = `rr.serve_grpc` · `connect` = `connect_grpc` vers viewer externe |
| `--host` | `0.0.0.0` | hote pour `serve` / `connect` |
| `--port` | `9876` | port gRPC |
| `--sim` | off | simulation interne (robot en cercle, nuage synthetique) |
| `--with-lidar` | off | demarre le thread LiDAR de `vision/localization.py` (module **gele**) et publie ses scans, balises et pose SVD |

`--with-lidar` ne coupe plus la Teensy : depuis le 11/09/2026 les deux sources
coexistent, et le pont montre odometrie et recalage cote a cote. Jusque-la
l'option se contentait d'interroger un etat vide, le thread LiDAR n'etant jamais
demarre (`doc_ref/CHANGELOG.md`).

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
| `table_map.py` | publie le terrain statique (playmat, balises, zones, murs, caisses), geometrie codee en dur, aucune dependance simulateur |
| `map_assets/eurobot2026/textures/playmat_2026.jpg` | tapis de la table, seule ressource encore utilisee (par `table_map.py` et `rerun_bridge.py`) |

> **Les 4 `RERUN_*.md` (96 Ko) ont ete supprimes le 11/09/2026** : anterieurs au passage en
> `serve_grpc`, ils decrivaient encore `robot.py` (devenu `app.py`) et une ouverture du pont dans
> un navigateur. Le pont sera elague a la refonte (`PLAN_REFONTE` §13) ; ce README est desormais
> la seule doc du module. Contenu recuperable dans `git log`.
>
> Supprime en meme temps : les 11 `.proto` **Webots** de `map_assets/eurobot2026/table/` et les
> textures `playmat_grid.jpg` / `attic.jpg` / `crate.png`, que rien ne referencait (3,1 Mo).
> `table_map.py` cite toujours les `.proto` en commentaire : c'est la provenance des cotes qu'il
> porte en dur, pas un fichier qu'il charge.
