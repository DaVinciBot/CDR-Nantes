# Pont Rerun - visualisation temps reel

`rerun_bridge.py` publie l'etat du robot (odometrie, nuage LiDAR, balises, pose corrigee,
cible, trajectoire, position fusionnee) vers le Rerun SDK a 20 Hz, pour monitoring et
rejeu avec timeline.

Rerun ne fait **aucun calcul de fusion** : il recoit des donnees deja calculees par le
haut niveau (`robot.py` / pathfinder) et les affiche.

> Etat : pont fonctionnel sur `main`, non branche sur la boucle de match, jamais teste sur
> robot. Le garder ou non en match depend d'un stress test CPU sur la Raspberry Pi 5, pas
> encore fait. Le bas + haut niveau seront reecrits a partir de novembre 2026 ;
> `rerun_bridge.py` sera repris elague.

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
demarre.

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
| `table_map.py` | publie le terrain statique (playmat, balises, zones, murs), geometrie codee en dur, aucune dependance simulateur |
| `map_assets/playmat2027/Field.png` | tapis de la table, seule ressource encore utilisee (par `table_map.py` et `rerun_bridge.py`) |

> **Les 4 `RERUN_*.md` (96 Ko) ont ete supprimes le 11/09/2026** : anterieurs au passage en
> `serve_grpc`, ils decrivaient encore `robot.py` (devenu `app.py`) et une ouverture du pont dans
> un navigateur. Le pont sera elague a la refonte ; ce README est desormais
> la seule doc du module. Contenu recuperable dans `git log`.
>
> Supprime en meme temps : les 11 `.proto` **Webots** de `map_assets/eurobot2026/table/` et les
> textures `playmat_grid.jpg` / `attic.jpg` / `crate.png`, que rien ne referencait (3,1 Mo).
> `table_map.py` cite toujours les `.proto` en commentaire : c'est la provenance des cotes qu'il
> porte en dur, pas un fichier qu'il charge.

---

## Écran local Raspberry Pi

L'interface locale affiche la position et le cap du robot, son équipe et son mode,
le chronomètre du match ainsi que les derniers journaux Python. Elle utilise
Tkinter et ne dépend ni de Rerun ni d'une classe matérielle particulière : une
source peut être branchée avec `RobotDashboard.set_data_provider()`.

Prérequis : Python **3.14 ou plus récent** (version déclarée dans `pyproject.toml`)
et une session graphique Raspberry Pi. Depuis la racine du dépôt :

```bash
python3.14 --version
python3.14 -m venv .venv
source .venv/bin/activate
python -m pip install -e .
python -m pip install -r requirements.txt
python -m robot1.rasp.main
```

Si `python3.14 --version` échoue, Python 3.14 n'est pas installé. Sous Windows,
créer l'environnement avec `py -3.14 -m venv .venv`, puis utiliser
`.venv\Scripts\python.exe` pour les installations et le lancement. L'installation
éditable rend `usb_com` et `robot1.rasp` importables ; utiliser le même
interpréteur Python pour toutes les commandes.

La fenêtre démarre en plein écran. `F11` bascule le plein écran et `Échap` ferme
l'interface et demande l'arrêt du robot. Les journaux du processus sont affichés
à partir du niveau INFO. La pose est lue depuis un instantané thread-safe fourni
par `Robot.dashboard_snapshot()` ; les coordonnées sont en millimètres et le cap
est affiché en degrés.

Le panneau `Diagnostic logiciel` indique si des trames d'odométrie Teensy sont
reçues récemment, l'état de connexion exposé par le LiDAR, et les valeurs lues
sur les entrées GPIO de tirette et de couleur. `IMU/OTOS/drivers non vérifiés`
signifie qu'aucun statut de ces composants n'est actuellement consommé par la
boucle Raspberry Pi. Ces indicateurs vérifient des échanges ou des lectures
logiciels ; ils ne prouvent pas l'intégrité physique d'un câble, d'une
alimentation ou d'un capteur.
