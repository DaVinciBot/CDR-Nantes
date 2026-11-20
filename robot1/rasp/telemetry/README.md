# Télémétrie : log du run et pont Rerun

Deux outils, qui ne dépendent pas l'un de l'autre :

- le **log du run** (`logger.py`) écrit sur disque ce qui est passé sur le lien avec la Teensy
  et ce que la Rasp a décidé. C'est ce qui reste d'un match ou d'un essai ;
- le **pont Rerun** (`rerun_bridge.py`) affiche l'état du robot en direct, pour la mise au point.

**Pendant un match, rien ne sort du robot** : le règlement interdit toute communication avec
l'extérieur de la table. Le log reste sur la Rasp et se récupère après. Les modes `serve` et
`connect` du pont n'existent que dans son propre point d'entrée (`python -m
telemetry.rerun_bridge`) ; le chemin de match (`main.py` -> `app.py`) n'ouvre aucune connexion.

---

## Log du run

### Ce qu'un run laisse

Un dossier par run, nommé d'après la date et le type de run :

```text
logs/2027-01-06_1432_match/
  meta.json     qui, quand, quels réglages, et les compteurs du run
  raw_usb.bin   chaque message du lien, dans les deux sens, daté : la référence
  log.jsonl     les mêmes messages décodés, plus les flux de la Rasp
  lidar.jsonl   scans LiDAR bruts, seulement si le run les garde
```

`raw_usb.bin` fait foi : `log.jsonl` peut toujours être reconstruit à partir de lui, donc un
défaut du décodeur ne fait perdre aucun run.

Le même format sort des trois modes (`hardware`, `dummy`, `sim2d`).

### Activer ou couper

Le log s'active ou se coupe, dans cet ordre de priorité :

1. la variable d'environnement `ROBOT_LOG` (`0` ou `1`) ;
2. `log_config` de `config.json` :

| Clé | Défaut | Rôle |
| --- | --- | --- |
| `enabled` | `true` | écrire un dossier par run |
| `dir` | `logs` | dossier qui reçoit les runs, relatif au répertoire de lancement |
| `pid_trace` | `true` | demander `PID_TRACE` à la Teensy (un message par tick) |
| `log_tick` | `true` | demander `LOG_TICK` à la Teensy (un message par tick) |
| `lidar_raw` | `false` | garder les scans LiDAR bruts |

Coupé, rien n'est écrit et le lien n'est pas écouté : le comportement du robot est le même.
Activé, la Rasp envoie `SET_TRACE` avec le masque choisi, et le renvoie si `STATUS` montre que la
Teensy l'a perdu (elle l'oublie à chaque redémarrage). `meta.json` inscrit ce qui était demandé :
un run se lit sans deviner ce qui manque.

```bash
ROBOT_LOG=0 python -m main        # un lancement sans log
```

### Ce qui ne doit jamais arriver

- **La boucle de match n'attend pas le log.** `tap()` et `emit()` datent et poussent dans une
  file ; un seul thread décode et écrit. File pleine : des lignes de log sont perdues et
  comptées, jamais un tour de boucle.
- **Une panne de disque arrête le log, pas le robot** : carte pleine ou retirée, le log s'arrête
  et le dit dans `meta.json` (`error`). Un log impossible à créer au démarrage est remplacé par
  un log coupé.
- **Les dates sont celles de la boucle qui a produit la ligne** : `time.monotonic_ns()` sur le
  robot (l'heure murale peut sauter quand le réseau la règle), l'horloge virtuelle en `sim2d`,
  où 100 s de match durent moins d'une seconde. `meta.json` dit laquelle (`clock`).

Coût mesuré sur PC, match `sim2d` : 0,03 ms de plus par tour d'une boucle à 50 ms.

### Format d'une ligne de `log.jsonl`

Un objet JSON par ligne. Trois clés communes, puis les champs du flux :

```json
{"t_rasp_ns":300000000,"src":"teensy","stream":"odometry","x":250.0,"y":1000.0,"theta":0.0,"vx":0.0,"vy":0.0,"omega":0.0,"t_us":300000}
{"t_rasp_ns":300000000,"src":"rasp","stream":"cmd_tx","msg":"SET_TARGET","msg_id":0,"x":339.44,"y":821.11,"theta":-1.107,"vx":0.0,"vy":0.0,"omega":0.0}
```

- `t_rasp_ns` : instant côté Rasp, dans l'horloge du run ;
- `src` : `teensy` (message reçu) ou `rasp` ;
- `stream` : le flux. Unités du protocole : mm, rad, s, µs, rpm.

Flux tirés du lien, décodés par `log_format.py` avec les noms de champs de `usb_com`
(`PAYLOAD_FIELDS`) :

| Flux | Message | Contenu |
| --- | --- | --- |
| `cmd_tx` | tout message envoyé | `msg`, `msg_id`, puis ses champs |
| `odometry` | `ODOMETRY` | pose et vitesse estimées, `t_us` |
| `status` | `STATUS` | drapeaux, estimateur, compteurs RS485 |
| `ctrl` | `PID_TRACE` | cible, pose, commande, roues, termes PID |
| `est_in`, `enc`, `otos`, `imu`, `est`, `cmd_rx`, `health` | `LOG_TICK` | un `LOG_TICK` donne une ligne par bloc ; chacune porte `seq`, `tick` et `t_us`, on les rejoint sur `tick` |
| `event`, `fw_info`, `params` | `LOG_EVENT`, `LOG_FW_INFO`, `LOG_PARAMS` | les deux derniers sont aussi recopiés dans `meta.json` |
| `print` | `PRINT` | `text` |
| `rejected` | trame refusée (COBS ou CRC) | `hex` |
| `raw` | ID inconnu ou taille inattendue | `hex`, `reason` : jamais d'exception, jamais de message qui disparaît |

Flux écrits par la Rasp (`log.emit(flux, **champs)`) :

| Flux | Quand | Contenu |
| --- | --- | --- |
| `state` | à chaque tour de boucle | l'état que montre Rerun (`_State`), sans le nuage LiDAR |
| `strat` | quand l'étape de stratégie change | étape, type d'action, cible |
| `path` | à chaque nouveau plan | raison, but, statut, détail, points du chemin |
| `opp` | quand le LiDAR voit un adversaire | position, confiance |
| `match` | début, fin, abandon | `event` |
| `link` | une fois par seconde | messages reçus, envoyés, refusés, perdus |

Un flux de la Rasp ne peut pas porter le nom d'un flux du lien : `emit()` le refuse.

`raw_usb.bin` : enregistrements `[u64 t_ns][u8 sens][u16 longueur][octets]`, petit-boutiste.
Sens : 0 reçu, 1 envoyé, 2 trame refusée. Les octets sont le message `[ID][données]`, après COBS
et CRC : c'est ce que les trois transports ont en commun. Un fichier coupé net (alimentation)
se lit jusqu'au dernier enregistrement complet.

### Relire et vérifier

```bash
python -m robot1.rasp.telemetry.log_reader logs/2027-01-06_1432_match
python -m robot1.rasp.telemetry.log_reader logs/2027-01-06_1432_match --rebuild
```

Le lecteur compte les lignes par flux et dit si le run est complet : run clos, `raw_usb.bin`
terminé sur un enregistrement entier, `log.jsonl` identique ligne à ligne à ce que redonne
`raw_usb.bin`, aucun message perdu par le logger, aucun trou dans les numéros de séquence de la
Teensy. Code de retour non nul sinon. `--rebuild` réécrit `log.jsonl` à partir de
`raw_usb.bin`, en gardant les lignes de la Rasp.

Depuis du code : `log_reader.check_run(dossier)`, `read_lines()`, `read_raw()`, `read_meta()`.

### Brancher un producteur

```python
from robot1.rasp.telemetry.logger import create_run_logger

log = create_run_logger("match")      # ou un log coupé, selon la configuration
log.attach(com)                       # écoute le lien, envoie SET_TRACE
log.emit("strat", step=3, action="DEPLACEMENT")
log.close()                           # vide la file, ferme, complète meta.json
```

`Robot(couleur, clock=..., log=log)` fait le `attach()` lui-même. Le log écoute le lien par
`set_tap()`, que les trois transports implémentent (`comm/link.py`).

### Essais et vérité terrain

Un essai est un run fait pour calibrer ou valider. Son dossier porte en plus :

- dans `meta.json` : `test_type`, `session`, `note`, `operator`, `truth_kind`, et `partial` quand
  il a été forcé sans `LOG_TICK` ;
- `truth.json` : ce qui s'est vraiment passé, mesuré sur la table, repère table, mm et rad.

```json
{"kind": "pose", "method": "marques_table", "final_pose_table": [1250.0, 1000.0, 0.0],
 "uncertainty_mm": 2.0, "uncertainty_rad": 0.0087, "note": ""}
{"kind": "turns", "method": "marque_au_sol", "turns": -10.0, "uncertainty_rad": 0.0087, "note": ""}
{"kind": "none", "reason": "robot immobile"}
```

Le type de l'essai (`trial.TRIAL_TYPES`) fixe la vérité attendue et dit s'il lui faut `LOG_TICK`.
`trial.check_trial(dossier)` ajoute au contrôle du log : la vérité du bon type, et des `LOG_TICK`
réellement reçus pour un essai de calibration. Les essais se lancent avec
`python -m tools.run_test`, voir [../tools/README.md](../tools/README.md).

### Modules du log

| Fichier | Rôle |
| --- | --- |
| `logger.py` | `RunLogger`, son remplaçant coupé, `create_run_logger()` |
| `log_format.py` | format des enregistrements bruts et des lignes, sans thread ni fichier |
| `log_reader.py` | relecture, vérification, reconstruction |
| `trial.py` | types d'essai, vérité terrain, contrôle d'un essai |

Tests : `python -m pytest tests/test_run_logger.py tests/test_trial.py`, depuis la racine du
dépôt.

Pas encore fait : les scans LiDAR ne sont pas encore remis au log (`emit_scan()` existe, la
détection ne l'appelle pas), et les anciens dossiers ne sont pas supprimés automatiquement.

---

## Pont Rerun - visualisation temps reel

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
| `table_map.py` | **seule source de la carte statique** (playmat, bordures, supports et balises, pierres des carrieres) ; cotes lues dans `world.py`, `rerun_bridge.py` et `sim2d` appellent son `log_static_map()` |
| `map_assets/playmat2027/Field.png` | tapis de la table, seule ressource encore utilisee (par `table_map.py`) |

> **Les 4 `RERUN_*.md` (96 Ko) ont ete supprimes le 11/09/2026** : anterieurs au passage en
> `serve_grpc`, ils decrivaient encore `robot.py` (devenu `app.py`) et une ouverture du pont dans
> un navigateur. Le pont sera elague a la refonte ; ce README est desormais
> la seule doc du module. Contenu recuperable dans `git log`.
>
> Supprime en meme temps : les 11 `.proto` **Webots** de `map_assets/eurobot2026/table/` et les
> textures `playmat_grid.jpg` / `attic.jpg` / `crate.png`, que rien ne referencait (3,1 Mo).
> Les cotes de `table_map.py` ne viennent plus des `.proto` : table, bordures et pierres
> sont lues dans `world.py` ; seuls les supports de balises restent ecrits dans le module.
