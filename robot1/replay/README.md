# replay — exploiter un dossier de run sur PC

Outils hors ligne pour les dossiers écrits par le log du run (`robot1/rasp/telemetry/logger.py`,
format dans [../rasp/telemetry/README.md](../rasp/telemetry/README.md)). Ils tournent sur un PC,
jamais sur le robot, et ne modifient ni `log.jsonl` ni `raw_usb.bin` : ce qu'ils calculent est
écrit à côté (`align.json`, `report.json`, `replay.rrd`).

## Installation

```bash
pip install -e ".[dev,analysis]"     # à la racine de CDR-Nantes/ : pandas, numpy, Rerun
```

L'extra `analysis` n'est jamais requis sur la Rasp.

## Lancer

Depuis n'importe quel répertoire :

```bash
python -m robot1.replay.align    logs/2027-01-06_1432_match     # horloges, écrit align.json
python -m robot1.replay.report   logs/2027-01-06_1432_match     # rapport, écrit report.json
python -m robot1.replay.to_rerun logs/2027-01-06_1432_match     # écrit replay.rrd
python -m robot1.replay.to_rerun logs/... --out match.rrd --spawn
```

Un run `sim2d` se rejoue de la même façon :

```bash
python -m robot1.sim2d.run --scenario carre --log --trace --log-dir logs
python -m robot1.replay.report logs/<dossier du run>
```

## Les outils

| Fichier | Rôle |
| --- | --- |
| `align.py` | met la Rasp et la Teensy sur un même temps `t` |
| `load.py` | charge un run : une table pandas par flux, avec ses contrôles |
| `report.py` | rapport par run : lien, cycles, latence des commandes, suivi, échelons, roues, vérité terrain |
| `to_rerun.py` | export `.rrd`, même arborescence d'entités que le pont Rerun |

### Temps commun (`align.py`)

Chaque trame Teensy porte `t_us`, l'instant de la mesure sur la carte ; le log date sa réception
côté Rasp (`t_rasp_ns`). La latence du lien n'est jamais négative : les plus petites valeurs de
`t_rasp − t_teensy` sur des fenêtres d'une seconde dessinent le décalage, et une droite passée par
ces minimums donne la dérive des quartz. Pas de ping, les trames du run suffisent.

`t` est en secondes, dans la base de temps de la Teensy. Une ligne Teensy garde son instant de
mesure ; une ligne de la Rasp y est ramenée.

- `t_us` reboucle toutes les 71,6 min : il est déroulé dans l'ordre d'arrivée des trames.
- Un redémarrage de la Teensy (`t_us` qui repart de zéro, ou `LOG_EVENT` `BOOT`) ouvre un nouveau
  segment, aligné à part. `t` reste croissant d'un segment à l'autre, en passant par l'horloge de
  la Rasp, qui ne redémarre pas.
- Sans aucune trame datée par la Teensy (log coupé côté lien, mode `dummy`), `t` est le temps de
  la Rasp depuis le début du run (méthode `rasp_only`).

### Tables (`load.py`)

```python
from robot1.replay.load import load_run

run = load_run("logs/2027-01-06_1432_otos_lin_x_01")
run.stream("ctrl")[["t", "target_x", "pose_x"]]   # une table par flux
run.tick_table()                                  # les blocs de LOG_TICK réunis par tick
run.warnings                                      # ce qui manque ou cloche
```

Chaque table porte `t`, `segment`, `t_rasp_ns`, `src`, puis les champs du flux. Les champs
tableaux du protocole sont éclatés en colonnes nommées :

| Champs | Colonnes |
| --- | --- |
| `target`, `pose`, `est_pose`, `p_term`, `i_term`, `d_term`, `value` | `_x`, `_y`, `_theta` |
| `velocity_cmd`, `est_vel` | `_vx`, `_vy`, `_omega` |
| `wheel_requested`, `wheel_cmd`, `wheel_measured`, `d`, `enc_counts`, `rs485_fail` | `_1`, `_2`, `_3` |
| `otos_pos`, `otos_vel`, `otos_acc` | `_x`, `_y`, `_h` (axes capteur) |

Les noms de champs viennent du décodeur (`PAYLOAD_FIELDS` de `usb_com`) : une évolution du
schéma des messages `LOG_*` passe sans retouche ici.

Contrôles, dans `run.warnings` : verdict de `log_reader`, redémarrages de la Teensy, trous de
`seq` de `LOG_TICK` et `LOG_EVENT`, ticks incomplets (`PID_TRACE` sans `LOG_TICK` ou l'inverse,
quand les deux étaient demandés), flux demandés dans `meta.json` et jamais reçus. Un run partiel
se charge : un flux absent donne une table vide, jamais une exception.

### Rapport (`report.py`)

Chaque section dit ce qu'il lui faut. Un flux absent la rend « non disponible », flux nommé.

| Section | Flux | Contenu |
| --- | --- | --- |
| `lien` | tous | verdict de relecture, trames refusées, NACK, trous de séquence, lignes perdues |
| `cycles` | `LOG_TICK` | `work_us` p50 / p99 / max contre la période, dépassements, consignes non envoyées, échecs RS485 par roue et par cause, durée de lecture OTOS |
| `commandes` | `LOG_TICK` + `cmd_tx` | latence `SET_TARGET` envoyé → tick qui l'a pris, appariés par rang |
| `suivi` | `PID_TRACE` | erreur cible − pose (RMS, max), accélération et jerk de la commande et du mouvement, saturations par drapeau, pertes de pas |
| `echelons` | `PID_TRACE`, sinon `SET_TARGET` + `ODOMETRY` | par échelon de cible : montée 10–90 %, dépassement, erreur statique, établissement |
| `roues` | `PID_TRACE` | par roue : retard pur, constante de temps τ_v et gain, ajustés sur consigne et vitesse mesurée |
| `verite` | `truth.json` | pose finale (ou tours) estimée contre la vérité terrain |

Un échelon n'est mesuré que si la cible reste fixe au moins 0,3 s : une cible qui avance par
petits pas (suivi de chemin) n'est pas un échelon. Le retard des roues est compté **en plus** du
tick qui sépare toujours une consigne de sa première mesure. Les seuils sont en tête de
`report.py` : premières valeurs, à revoir avec les premiers runs du robot.

Sur un run `sim2d`, `cycles` et `commandes` sont non disponibles (pas de `LOG_TICK`). Sans
`--trace`, `suivi` et `roues` le sont aussi, et `echelons` se mesure sur `SET_TARGET` +
`ODOMETRY`. Avec `--trace`, les roues sortent avec un retard et un τ_v nuls : le simulateur a des
roues idéales.

### Rerun (`to_rerun.py`)

Rejoue un match réel dans le viewer, sans `sim2d`. La vue 3D reprend les entités du pont
(`world/robot/odom`, `world/robot/fused`, `world/pathfinding/target`,
`world/pathfinding/trajectory`, `world/obstacles/adversaries`), plus `world/robot/reference`
(référence du contrôleur) et `world/truth/final` (vérité terrain).

Les courbes sont à la pleine fréquence de chaque flux, sans la décimation à 20 Hz du pont :
`data/<flux>/<champ>`, et pour un champ tableau, une entité par composante, de sorte qu'un même
graphe montre cible et pose (`data/ctrl/x/target`, `data/ctrl/x/pose`) ou consigne et mesure d'une
roue (`data/ctrl/roue_1/wheel_cmd`, `data/ctrl/roue_1/wheel_measured`). Seule la vue 3D est décimée,
à 30 Hz. Événements, stratégie et match vont en texte sous `logs/`.

Lignes de temps : `teensy_time` (le `t` commun) pour tout, `tick` pour les flux de `LOG_TICK`.

## Tests

```bash
python -m pytest tests/test_replay_align.py tests/test_replay_load.py \
    tests/test_replay_report.py tests/test_replay_rerun.py tests/test_sim2d_trace.py
```

Les runs de test sont synthétiques et passent par le vrai `RunLogger` (`tests/log_helpers.py`) :
décalage et dérive injectés à retrouver, rebouclage de `t_us`, redémarrage, roues du premier
ordre à retard connu, échelon de pose connu.
