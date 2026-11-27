# replay — exploiter un dossier de run sur PC

Outils hors ligne pour les dossiers écrits par le log du run (`robot1/rasp/telemetry/logger.py`,
format dans [../rasp/telemetry/README.md](../rasp/telemetry/README.md)). Ils tournent sur un PC,
jamais sur le robot, et ne modifient ni `log.jsonl` ni `raw_usb.bin` : ce qu'ils calculent est
écrit à côté (`align.json`, `report.json`, `replay.rrd`, `est_replay.json`).

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
python -m robot1.replay.estimators.complementary logs/2027-01-06_1432_match   # écrit est_replay.json
```

Un run `sim2d` se rejoue de la même façon :

```bash
python -m robot1.sim2d.run --scenario carre --log --trace --log-tick --log-dir logs
python -m robot1.replay.report logs/<dossier du run>
```

## Les outils

| Fichier | Rôle |
| --- | --- |
| `align.py` | met la Rasp et la Teensy sur un même temps `t` |
| `load.py` | charge un run : une table pandas par flux, avec ses contrôles |
| `report.py` | rapport par run : lien, cycles, latence des commandes, suivi, échelons, roues, vérité terrain |
| `to_rerun.py` | export `.rrd`, même arborescence d'entités que le pont Rerun |
| `estimators/` | rejeu du filtre complémentaire de la carte, comparé à l'estimation loggée ; EKF hors ligne, sa cohérence, défauts injectés, réglage de `Q` / `R` |

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
| `rejeu` | `est_replay.json` | résultat du rejeu de l'estimateur, s'il a été lancé (le rapport ne le lance pas) |

Un échelon n'est mesuré que si la cible reste fixe au moins 0,3 s : une cible qui avance par
petits pas (suivi de chemin) n'est pas un échelon. Le retard des roues est compté **en plus** du
tick qui sépare toujours une consigne de sa première mesure. Les seuils sont en tête de
`report.py` : premières valeurs, à revoir avec les premiers runs du robot.

Sur un run `sim2d`, `cycles` et `commandes` demandent `--log-tick` : `work_us` y vaut 0 (coût CPU
non simulé) et la latence des commandes 5 ms (résidu de l'alignement, la latence vraie est
nulle). Sans `--trace`, `suivi` et `roues` sont non disponibles, et `echelons` se mesure sur
`SET_TARGET` + `ODOMETRY`. Avec `--trace`, les roues sortent avec un retard et un τ_v nuls : le
simulateur a des roues idéales.

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

### Rejeu des estimateurs (`estimators/`)

Le filtre complémentaire n'est pas réécrit en Python : le code C++ de la carte
(`lib/pose_estimator`, `lib/kinematics`) tourne sur le PC dans un petit outil,
`teensy_moteur/tools/est_replay`, piloté ligne à ligne par `estimators/native.py`. Il se compile
avec PlatformIO, depuis `robot1/teensy_moteur` :

```bash
pio run -e replay        # -> .pio/build/replay/program(.exe)
```

`EST_REPLAY_BIN` désigne un autre exécutable. Sans lui, le rejeu le dit et s'arrête (code 2), et
les tests sont sautés.

`LOG_TICK` porte à chaque tick l'entrée exacte de l'estimateur (`est_in`) et sa réponse (`est`).
Le rejeu redonne les mêmes entrées au même code et compare, tick par tick :

- position à 1e-3 mm, cap à 1e-6 rad, vitesses à 1e-2 mm/s et 1e-4 rad/s ; drapeaux de santé et
  d'étape **égaux**. Les flottants passent en hexadécimal entre Python et C++, sans arrondi ; ce
  qui reste vient des bibliothèques mathématiques et des compilateurs (quelques unités du dernier
  chiffre). Seuils en tête de `complementary.py` ;
- verdict `REJEU : OK`, `ECART` (premier tick fautif, champs, drapeaux des deux côtés) ou
  `NON CONCLUANT` ; code retour 0, 1 ou 2.

**Où le rejeu peut juger.** `LOG_TICK` est coupé au démarrage : on ne rejoue jamais depuis le
boot. L'état du filtre n'est connu exactement qu'après un `reset()`, que la carte logge en
événement :

| Événement | Ce que fait la carte |
| --- | --- |
| `POSE_SET` | `reset()` à la pose imposée par `SET_ODOMETRIE` |
| `ESTIMATOR_SELECTED` (complémentaire) | `reset()` à la dernière estimation, celle du tick d'avant |

La carte date l'événement après le `reset()`, hors de sa section critique : un tick peut passer
entre les deux. Un événement daté du tick N a donc eu lieu avant N + 1, ou avant N ; le rejeu
essaie les deux et garde celui que le log confirme.

Ailleurs (début du log, trou dans les ticks), le rejeu part « à chaud » de l'estimation loggée,
sans connaître les comptes encodeurs que le filtre retenait encore : ces ticks sont comparés et
affichés comme **approchés**, hors verdict, jusqu'au `reset()` suivant. La continuité se juge
sur `tick`, qui compte tous les ticks de la carte, loggés ou non. Les ticks passés sous l'EKF
sont sautés.

**Régler une constante.** `--set` change une constante du filtre (plusieurs fois si besoin) :

```bash
python -m robot1.replay.estimators.complementary logs/... --set alpha_encoders=0.3
```

Le rejeu n'est alors plus un test : pas de verdict, mais la pose finale du rejeu et celle de la
carte, et leur écart à la vérité terrain quand `truth.json` est une pose.

Les constantes viennent de `LOG_PARAMS` (flottants) ; `counts_to_mm` n'y est pas et se recalcule
comme dans `config.h`, à partir du diamètre de roue et des comptes par tour.

### EKF hors ligne (`estimators/ekf.py`)

À bord, le complémentaire estime la pose. L'EKF 6 états de `lib/kalman` tourne ici, dans le même
`est_replay` (`--estimator ekf6`), sur les entrées que la carte a loggées : **les mêmes essais**
servent à régler les deux filtres.

```bash
python -m robot1.replay.estimators.ekf logs/2027-01-06_1432_seq_01
python -m robot1.replay.estimators.ekf logs/... --params ekf.json --set accel_noise=150
python -m robot1.replay.estimators.ekf logs/seq_* --compare
python -m robot1.replay.estimators.ekf logs/... --fault optical_scale:0.8
```

Les constantes de l'EKF ne sont pas dans `LOG_PARAMS` : ce sont elles qu'on règle. Par défaut,
celles de `provisional_ekf6_params()` (recopiées dans `EkfParams` de `native.py`, à tenir
alignées), puis un fichier `--params` et les `--set`. La géométrie et la période viennent de
`LOG_PARAMS`.

Un filtre se juge sur sa **cohérence**, pas sur son erreur seule :

| Indicateur | Ce qu'il dit |
| --- | --- |
| NIS par source (roues, optique, cap) | moyenne sur les mises à jour acceptées ; attendu 0,925 (χ² à 1 degré tronqué par la porte à 99 %). Au-dessus de 1,3 : trop confiant (`Q` ou `R` trop petits) ; sous 0,7 : trop pessimiste |
| biais | moyenne des innovations normalisées, en score z, par canal (roues 1 à 3, optique x / y, cap) ; au-delà de 4 : erreur de calibration ou dérive |
| blancheur | autocorrélation des innovations normalisées, retards 1 à 20 ; plus de 3 retards hors de ±1,96/√N : le modèle ne colle pas |
| 2 sigma | pose finale contre `truth.json` (son incertitude ajoutée) : dans l'ellipse 2σ en (x, y), à 2σ en cap. Sur beaucoup d'essais : 86,5 % et 95,4 % |
| diagnostics | optique non OK, rejets consécutifs, rapport optique / roues hors bande, covariance hors bornes |

Les innovations viennent de `Ekf6::tick_update()`, qui garde chaque mise à jour scalaire du tick
(les `NisStats` ne gardent que des sommes). `--compare` met l'erreur finale de l'EKF à côté de
celle de l'estimation loggée par la carte, essai par essai, avec la couverture sur l'ensemble.

Le rejeu part d'un `POSE_SET`, avec l'incertitude initiale de `EkfInitialSigmas`. Sans lui, ou
après un trou dans les ticks, il part à chaud de l'estimation de la carte et le dit : son
incertitude initiale est alors sous-estimée. Il tourne sur tous les ticks, quel que soit
l'estimateur actif à bord. Résultat dans `ekf_report.json`, repris par `report.py` (section
`ekf`).

**Défauts injectés** (`--fault`, plusieurs possibles) : ils réécrivent les `est_in` avant le
rejeu, jamais le dossier, et rien n'est écrit (le dossier garde le rejeu de ce qui a été loggé).

| Défaut | Effet | Vu par |
| --- | --- | --- |
| `optical_scale:F` | pas et vitesses optiques × F (capteur qui sous-compte, 0,7 à 0,9) | rapport optique / roues, pas par la porte |
| `optical_dropout:T0,D` | optique à zéro de T0 pendant D secondes, roues qui tournent | porte, puis rejets consécutifs |
| `gyro_drift:R` | cap IMU qui dérive de R rad/s | biais des innovations de cap, couverture |

**Régler `Q` et `R`** (`estimators/ekf_tune.py`) :

```bash
python -m robot1.replay.estimators.ekf_tune --tune logs/seq_01 logs/seq_02 \
    --validate logs/seq_03 logs/seq_04 --grid grille.json --out ekf.json
```

`grille.json` donne les valeurs à essayer, toutes les combinaisons sont rejouées :
`{"accel_noise": [100, 200, 400], "alpha_noise": [1, 2, 4], "optical_sigma_mm_s": [5, 10, 20]}`.
Critère sur les runs de réglage pris ensemble : somme sur les sources de |ln(NIS moyen / 0,925)|,
départagée par l'erreur finale. Les runs de validation sont ensuite jugés avec les meilleures
constantes ; **un run donné dans les deux jeux est refusé** : un filtre jugé sur les essais qui
l'ont réglé ne dit rien. `--out` écrit un fichier que `ekf.py --params` relit.

## Tests

```bash
python -m pytest tests/test_replay_align.py tests/test_replay_load.py \
    tests/test_replay_report.py tests/test_replay_rerun.py tests/test_sim2d_trace.py \
    tests/test_sim2d_log_tick.py tests/test_replay_estimators.py tests/test_replay_ekf.py
```

Les runs de test sont synthétiques et passent par le vrai `RunLogger` (`tests/log_helpers.py`) :
décalage et dérive injectés à retrouver, rebouclage de `t_us`, redémarrage, roues du premier
ordre à retard connu, échelon de pose connu. Pour le rejeu, la « carte » est `est_replay` lui-même,
nourri d'entrées synthétiques qui passent par toutes les branches du filtre : il faut retrouver
son estimation, placer chaque `reset()` au bon tick, et repérer une carte réglée autrement. La
comparaison avec la vraie carte (sa libm, son compilateur) se fait sur une Teensy.

Pour l'EKF, les essais sont bruités selon le modèle même du filtre (`noisy_trial()`) : avec les
constantes du générateur il doit sortir cohérent (NIS, biais, blancheur, couverture sur 20
essais), avec d'autres il doit le dire, chaque défaut injecté doit donner son signal, et le
balayage doit retrouver les constantes du générateur.
