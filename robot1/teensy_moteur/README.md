# robot1/teensy_moteur - bas niveau (Teensy 4.1)

Firmware de la base roulante : pilotage des trois moteurs MKS SERVO57D (RS485), lecture des
capteurs d'odométrie (encodeurs, IMU BNO085, capteur optique Qwiic OTOS), estimation de pose et
asservissement en position.

> État : `src/main.cpp` est un **firmware de test** (translation de 100 mm en Y, résultats sur le
> port série). Il n'ouvre pas la liaison USB avec la Raspberry Pi. Le firmware de match de la CDR
> 2026 (boucle USB, watchdog) est dans le tag `archive/precoupe-2026-05`. Les bibliothèques de
> `lib/`, elles, sont celles de la refonte du bas niveau.

---

## Compiler et tester

Depuis `robot1/teensy_moteur/` :

```bash
pio run                    # firmware Teensy (environnement par défaut : teensy41)
pio run -t upload          # flash (teensy-gui)
pio test -e native         # tests des algorithmes, sur le PC, sans carte
pio test -e teensy41       # mesure du temps de calcul de l'EKF, carte branchée
```

L'environnement `native` sert **uniquement** à `pio test`. `pio run -e native` échoue : il
essaierait de compiler `src/main.cpp`, qui dépend d'Arduino.

---

## Structure

```text
teensy_moteur/
├── platformio.ini         environnements teensy41 et native
├── include/
│   └── config.h           toute la configuration de la carte (pins, géométrie, gains, limites)
├── src/
│   └── main.cpp           firmware de test (voir État)
├── lib/
│   ├── holonomic_basis/   chef d'orchestre : boucle d'asservissement, odométrie, moteurs
│   ├── MKSServo/          pilotes RS485 des MKS SERVO57D
│   ├── pid/               PID des trois axes X, Y, theta
│   ├── motion/            commande robot -> vitesses des roues (freinage, saturations)
│   ├── kinematics/        équations des roues, unique exemplaire
│   ├── pose_estimator/    interface commune des estimateurs + filtre complémentaire
│   ├── kalman/            EKF 3 et 6 états, écrit à la main
│   └── sensors/           BNO085 et OTOS, lecture non bloquante sur I2C
└── test/                  tests Unity (voir Tests)
```

La bibliothèque `com` (protocole USB) n'est pas dans `lib/` : elle vient de
`common/usb_com/cpp/` par le lien symbolique de `lib_deps`. Voir
[../../common/usb_com/cpp/README.md](../../common/usb_com/cpp/README.md).

---

## Dépendances entre bibliothèques

```text
                         src/main.cpp
                              |
                       holonomic_basis                       [Arduino]
      +-----------+-----------+-----------+-----------+
      |           |           |           |           |
  MKSServo       pid       sensors      motion   pose_estimator
 [Arduino]    [Arduino]   [Arduino]       |       |         |
   RS485                    I2C           |       |       kalman
                                          |       |         |
                                          +-------+---------+
                                                  |
                                              kinematics
```

Deux familles :

- **Les bibliothèques sans Arduino** : `kinematics`, `motion`, `kalman`, `pose_estimator`. Ce sont
  les algorithmes. Elles se compilent sur le PC et sont testées dans l'environnement `native`.
  Elles ne lisent jamais `config.h` : la géométrie et les paramètres leur sont passés à la
  construction. **Aucun `#include <Arduino.h>` ne doit y entrer**, sinon `pio test -e native` ne
  compile plus.
- **Les bibliothèques liées au matériel** : `MKSServo`, `sensors`, `pid`, `holonomic_basis`.
  Elles ne se testent que sur la carte.

---

## Déroulé d'un cycle d'asservissement

Le travail est coupé en deux, parce que le RS485 et l'I2C sont trop lents et bloquants pour une
interruption :

```text
IntervalTimer (période ASSERVISSEMENT_FREQUENCY, 15 ms)         -> compute_loop(), en ISR
    update_odometry()   construit l'EstimatorInput depuis les buffers des capteurs,
                        fait avancer l'estimateur actif, recopie la pose dans X / Y / THETA
    handle(target)      PID sur l'erreur de position + feed-forward, mis en forme par motion
    lève need_read_encoders / need_send_movement (et need_read_imu / need_read_optical)

loop(), hors ISR
    read_encoders_nonblocking()          RS485 : lit les trois encodeurs dans un buffer
    send_movement_commands_nonblocking() RS485 : envoie les trois vitesses calculées
    imu.poll() / otos.poll()             I2C : un échantillon de plus dans le buffer du capteur
```

L'ISR ne fait jamais d'entrée-sortie : elle lit des buffers remplis par `loop()` et en remplit
d'autres que `loop()` enverra.

---

## Les bibliothèques

### `holonomic_basis` - le chef d'orchestre

Classe `Holonomic_Basis`, qui possède les trois moteurs, les trois PID et le `WheelCommandShaper`,
et reçoit de l'appelant les capteurs et l'estimateur actif.

- **Initialisation** : `define_wheel1/2/3()` (un port série par roue), `init_motors()`,
  `attach_sensors(imu, otos)` (l'un ou l'autre peut être `nullptr`), `set_estimator()`,
  `init_holonomic_basis(x, y, theta)`, `enable_motors()`.
- **Interrupteurs** : `use_optical_flow` et `use_imu` décident si un capteur est lu. Un capteur
  coupé arrive à l'estimateur avec son drapeau de validité à faux. `use_pid_control` est ignoré
  (les PID tournent toujours), il ne reste que pour le firmware de test.
- **Changement d'estimateur** : `select_estimator()` est refusé tant que les roues ne sont pas à
  l'arrêt ; le nouvel estimateur repart de la pose de l'ancien.
- **Trajectoire** : `set_velocity_feedforward()` donne la vitesse de la trajectoire au point
  cible, ajoutée après le plafond de freinage. Zéro pour une simple cible de pose.
- `filtered_wheelN_rpm` : vitesses réellement envoyées. Le nom est historique, il n'y a plus de
  filtre passe-bas.
- `structures.h` : la classe `Point` (x, y, theta).

Le `library.json` s'appelle encore `rolling_basis`, l'ancien nom du dossier.

### `MKSServo` - pilotes des moteurs

- `MKSServo` : un driver MKS SERVO57D sur un port série RS485. Activation, consigne de vitesse
  (`setSpeed`), arrêt sur rampe ou immédiat, lecture d'encodeur (bloquante ou en deux temps
  `sendReadRequest` / `readEncoderResponse`), calibration.
- `MKSGroup` : les trois roues ensemble. Envoi des trois vitesses et lecture des trois encodeurs
  en rafale synchronisée, arrêt de tous les moteurs. `setSpeedsSynced` re-borne les vitesses à
  `MAX_SPEED_RPM` en garde-fou.

Pas de `library.json` : `holonomic_basis` l'inclut par chemin relatif. Les encodeurs comptent
`MKS_COUNTS_PER_REV` = 16384 pas par tour et gardent leur position absolue entre deux démarrages,
d'où la lecture d'amorçage dans `setup()` avant le premier tick.

### `pid` - régulateurs de position

Un PID par axe (X, Y, theta), sortie en tr/min bornée. Bande morte lissée autour de zéro
(frottement statique), dt borné entre `dtMin` et `dtMax`, et **intégration conditionnelle** :
`compute(error, freeze_integral)` gèle l'intégrale quand la suite de la chaîne (plafond de vitesse,
d'accélération ou de freinage) empêche la sortie d'agir. Dépend d'Arduino (horloge).

### `motion` - de la commande robot aux vitesses des roues

`WheelCommandShaper::step()`, une fois par tick, dans cet ordre :

1. **freinage** : la part PID de la commande est plafonnée à la vitesse depuis laquelle le robot
   peut encore s'arrêter sur la distance restante (en translation et en rotation). Le
   feed-forward est ajouté après ce plafond ;
2. repère table -> repère robot, par le cap ;
3. cinématique inverse (`kinematics`), mm/s -> tr/min ;
4. **normalisation proportionnelle** : si une roue dépasse `max_wheel_rpm`, les trois sont
   réduites du même facteur, la direction est conservée ;
5. **limite d'accélération synchronisée** : si une roue change trop vite, les trois changements
   sont réduits du même facteur, donc le robot garde son cap pendant une rampe.

Le résultat (`ShapedCommand`) porte les trois vitesses et des drapeaux de trace : freinage,
saturation de vitesse, limite d'accélération. Ce sont eux qui gèlent l'intégrale des PID.

### `kinematics` - les équations des roues

**Le seul endroit où les équations des roues sont écrites.** Les commandes moteur (`motion`),
l'EKF (`kalman`) et le filtre complémentaire (`pose_estimator`) passent tous par elle : les roues
sont commandées et relues avec des équations exactement inverses l'une de l'autre. Une
modification des équations se fait ici, une seule fois.

- `inverse_kinematics()` : déplacement (ou vitesse) robot -> déplacement des trois roues.
- `forward_kinematics()` : deltas d'encodeurs -> déplacement robot.
- Roues : W1 à 120°, W2 à 240°, W3 à 0°. Un omega positif fait tourner toutes les roues en
  arrière (signe moins sur `R omega`).

Unités : mm, rad, repère robot (x vers l'avant).

### `pose_estimator` - estimation de pose

`pose_estimator.h` définit le contrat commun :

- `EstimatorInput` : tout ce qu'un tick fournit (deltas d'encodeurs, cap IMU, pas et vitesse
  optiques), chaque capteur avec son drapeau de validité. Un capteur qui n'a rien de neuf baisse
  son drapeau, il n'envoie jamais de zéros ;
- `PoseEstimate` : pose et vitesse, repère table ;
- `health_flags()` : `FILTER_READY`, `OPTICAL_OK`, `MEAS_REJECTED`.

Deux implémentations, toutes deux compilées, une seule active :

| Estimateur               | Principe                                                                                   |
| :----------------------- | :----------------------------------------------------------------------------------------- |
| `ComplementaryEstimator` | Filtre de la CDR 2026 réécrit : 20 % encodeurs / 80 % optique sur le déplacement du tick, cap par l'IMU. Seuils exprimés en vitesse (indépendants de la période). Rotation pure détectée : le pas optique est alors ignoré. |
| `EkfEstimator`           | L'EKF 6 états de `kalman` derrière le même contrat.                                        |

Le choix au démarrage se fait par `POSE_ESTIMATOR_DEFAULT` dans `config.h` (complémentaire par
défaut).

### `kalman` - filtre de Kalman étendu

Écrit à la main, sans bibliothèque de matrices, pour tenir le budget de calcul (de l'ordre de
40 à 80 µs par cycle pour 6 états) : stockage de taille fixe (`matrix.h`, sans allocation ni
exception), mises à jour scalaires (rien n'est jamais inversé), `F P F^T` développé à la main.

- `ekf.h` : le cœur commun (mise à jour scalaire en forme de Joseph, porte chi², statistiques
  NIS par source).
- `ekf3.h` : 3 états `[x, y, theta]`, odométrie roues en entrée, cap IMU en mesure. Repli si le
  6 états n'est pas réglé à temps, et modèle simple pour déboguer la tuyauterie.
- `ekf6.h` : 6 états `[x, y, theta, vx, vy, omega]`, le filtre visé. Mesures : vitesses des
  trois roues, vitesse optique (avec le bras de levier du capteur), cap IMU.
- `ekf_kinematics.h` : reprend `kinematics` et ajoute la covariance d'un déplacement calculé à
  partir de roues bruitées.

Les paramètres actuels (`provisional_ekf6_params()`) sont **provisoires** jusqu'à la mesure de R
et Q sur table. X et Y ne sont mesurés par rien : leur variance croît pendant tout le match, c'est
le comportement attendu. Les pièges de conception sont détaillés en tête de `ekf.h`.

### `sensors` - capteurs I2C

Lecture **non bloquante** : `poll()` dans `loop()`, jamais dans l'ISR ; `latest()` donne à l'ISR
une copie prise interruptions coupées.

- `Bno085Sensor` : cap du robot depuis le *game rotation vector* (sans magnétomètre), sur `Wire`.
  `zero_heading()` fixe l'origine. Sa vitesse angulaire n'est pas une seconde mesure : le cap
  l'intègre déjà.
- `OtosSensor` : déplacement et vitesse dans le repère robot, sur `OTOS_WIRE`. Configuration
  réécrite à chaque démarrage (le capteur l'oublie hors tension) ; le cap propre de l'OTOS n'est
  jamais utilisé ; facteurs d'échelle refusés hors de [0.95, 1.05] ; le bras de levier est
  compensé dans l'EKF seulement. La durée de chaque lecture est enregistrée.

---

## `include/config.h`

Toute la configuration propre à la carte, lue par `main.cpp`, `holonomic_basis` et `MKSServo` :

- ports série et adresses des trois roues, pins I2C, réglages BNO085 et OTOS ;
- paramètres MKS (baudrate, microstep, accélération, pas par tour) ;
- géométrie : `ROBOT_RADIUS`, `WHEEL_DIAMETER`, `COUNTS_TO_MM` ;
- limites : `MAX_SPEED_RPM`, `MAX_WHEEL_ACCEL_RPM_S`, vitesses et accélération robot ;
- gains des trois PID, période d'asservissement, choix de l'estimateur ;
- watchdog et timeouts de mouvement.

`ROBOT_RADIUS` (160 mm, centre -> roues, pour la cinématique et l'odométrie) n'a rien à voir avec
`ROBOT_RADIUS_MM` de `robot1/rasp/world.py` (120 mm, gonflage des obstacles pour l'A*). Les deux
sont indépendants : ne jamais les aligner.

---

## Tests

| Suite                 | Où        | Ce qu'elle vérifie                                                          |
| :-------------------- | :-------- | :-------------------------------------------------------------------------- |
| `test_kalman_core`    | `native`  | Cœur de l'EKF (mise à jour, porte, NIS) contre une version dense de référence |
| `test_ekf3`           | `native`  | Filtre 3 états                                                              |
| `test_ekf6`           | `native`  | Filtre 6 états                                                              |
| `test_pose_estimator` | `native`  | Les deux estimateurs derrière le contrat commun                             |
| `test_motion`         | `native`  | Freinage, normalisation, limite d'accélération                              |
| `test_ekf_timing`     | `teensy41`| Temps d'un cycle d'EKF sur la vraie puce (Teensy nue, rien de branché)      |

`test/support/dense_reference.h` contient la version matricielle dense et générique de chaque
équation : la référence que le code écrit à la main doit reproduire, côté tests seulement.
