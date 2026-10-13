# Test du capteur optique OTOS

Firmware de mise en service du SparkFun Qwiic Optical Tracking Odometry Sensor (OTOS) sur une
Teensy 4.1 seule : aucun autre composant n'est nécessaire.

Le firmware déroule les étapes tout seul au démarrage et affiche le résultat de chacune sur le
moniteur série, puis envoie les mesures en continu pour les tests à la main. Il n'y a aucune
commande à taper : tous les réglages sont dans `include/config.h`, à modifier puis recompiler.

## Câblage

| OTOS (Qwiic) | Teensy 4.1 |
| --- | --- |
| 3.3V | 3.3V |
| GND | GND |
| SDA | 17 (SDA1) |
| SCL | 16 (SCL1) |

Les broches 16/17 sont celles du bus **`Wire1`**, pas de `Wire` (18/19). L'exemple SparkFun
appelle `begin()` sans argument, donc sur `Wire` : il ne trouve pas le capteur avec ce câblage.
Ici le firmware appelle `begin(Wire1)`.

Moniteur série : 115200 bauds. Le firmware attend 3 s l'ouverture du port avant de démarrer.

## Déroulé au démarrage

| Étape | Contenu | Appels de la bibliothèque |
| --- | --- | --- |
| 1 | Scan du bus I2C, l'OTOS doit répondre en 0x17 | `Wire1.beginTransmission` |
| 2 | Identification : product ID (0x5F), versions HW et FW | `begin`, `getVersionInfo` |
| 3 | Autotest et registre de status | `selfTest`, `getStatus` |
| 4 | Configuration usine : unités, scalars, offset, traitement du signal | `getLinearScalar`, `getAngularScalar`, `getOffset`, `getSignalProcessConfig` |
| 5 | Calibration IMU après compte à rebours, progression affichée | `calibrateImu`, `getImuCalibrationProgress` |
| 6 | Application des réglages de `config.h`, puis relecture | `setLinearUnit`, `setAngularUnit`, `setLinearScalar`, `setAngularScalar`, `setOffset` |
| 7 | Dérive statique (désactivable) | `resetTracking`, `setPosition`, `getPosition`, `getPositionStdDev` |
| 8 | Temps d'une lecture I2C complète (désactivable) | `getPosVelAcc` |
| flux | Mesures en continu + bilan toutes les secondes | `getPosVelAcc`, `getStatus`, `begin` pour la reconnexion |

Si l'étape 1 ou 2 échoue, le firmware réessaie toutes les secondes en l'affichant.

Pendant le flux :

- une ligne de mesure toutes les 50 ms : x, y (mm), cap (deg), cap déroulé (deg, compte les
  tours complets), vitesses, accélérations, status ;
- une ligne `[SUMMARY]` toutes les secondes : déplacement depuis le départ, cap déroulé en
  degrés et en tours, compteurs de lectures, d'erreurs et de pertes de suivi ;
- une ligne `[EVENT]` dès que le suivi optique est perdu ou retrouvé, ou que l'inclinaison est
  dépassée ;
- `[ERROR] sensor lost` après 3 lectures ratées, puis tentative de reconnexion toutes les
  secondes. À la reconnexion (`[RECOVERED]`), le firmware refait la calibration et réapplique
  les réglages : l'OTOS les perd à chaque coupure d'alimentation.

Avec `STREAM_CSV 1`, les mesures sortent en CSV et toutes les autres lignes commencent par `#`,
pour pouvoir copier la sortie dans un tableur ou un traceur série.

## Réglages (`include/config.h`)

| Réglage | Rôle |
| --- | --- |
| `OTOS_I2C_CLOCK_HZ` | 400000 ou 100000 |
| `RUN_DRIFT_TEST`, `DRIFT_TEST_DURATION_S` | active et dimensionne l'étape 7 |
| `RUN_I2C_BENCHMARK`, `BENCHMARK_READS` | active et dimensionne l'étape 8 |
| `STREAM_PERIOD_MS`, `STREAM_CSV`, `SUMMARY_PERIOD_MS` | cadence et format du flux |
| `LINEAR_SCALAR`, `ANGULAR_SCALAR` | correction d'échelle, dans [0.872, 1.127] |
| `OFFSET_X_MM`, `OFFSET_Y_MM`, `OFFSET_H_DEG` | position du capteur par rapport au centre du robot |
| `START_X_MM`, `START_Y_MM`, `START_H_DEG` | pose de départ après remise à zéro |

## Tests

Deux montages : **M** = capteur sur breadboard ou tenu à la main (qualitatif : la hauteur
varie, on valide que ça marche et le sens des axes, pas la précision) ; **R** = capteur monté
sur le robot, à hauteur fixe (quantitatif). Les tests M servent de go / no-go avant montage.

| # | Test | Montage | Méthode | Attendu |
| --- | --- | --- | --- | --- |
| 1 | Scan I2C | M, R | étape 1, refaire avec `OTOS_I2C_CLOCK_HZ 100000` | un seul périphérique, en 0x17. Rien : câblage, alimentation 3.3 V ou mauvais bus |
| 2 | Identification | M, R | étape 2 | product ID OK, versions HW et FW à noter |
| 3 | Autotest et status | M, R | étape 3 | PASS, les 4 bits de status à 0 (`paa` = capteur optique HS, `lsm` = IMU HS) |
| 4 | Configuration usine | M, R | étape 4 | scalars 1.000, offset nul, `lut/acc/rot/var` à 1, unités par défaut pouce / degré |
| 5 | Calibration IMU | M, R | étape 5, capteur posé à plat et immobile | compteur jusqu'à 0 en environ 0.6 s. Variante : bouger pendant la calibration et comparer la dérive du test 7 |
| 6 | Réception et sens des axes | M, R | flux : pousser selon +X sérigraphié, puis +Y, puis tourner dans le sens trigonométrique | x augmente seul, puis y seul, puis le cap augmente. Noter les sens réels |
| 7 | Dérive statique | M (posé sur la table), R | étape 7 | dx, dy, dh, dérive du cap en deg/min et écart-type affichés. Seuils à fixer après la première mesure |
| 8 | Perte de suivi optique | M | flux : soulever progressivement, incliner, changer de surface (table, tapis de jeu, carton, surface brillante), éclairage fort | `optical tracking LOST` au-delà d'une hauteur à noter, `tilt` à l'inclinaison, pose figée ou sur IMU seul pendant la perte |
| 9 | Débranchement à chaud | M | débrancher le câble Qwiic pendant le flux, puis rebrancher | `[ERROR] sensor lost` sans blocage, puis `[RECOVERED]` et reprise du flux |
| 10 | Temps de lecture I2C | M, R | étape 8 à 400 kHz puis 100 kHz | temps moyen et max d'une lecture, 0 erreur |
| 11 | Translation et scalar linéaire | R | pousser le robot le long d'un bord droit sur 1 m mesuré, en X puis en Y, aller et retour, lire `dist` du `[SUMMARY]` | `scalar = réel / mesuré`, dans [0.872, 1.127]. Le reporter dans `LINEAR_SCALAR` et refaire : erreur inférieure à 1 %, cap quasi constant |
| 12 | Rotation et scalar angulaire | R | 5 à 10 tours sur place, lire `turns` du `[SUMMARY]` | `scalar = tours réels / tours mesurés`, à reporter dans `ANGULAR_SCALAR` puis refaire |
| 13 | Boucle fermée | R (M possible, grossier) | carré de 500 mm ou trajet libre, retour exact au point de départ | erreur de fermeture x, y, cap |
| 14 | Offset capteur | R | mesurer la position du capteur par rapport au centre du robot, la reporter dans `OFFSET_*`, tourner sur place autour du centre | la position reste proche de 0 pendant que le cap tourne |
| 15 | Pose de départ | R | `START_*` = (1000, 500, 90) | le flux part de cette pose et le suivi continue normalement |

## Résultats

| Mesure | Valeur | Date |
| --- | --- | --- |
| Versions HW / FW | | |
| Dérive statique 60 s (dx, dy, dh) | | |
| Dérive du cap (deg/min) | | |
| Hauteur de perte de suivi (mm) | | |
| Surfaces testées et comportement | | |
| Lecture I2C 400 kHz (moyenne / max) | | |
| Lecture I2C 100 kHz (moyenne / max) | | |
| Scalar linéaire retenu | | |
| Scalar angulaire retenu | | |
| Erreur de boucle fermée | | |
| Offset capteur sur le robot | | |
