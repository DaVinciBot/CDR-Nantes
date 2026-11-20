# USB Communication - C++

> Côté Teensy du protocole USB Rasp ↔ Teensy. Utilisé par `teensy_moteur` et
> `teensy_actuator` (lien symbolique `com=symlink://../../common/usb_com/cpp` dans leur
> `platformio.ini`). Le côté Rasp est dans [../python/](../python/README.md) : les deux doivent
> rester identiques à l'octet près.

---

## Structure

```text
cpp/
├── library.json
├── include/
│   ├── com.h        # Classe Com : réception, envoi, callbacks, NACK
│   ├── messages.h   # IDs, structures des messages, drapeaux
│   ├── cobs.h       # Encodage COBS (sans Arduino)
│   └── crc.h        # CRC8 par table
└── src/
    ├── com.cpp
    ├── cobs.cpp
    └── crc.cpp
```

---

## Format de trame

```text
COBS( [ID][données][CRC8] )  0x00
```

- **ID** : 1 octet. Rasp → Teensy de 0 à 127, Teensy → Rasp de 128 à 255, `NACK` = 127.
- **Données** : la structure du message, sans padding (`#pragma pack(push, 1)`). ID + données
  font au plus `COM_MAX_PAYLOAD` = 255 octets.
- **CRC8** : polynôme 0x07, **valeur initiale 0xFF**, calculé sur ID + données. Pas 0 : avec 0,
  les 0x00 de tête ne changent pas le CRC, et un octet parasite devant une trame passerait.
- **COBS** (Consistent Overhead Byte Stuffing) retire tous les 0x00 de la trame, pour un octet de
  plus tous les 254. Le **0x00 final délimite donc toujours une trame** : aucune donnée ne peut
  couper sa propre trame, et après du bruit ou une trame tronquée, le flux est resynchronisé au
  0x00 suivant.

Il n'y a plus d'octet de taille ni de signature de fin : c'est le délimiteur qui borne la trame.

---

## Classe `Com` (include/com.h)

```cpp
#include <com.h>

Com* com;
void (*callback_functions[256])(byte* msg, byte size);  // indexé par ID, nullptr par défaut

void set_target(byte* msg, byte size) {
    if (size != sizeof(msg_set_target)) return;
    msg_set_target* m = (msg_set_target*)msg;  // msg[0] est l'ID
    // m->x, m->y, m->theta, m->vx, m->vy, m->omega
}

void setup() {
    com = new Com(&Serial, 115200);
    callback_functions[SET_TARGET] = &set_target;
}

void loop() {
    com->handle_callback(callback_functions);
}
```

- `handle_callback()` lit ce qui est disponible jusqu'à la fin de la **première** trame complète
  et appelle le callback de son ID, avec `size` = ID + données. Une trame incomplète est gardée
  pour l'appel suivant.
- Trame invalide (COBS invalide, CRC faux, trop longue, trop courte) : `NACK` envoyé, trame
  comptée (`rejected_frames()`).
- `NACK` reçu : renvoi du dernier message envoyé. ID sans callback : réponse `UNKNOWN_MSG_TYPE`.
- `send_msg(msg, size)` : ajoute le CRC, encode, écrit la trame et son délimiteur. Aucune
  allocation.
- `send_log(msg, size)` : même trame, pour les messages de log émis à chaque tick. Trois
  différences avec `send_msg()` :
  - rien n'est écrit si la trame entière ne tient pas tout de suite dans le tampon d'émission
    (`availableForWrite()`). La fonction rend alors `false` : la trame est perdue, à l'appelant
    de la compter. Sur une Teensy 4, une écriture USB bloque jusqu'à 120 ms quand l'hôte ne lit
    plus : un log ne doit jamais coûter ça à la boucle d'asservissement ;
  - le message n'est pas gardé pour retransmission : un `NACK` ne fait jamais renvoyer une ligne
    de log à la place d'un vrai message ;
  - pas de `flush()`.

  À appeler depuis `loop()`, comme `send_msg()` : les deux partagent leurs tampons.
- `print(text)` : message `PRINT`, texte coupé à 254 caractères.

**`Serial` porte le protocole** : un `Serial.print` dans un firmware qui utilise `Com` sur
`Serial` corrompt le flux. Passer par `com->print()`.

---

## Messages (include/messages.h)

> État : le contrat est défini des deux côtés (structures C++, formats Python, tests de cadrage).
> Le firmware de `teensy_moteur` ne l'implémente pas encore : son `main.cpp` actuel est un
> firmware de test qui n'ouvre pas la liaison USB. Côté Rasp, `app.py` envoie `SET_TARGET` et
> `SET_ODOMETRIE` et lit `ODOMETRY` ; `robot1/sim2d/` simule ces trois messages. Les messages
> `LOG_*` ne sont émis par aucun firmware et lus par aucun programme pour l'instant.

Conventions, valables pour tous les messages :

- **Taille** = données seules, ID exclu. C'est ce que reçoit un callback Python ; côté C++, le
  `size` passé au callback compte l'ID en plus.
- Little-endian, sans padding. `float` = 4 octets, `double` = 8, `int` = 4 (ARM), `bool` = 1.
- Unités : mm, rad, s, µs, rpm. Repère table sauf mention contraire.
- Format Python : chaîne `struct` de `PAYLOAD_FORMATS` (`python/messages.py`), utilisée par
  `pack_message()` / `unpack_message()`. Les messages actionneurs n'y sont pas : leurs appelants
  les empaquettent eux-mêmes.
- La taille de chaque structure de la base roulante est fixée par un `static_assert` à la fin de
  `messages.h` : une structure qui change de taille ne compile plus tant que le contrat n'est pas
  repris des deux côtés.

### Vue d'ensemble

| ID | Message | Sens | Taille | Format Python |
| ---: | --- | --- | ---: | --- |
| 0 | `SET_TARGET` | Rasp → Teensy | 24 | `<6f` |
| 1 | `SET_PID` | Rasp → Teensy | 13 | `<B3f` |
| 2 | `SET_ODOMETRIE` | Rasp → Teensy | 24 | `<3d` |
| 3 à 8 | actionneurs | Rasp → Teensy | voir plus bas | appelant |
| 9 | `STOP` | Rasp → Teensy | 0 | `<` |
| 10 | `SET_ESTIMATOR` | Rasp → Teensy | 1 | `<B` |
| 11 | `SET_TRACE` | Rasp → Teensy | 1 | `<B` |
| 12 | `HEARTBEAT` | Rasp → Teensy | 0 | `<` |
| 126 | `RESET_TEENSY` | Rasp → Teensy | 0 | `<` |
| 127 | `NACK` | les deux | 0 | — |
| 128 | `ODOMETRY` | Teensy → Rasp | 28 | `<6fI` |
| 129 | `SWITCH_STATE_RETURN` | Teensy → Rasp | 2 | appelant |
| 130 | `STATUS` | Teensy → Rasp | 16 | `<IHBBII` |
| 131 | `PID_TRACE` | Teensy → Rasp | 113 | `<I27fB` |
| 200 | `LOG_TICK` | Teensy → Rasp | 184 | voir `PAYLOAD_FORMATS` |
| 201 | `LOG_EVENT` | Teensy → Rasp | 28 | `<BHIIB3fI` |
| 202 | `LOG_FW_INFO` | Teensy → Rasp | 45 | `<B24s20s` |
| 203 | `LOG_PARAMS` | Teensy → Rasp | 135 | voir `PAYLOAD_FORMATS` |
| 254 | `PRINT` | Teensy → Rasp | variable | — |
| 255 | `UNKNOWN_MSG_TYPE` | Teensy → Rasp | 1 | — |

### Base roulante, Rasp → Teensy

#### `SET_TARGET` (0) : cible de position

| Octets | Type | Champ | Unité |
| --- | --- | --- | --- |
| 0-3 | float | `x` | mm |
| 4-7 | float | `y` | mm |
| 8-11 | float | `theta` | rad |
| 12-15 | float | `vx` | mm/s |
| 16-19 | float | `vy` | mm/s |
| 20-23 | float | `omega` | rad/s |

Pose cible, repère table, et vitesse de la trajectoire à cette cible. La vitesse est un
feed-forward : le firmware l'ajoute à la sortie des PID **après** le plafond de freinage, pour
qu'une cible mobile freine sur son propre profil et non sur la distance qui l'en sépare. Pour une
simple cible de pose, les trois vitesses valent zéro :

```python
com.send_bytes(pack_message(Messages.SET_TARGET, x, y, theta, 0.0, 0.0, 0.0))
```

#### `SET_PID` (1) : gains d'un PID

| Octets | Type | Champ | Valeurs |
| --- | --- | --- | --- |
| 0 | uint8 | `pid_type` | 0 = X, 1 = Y, 2 = theta (`X_PID_ID`, `Y_PID_ID`, `THETA_PID_ID` de `config.h`) |
| 1-4 | float | `kp` | |
| 5-8 | float | `ki` | |
| 9-12 | float | `kd` | |

#### `SET_ODOMETRIE` (2) : recalage de la pose

| Octets | Type | Champ | Unité |
| --- | --- | --- | --- |
| 0-7 | double | `x` | mm |
| 8-15 | double | `y` | mm |
| 16-23 | double | `theta` | rad |

Impose la pose de l'estimateur : position de départ, ou recalage après une correction externe.
C'est le seul message de la base resté en `double`.

#### `STOP` (9) : arrêt franc

Sans données. Arrêt immédiat des moteurs et retour au repos. Ce n'est **pas** « va à ta position
actuelle » : aucune consigne n'est maintenue après.

#### `SET_ESTIMATOR` (10) : choix de l'estimateur de pose

| Octets | Type | Champ | Valeurs |
| --- | --- | --- | --- |
| 0 | uint8 | `estimator` | 0 = `ESTIMATOR_COMPLEMENTARY`, 1 = `ESTIMATOR_EKF6` (Python : `Estimator`) |

Refusé tant que les roues tournent. Le refus se lit dans `STATUS` (drapeau
`STATUS_ESTIMATOR_REFUSED`). Le nouvel estimateur repart de la pose de l'ancien.

#### `SET_TRACE` (11) : messages émis à chaque tick

| Octets | Type | Champ | Valeurs |
| --- | --- | --- | --- |
| 0 | uint8 | `mask` | masque `TRACE_MASK_*` (Python : `TraceMask`), ci-dessous |

| Bit | Masque | Message activé |
| ---: | --- | --- |
| 0 | `TRACE_MASK_PID` | `PID_TRACE` : réglage des PID |
| 1 | `TRACE_MASK_LOG_TICK` | `LOG_TICK` : capteurs bruts, entrée et sortie de l'estimateur |

Les deux sont **coupés au démarrage** et s'activent séparément ; un message coupé ne coûte rien
au tick. C'est l'émetteur qui choisit ce qu'il lui faut : tout pour un essai de calibration, ce
que le temps de cycle permet en match. La valeur 1 garde le sens qu'elle avait quand ce champ
était un simple marche / arrêt de `PID_TRACE`. Après un redémarrage de la carte, les deux
retombent : l'état réel se lit dans `STATUS`.

Chaque `SET_TRACE` reçu fait aussi renvoyer `LOG_FW_INFO` et `LOG_PARAMS`, quel que soit le
masque : la Rasp ouvre souvent le port après le démarrage de la carte, et les trames du
démarrage sont alors perdues. Un enregistreur qui envoie `SET_TRACE` en s'accrochant reçoit donc
toujours de quoi dater et rejouer son enregistrement.

#### `HEARTBEAT` (12) : entretien du watchdog

Sans données. Le firmware s'arrête s'il ne reçoit aucun message pendant `WATCHDOG_TIMEOUT_MS`
(3 s, `config.h`). N'importe quel message entretient le watchdog ; `HEARTBEAT` sert quand il n'y a
rien d'autre à envoyer. Côté Rasp, `Com.start_heartbeat()` en envoie un toutes les 0,5 s.

#### `RESET_TEENSY` (126)

Sans données. Redémarre la carte. Le port USB disparaît puis réapparaît : `Com` ne se reconnecte
pas tout seul, il faut recréer l'objet.

### Base roulante, Teensy → Rasp

#### `ODOMETRY` (128) : pose et vitesse estimées

| Octets | Type | Champ | Unité |
| --- | --- | --- | --- |
| 0-3 | float | `x` | mm |
| 4-7 | float | `y` | mm |
| 8-11 | float | `theta` | rad |
| 12-15 | float | `vx` | mm/s |
| 16-19 | float | `vy` | mm/s |
| 20-23 | float | `omega` | rad/s |
| 24-27 | uint32 | `t_us` | µs |

Sortie de l'estimateur actif, repère table. `t_us` est le `micros()` de la carte au moment de
l'estimation : il repasse par zéro toutes les 71,6 minutes, donc comparer des écarts, jamais des
valeurs absolues.

```python
x, y, theta, vx, vy, omega, t_us = unpack_message(Messages.ODOMETRY, data)
```

#### `STATUS` (130) : état de la carte

| Octets | Type | Champ | Contenu |
| --- | --- | --- | --- |
| 0-3 | uint32 | `t_us` | `micros()` de la carte |
| 4-5 | uint16 | `flags` | drapeaux `STATUS_*`, ci-dessous |
| 6 | uint8 | `estimator` | estimateur actif, `ESTIMATOR_*` |
| 7 | uint8 | `estimator_health` | santé de l'estimateur, ci-dessous |
| 8-11 | uint32 | `rs485_read_failures` | lectures d'encodeurs sans réponse, depuis le démarrage |
| 12-15 | uint32 | `rs485_write_failures` | consignes de vitesse non envoyées, depuis le démarrage |

Drapeaux `flags` (Python : `StatusFlag`) :

| Bit | Drapeau | Sens |
| ---: | --- | --- |
| 0 | `STATUS_MOTORS_ENABLED` | drivers des moteurs activés |
| 1 | `STATUS_MOVING` | mouvement en cours |
| 2 | `STATUS_WATCHDOG_TRIPPED` | aucun message reçu pendant `WATCHDOG_TIMEOUT_MS` |
| 3 | `STATUS_TIMEOUT_TRIPPED` | mouvement plus long que son timeout |
| 4 | `STATUS_IMU_OK` | BNO085 démarrée |
| 5 | `STATUS_OPTICAL_OK` | OTOS démarré |
| 6 | `STATUS_ESTIMATOR_REFUSED` | dernier `SET_ESTIMATOR` refusé (roues en mouvement) |
| 7 | `STATUS_TRACE_ON` | `PID_TRACE` active |
| 8, 9 | — | gardés pour « base bloquée » et « drivers sans alimentation » |
| 10 | `STATUS_LOG_TICK_ON` | `LOG_TICK` actif |

`estimator_health` reprend les drapeaux de l'estimateur
(`teensy_moteur/lib/pose_estimator/include/pose_estimator.h`) : bit 0 `FILTER_READY`
(l'estimation est fiable), bit 1 `OPTICAL_OK` (capteur optique utilisé et cohérent), bit 2
`MEAS_REJECTED` (au moins une mesure rejetée au dernier tick).

#### `PID_TRACE` (131) : une ligne de trace par tick

Envoyé à chaque tick d'asservissement tant que `SET_TRACE` l'active. Contient ce qu'il faut pour
régler les PID : réponses indicielles, terme dominant, action du limiteur, suivi des drivers.

| Octets | Type | Champ | Contenu |
| --- | --- | --- | --- |
| 0-3 | uint32 | `t_us` | `micros()` de la carte |
| 4-15 | float[3] | `target` | consigne x, y, theta |
| 16-27 | float[3] | `pose` | pose estimée x, y, theta |
| 28-39 | float[3] | `velocity_cmd` | sortie PID + feed-forward, repère table (mm/s, rad/s) |
| 40-51 | float[3] | `wheel_requested` | vitesses roues 1, 2, 3 avant la limite d'accélération (rpm) |
| 52-63 | float[3] | `wheel_cmd` | vitesses roues envoyées aux drivers (rpm) |
| 64-75 | float[3] | `wheel_measured` | vitesses roues mesurées par les encodeurs (rpm) |
| 76-87 | float[3] | `p_term` | terme P des PID x, y, theta |
| 88-99 | float[3] | `i_term` | terme I |
| 100-111 | float[3] | `d_term` | terme D |
| 112 | uint8 | `flags` | drapeaux `TRACE_*`, ci-dessous |

Drapeaux (Python : `TraceFlag`) :

| Bit | Drapeau | Sens |
| ---: | --- | --- |
| 0 | `TRACE_BRAKING` | le plafond de freinage a réduit la commande |
| 1 | `TRACE_SPEED_SATURATED` | une roue dépassait la vitesse max, les trois ont été réduites |
| 2 | `TRACE_ACCEL_LIMITED` | la limite d'accélération a réduit le changement des roues |
| 3 | `TRACE_INTEGRAL_FROZEN` | l'intégrale des PID a été gelée (anti-windup) |

### Logs, Teensy → Rasp

Ce qu'il faut pour rejouer un essai et recalibrer hors du robot. Les quatre messages portent
`schema_version` (`LOG_SCHEMA_VERSION`, 1 aujourd'hui) : toute modification d'une structure
`LOG_*` l'incrémente, et un enregistrement se relit avec la disposition de sa version. Ils
partent par `send_log()`, jamais par `send_msg()`.

`LOG_TICK` et `LOG_EVENT` commencent par le même en-tête :

| Octets | Type | Champ | Contenu |
| --- | --- | --- | --- |
| 0 | uint8 | `schema_version` | |
| 1-2 | uint16 | `seq` | +1 par message de ce type : un trou est une perte |
| 3-6 | uint32 | `tick` | ticks d'asservissement depuis le démarrage |
| 7-10 | uint32 | `t_us` | `micros()` au début du tick (`LOG_TICK`), à l'événement (`LOG_EVENT`) |

#### `LOG_TICK` (200) : une ligne de log par tick

Envoyé à chaque tick tant que `SET_TRACE` l'active. Il se lit avec le `PID_TRACE` du même tick
(même `t_us`), qui porte la commande.

L'entrée de l'estimateur est écrite **telle qu'elle lui a été donnée**, pas reconstruite à partir
des valeurs brutes : rejouer l'estimateur hors du robot demande exactement les mêmes nombres. Les
valeurs brutes sont **non corrigées** (ni rotation de montage, ni facteur d'échelle logiciel),
pour qu'une calibration puisse être refaite sur un enregistrement.

| Octets | Type | Champ | Contenu |
| --- | --- | --- | --- |
| 0-10 | | en-tête | ci-dessus |
| | | **Entrée de l'estimateur** | |
| 11-22 | int32[3] | `d` | écarts de compteurs des roues 1, 2, 3 depuis le tick précédent |
| 23-26 | uint32 | `dt_us` | durée depuis le tick précédent |
| 27-30 | uint32 | `encoder_age_us` | âge de la dernière lecture d'encodeurs réussie |
| 31-34 | float | `heading_rad` | cap IMU, repère table |
| 35-38 | uint32 | `imu_timestamp_us` | instant de l'échantillon IMU |
| 39-42 | uint32 | `imu_age_us` | son âge à ce tick |
| 43-46 | uint32 | `optical_timestamp_us` | instant de l'échantillon optique |
| 47-54 | float[2] | `optical_dx_mm`, `optical_dy_mm` | pas depuis l'échantillon précédent, repère robot |
| 55-58 | uint32 | `optical_step_dt_us` | durée que ce pas couvre |
| 59-66 | float[2] | `optical_vx_mm_s`, `optical_vy_mm_s` | vitesse optique, repère robot |
| 67 | uint8 | `input_flags` | drapeaux `LOG_IN_*`, ci-dessous |
| | | **Encodeurs bruts** | |
| 68-91 | int64[3] | `enc_counts` | compteurs absolus des roues 1, 2, 3, ceux que l'estimateur a reçus (rafraîchis seulement quand les trois roues répondent) |
| 92-95 | uint32 | `enc_t_us_mid` | milieu de la lecture |
| 96 | uint8 | `enc_read_ok` | bit i : la roue i + 1 a répondu à la dernière lecture |
| | | **Capteur optique brut, axes du capteur** | |
| 97-108 | float[3] | `otos_pos` | x, y (mm), h (rad), depuis sa dernière remise à zéro |
| 109-120 | float[3] | `otos_vel` | mm/s, mm/s, rad/s |
| 121-132 | float[3] | `otos_acc` | mm/s², mm/s², rad/s² |
| | | **IMU brute** | |
| 133-136 | float | `imu_raw_yaw_rad` | lacet tel que le capteur le donne |
| 137-140 | float | `imu_yaw_offset_rad` | `heading_rad` = lacet brut − offset, ramené dans [−π, π) |
| 141 | uint8 | `imu_accuracy` | statut de précision du rapport, 0 à 3 |
| | | **Sortie de l'estimateur** | |
| 142-153 | float[3] | `est_pose` | x, y (mm), theta (rad), repère table |
| 154-165 | float[3] | `est_vel` | mm/s, mm/s, rad/s, repère table |
| 166 | uint8 | `est_health` | santé de l'estimateur, comme `estimator_health` de `STATUS` |
| 167 | uint8 | `est_flags` | drapeaux `LOG_EST_*`, ci-dessous |
| | | **Commandes** | |
| 168-169 | uint16 | `cmd_count` | `SET_TARGET` acceptés depuis le démarrage |
| 170-173 | uint32 | `cmd_t_us` | `micros()` à la prise en compte du dernier |
| | | **Santé du cycle** | |
| 174-175 | uint16 | `work_us` | travail du tick **précédent** : son ISR, plus ce que `loop()` a fait pour lui (RS485, capteurs, envoi des trames). Saturé à 65535 |
| 176 | uint8 | `health_flags` | drapeaux `LOG_HEALTH_*`, ci-dessous |
| 177-179 | uint8[3] | `rs485_fail` | cause de l'échec de lecture des roues 1, 2, 3, ci-dessous |
| 180-181 | uint16 | `log_dropped` | trames de log perdues depuis le démarrage |
| 182-183 | uint16 | `otos_read_us` | durée de la dernière lecture du capteur optique |

`cmd_count` sert à mesurer la latence de commande sans numéroter `SET_TARGET` : chaque fois qu'il
augmente, une consigne a été prise en compte à ce tick, et la n-ième s'apparie avec la n-ième
envoyée par la Rasp.

| Champ | Bit ou valeur | Nom | Sens |
| --- | ---: | --- | --- |
| `input_flags` | bit 0 | `LOG_IN_FIRST_FRAME` | premier tick après un démarrage ou une remise à zéro |
| | bit 1 | `LOG_IN_ENCODERS_FRESH` | les écarts couvrent exactement un tick |
| | bit 2 | `LOG_IN_IMU_VALID` | |
| | bit 3 | `LOG_IN_OPTICAL_STEP_VALID` | |
| | bit 4 | `LOG_IN_OPTICAL_VELOCITY_VALID` | |
| `est_flags` | bit 0 | `LOG_EST_OPTICAL_USED` | l'échantillon optique a servi |
| | bit 1 | `LOG_EST_OPTICAL_REJECTED` | il a été rejeté |
| | bit 2 | `LOG_EST_PURE_ROTATION` | rotation sur place détectée |
| `health_flags` | bit 0 | `LOG_HEALTH_OVERRUN` | le travail du tick a dépassé la période |
| | bit 1 | `LOG_HEALTH_OPTICAL_READ` | nouvel échantillon optique à ce tick |
| | bit 2 | `LOG_HEALTH_IMU_READ` | nouvel échantillon IMU à ce tick |
| | bit 3 | `LOG_HEALTH_WRITE_FAILED` | consignes de vitesse non envoyées à ce tick |
| `rs485_fail` | 0 | `RS485_READ_OK` | lecture réussie |
| | 1 | `RS485_NO_ANSWER` | pas de réponse |
| | 2 | `RS485_BAD_CHECKSUM` | somme de contrôle fausse |
| | 3 | `RS485_SHORT_FRAME` | trame trop courte |
| | 4 | `RS485_BAD_ADDRESS` | adresse inattendue |

```python
fields = unpack_fields(Messages.LOG_TICK, data)   # dict nom -> valeur, tableaux en listes
```

#### `LOG_EVENT` (201) : un événement

Envoyé une fois quand l'événement se produit, quel que soit `SET_TRACE`.

| Octets | Type | Champ | Contenu |
| --- | --- | --- | --- |
| 0-10 | | en-tête | ci-dessus |
| 11 | uint8 | `event` | `LOG_EVENT_*` (Python : `LogEvent`), ci-dessous |
| 12-23 | float[3] | `value` | donnée de l'événement |
| 24-27 | uint32 | `arg` | donnée de l'événement |

| Valeur | Événement | `value` / `arg` |
| ---: | --- | --- |
| 0 | `LOG_EVENT_BOOT` | `arg` : bit 0 = IMU démarrée, bit 1 = OTOS démarré |
| 1 | `LOG_EVENT_ESTIMATOR_SELECTED` | `arg` = `ESTIMATOR_*` |
| 2 | `LOG_EVENT_POSE_SET` | `value` = x, y, theta imposés à l'estimateur |
| 3 | `LOG_EVENT_WATCHDOG` | |
| 4 | `LOG_EVENT_STOP` | |
| 5 | `LOG_EVENT_BLOCKED` | |
| 6 | `LOG_EVENT_DRIVERS_UNPOWERED` | `arg` = 1 à la perte, 0 au retour |
| 7 | `LOG_EVENT_ERROR` | `arg` = code d'erreur |

`LOG_EVENT_POSE_SET` est ce qui permet de rejouer l'estimateur : il repart de cette pose à ce
tick.

#### `LOG_FW_INFO` (202) : le firmware qui tourne

Envoyé au démarrage et à chaque `SET_TRACE` reçu.

| Octets | Type | Champ | Contenu |
| --- | --- | --- | --- |
| 0 | uint8 | `schema_version` | |
| 1-24 | char[24] | `git_describe` | version du firmware, complétée par des 0x00 |
| 25-44 | char[20] | `build_date` | `AAAA-MM-JJ HH:MM:SS`, complétée par des 0x00 |

#### `LOG_PARAMS` (203) : les constantes dont dépend un rejeu

Envoyé au démarrage, à chaque `SET_TRACE` reçu, et quand un réglage change (`SET_PID`,
`SET_ESTIMATOR` accepté). Un enregistrement qui ne les porte pas ne dit pas avec quels réglages
il a été fait.

| Octets | Type | Champ | Contenu |
| --- | --- | --- | --- |
| 0 | uint8 | `schema_version` | |
| 1 | uint8 | `estimator` | estimateur actif, `ESTIMATOR_*` |
| 2-5 | uint32 | `period_us` | période d'asservissement |
| 6-9 | float | `robot_radius_mm` | centre du robot au contact des roues |
| 10-21 | float[3] | `wheel_diameter_mm` | roues 1, 2, 3 |
| 22-33 | float[3] | `wheel_angle_rad` | roues 1, 2, 3 |
| 34-37 | float | `counts_per_rev` | |
| 38-41 | float | `alpha_encoders` | part des encodeurs dans le filtre complémentaire |
| 42-45 | float | `encoder_noise_counts` | |
| 46-49 | uint32 | `imu_max_age_us` | |
| 50-53 | float | `optical_outlier_mm_s` | |
| 54-57 | float | `optical_noise_mm_s` | |
| 58-61 | float | `pure_rotation_min_rad` | |
| 62-65 | float | `pure_rotation_min_counts` | |
| 66-69 | float | `pure_rotation_tolerance` | |
| 70 | uint8 | `use_encoders` | |
| 71-74 | float | `otos_linear_scalar` | facteur écrit **dans** le capteur |
| 75-78 | float | `otos_angular_scalar` | facteur écrit **dans** le capteur |
| 79-82 | float | `otos_linear_scale_soft` | facteur appliqué en logiciel après la lecture |
| 83-86 | float | `otos_angular_scale_soft` | facteur appliqué en logiciel après la lecture |
| 87-90 | float | `otos_mount_angle_rad` | |
| 91-102 | float[3] | `kp` | PID x, y, theta |
| 103-114 | float[3] | `ki` | |
| 115-126 | float[3] | `kd` | |
| 127-130 | float | `max_wheel_rpm` | |
| 131-134 | float | `max_wheel_accel_rpm_s` | |

### Messages communs

| ID | Message | Données |
| ---: | --- | --- |
| 127 | `NACK` | aucune. Envoyé en réponse à une trame invalide ; à sa réception, le dernier message est renvoyé |
| 254 | `PRINT` | texte brut, sans terminateur, 254 caractères au plus (`com->print()`) |
| 255 | `UNKNOWN_MSG_TYPE` | `type_id` (uint8) : l'ID reçu qui n'a pas de callback |

### Actionneurs (`teensy_actuator`)

Inchangés. Pas de format dans `PAYLOAD_FORMATS` : l'équivalent `struct` est donné pour
l'appelant Python.

| ID | Message | Champs | Taille | Équivalent `struct` |
| ---: | --- | --- | ---: | --- |
| 3 | `SET_SERVO_ANGLE_I2C` | `pin` u8, `angle` u16 (°), `max_angle` u16 (°) | 5 | `<BHH` |
| 4 | `STEPPER_STEP` | `steps` int, `dir` bool, `speed` int, `pin_dir` u8, `pin_step` u8, `enable_pin_driver` u8 | 12 | `<i?iBBB` |
| 5 | `SET_SERVO_ANGLE_DETACH` | `pin` u8, `angle` u16, `max_angle` u16, `detach_delay` int (ms) | 9 | `<BHHi` |
| 6 | `ATTACH_SWITCH` | `pin` u8 | 1 | `<B` |
| 7 | `SET_SERVO_ANGLE` | `pin` u8, `angle` u16, `max_angle` u16 | 5 | `<BHH` |
| 8 | `SET_STEPPER_DRIVER_ACTIVATION_STATE` | `pin` u8, `enable_driver_state` bool (actif bas : `true` = driver désactivé) | 2 | `<B?` |
| 129 | `SWITCH_STATE_RETURN` | `pin` u8, `state` bool | 2 | `<B?` |

### Changements par rapport au protocole de la CDR 2026

| CDR 2026 | Maintenant |
| --- | --- |
| Trame `[ID][données][taille][CRC8][BA DD 1C C5]` | `COBS([ID][données][CRC8]) 0x00` |
| 0 `SET_TARGET_POSITION` : x, y, θ en `double` (24 o) | 0 `SET_TARGET` : x, y, θ, vx, vy, ω en `float` (24 o) |
| 128 `UPDATE_ROLLING_BASIS` : x, y, θ en `double` (24 o) | 128 `ODOMETRY` : x, y, θ, vx, vy, ω en `float` + `t_us` (28 o) |
| — | nouveaux : 9 `STOP`, 10 `SET_ESTIMATOR`, 11 `SET_TRACE`, 12 `HEARTBEAT`, 130 `STATUS`, 131 `PID_TRACE`, 200 à 203 `LOG_*` |

`SET_PID`, `SET_ODOMETRIE`, les messages actionneurs, `RESET_TEENSY`, `NACK`, `PRINT` et
`UNKNOWN_MSG_TYPE` sont inchangés.

**Piège** : `SET_TARGET_POSITION` et `SET_TARGET` font tous deux 24 octets avec le même ID. Un
code resté sur l'ancien `struct.pack('<ddd', ...)` passe donc le contrôle de taille et envoie une
cible absurde. Toujours passer par `pack_message()`.

La liste fait foi dans `messages.h`. Un ID, une structure ou un drapeau modifié ici l'est aussi
dans `python/messages.py`, dans le même commit.

---

## Tests

Pas de test natif PlatformIO pour cette lib. La compatibilité avec le côté Python est vérifiée
par `python/tests/test_framing.py`, dont les trames de référence ont été produites par
`cobs.cpp` + `crc.cpp` compilés sur PC, et dont les tailles de messages sont celles des
`sizeof` de `messages.h`. Pour les messages `LOG_*`, la trame de référence a été produite par
`Com::send_log()` avec chaque champ rempli **par son nom** ; le test Python la relit et doit
retrouver chaque valeur sous le même nom. Les tailles sont en plus tenues par les
`static_assert` de `messages.h`, à chaque compilation.
