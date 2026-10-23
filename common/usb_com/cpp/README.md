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
- `print(text)` : message `PRINT`, texte coupé à 254 caractères.

**`Serial` porte le protocole** : un `Serial.print` dans un firmware qui utilise `Com` sur
`Serial` corrompt le flux. Passer par `com->print()`.

---

## Messages (include/messages.h)

> État : le contrat est défini des deux côtés (structures C++, formats Python, tests de cadrage).
> Le firmware de `teensy_moteur` ne l'implémente pas encore : son `main.cpp` actuel est un
> firmware de test qui n'ouvre pas la liaison USB. Côté Rasp, `app.py` envoie `SET_TARGET` et
> `SET_ODOMETRIE` et lit `ODOMETRY` ; `robot1/sim2d/` simule ces trois messages.

Conventions, valables pour tous les messages :

- **Taille** = données seules, ID exclu. C'est ce que reçoit un callback Python ; côté C++, le
  `size` passé au callback compte l'ID en plus.
- Little-endian, sans padding. `float` = 4 octets, `double` = 8, `int` = 4 (ARM), `bool` = 1.
- Unités : mm, rad, s, µs, rpm. Repère table sauf mention contraire.
- Format Python : chaîne `struct` de `PAYLOAD_FORMATS` (`python/messages.py`), utilisée par
  `pack_message()` / `unpack_message()`. Les messages actionneurs n'y sont pas : leurs appelants
  les empaquettent eux-mêmes.

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

#### `SET_TRACE` (11) : trace PID

| Octets | Type | Champ | Valeurs |
| --- | --- | --- | --- |
| 0 | uint8 | `enabled` | 1 = `PID_TRACE` à chaque tick, 0 = coupée |

À couper en match : elle n'existe que pour le réglage des PID.

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
| — | nouveaux : 9 `STOP`, 10 `SET_ESTIMATOR`, 11 `SET_TRACE`, 12 `HEARTBEAT`, 130 `STATUS`, 131 `PID_TRACE` |

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
`sizeof` de `messages.h`.
