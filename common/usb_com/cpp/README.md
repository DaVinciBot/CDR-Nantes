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

Unités : mm, rad, s, µs, rpm. Repère table sauf mention contraire.

### Rasp → Teensy

| ID | Message | Contenu |
| --- | --- | --- |
| 0 | `SET_TARGET` | x, y, θ, vx, vy, ω (float). Vitesse de la trajectoire à la cible, nulle pour une simple cible de pose |
| 1 | `SET_PID` | `pid_type`, kp, ki, kd |
| 2 | `SET_ODOMETRIE` | x, y, θ (double) : recalage de la pose |
| 3 à 8 | actionneurs | servos, pas-à-pas, switch (`teensy_actuator`) |
| 9 | `STOP` | arrêt franc, retour à l'arrêt |
| 10 | `SET_ESTIMATOR` | `ESTIMATOR_COMPLEMENTARY` (0) ou `ESTIMATOR_EKF6` (1), refusé si les roues tournent |
| 11 | `SET_TRACE` | `PID_TRACE` activée (1) ou non (0) |
| 12 | `HEARTBEAT` | entretient le watchdog quand rien d'autre n'est à envoyer |
| 126 | `RESET_TEENSY` | redémarrage |

### Teensy → Rasp

| ID | Message | Contenu |
| --- | --- | --- |
| 128 | `ODOMETRY` | x, y, θ, vx, vy, ω (float), `t_us` |
| 129 | `SWITCH_STATE_RETURN` | état d'un switch |
| 130 | `STATUS` | `t_us`, drapeaux `STATUS_*`, estimateur actif et sa santé, échecs RS485 en lecture et en écriture |
| 131 | `PID_TRACE` | une ligne par tick : consigne, pose, vitesse commandée, vitesses roues avant et après limiteur et mesurées, termes P / I / D, drapeaux `TRACE_*` |
| 254 | `PRINT` | texte de debug |
| 255 | `UNKNOWN_MSG_TYPE` | ID reçu sans callback |

La liste fait foi dans `messages.h`. Un ID, une structure ou un drapeau modifié ici l'est aussi
dans `python/messages.py`, dans le même commit.

---

## Tests

Pas de test natif PlatformIO pour cette lib. La compatibilité avec le côté Python est vérifiée
par `python/tests/test_framing.py`, dont les trames de référence ont été produites par
`cobs.cpp` + `crc.cpp` compilés sur PC, et dont les tailles de messages sont celles des
`sizeof` de `messages.h`.
