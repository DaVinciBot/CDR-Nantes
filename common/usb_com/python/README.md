# USB Communication - Python

> Côté Raspberry Pi du protocole USB Rasp ↔ Teensy. Le côté Teensy est dans
> [../cpp/](../cpp/README.md) : les deux doivent rester identiques à l'octet près.

---

## Structure

```text
python/
├── __init__.py
├── messages.py          # IDs, formats des messages, drapeaux
├── com/
│   ├── com.py           # Classe Com : port, thread récepteur, callbacks, heartbeat
│   ├── framing.py       # COBS + CRC8 : encode_frame / decode_frame, sans I/O
│   ├── dummy.py         # DummySerial : port factice, sans matériel
│   └── exceptions.py    # ComError
├── tools/
│   └── get_serial_number.py
└── tests/
    └── test_framing.py  # tests hors ligne du cadrage et des formats
```

`usb_com` vient de `pip install -e .` à la racine du dépôt : `from usb_com import Com, Messages`.

---

## Format de trame

```text
COBS( [ID][données][CRC8] )  0x00
```

CRC8 de polynôme 0x07, valeur initiale 0xFF, sur ID + données. COBS retire tous les 0x00 de la
trame : le 0x00 final la délimite toujours, aucune donnée ne peut la couper, et après du bruit le
flux repart au 0x00 suivant. Détails et raisons : [../cpp/README.md](../cpp/README.md).

`framing.py` ne fait aucune I/O : c'est là que se testent le cadrage et la compatibilité avec le
C++.

---

## Classe `Com` (com/com.py)

Côté `robot1/rasp/`, ne pas instancier `Com` à la main : `init_robot()` lit `config.json`
(section `serial_config`) et choisit le mode d'après `ROBOT_MODE`.

```python
from robot1.rasp.comm import init_robot

com, mode = init_robot(logger)
```

Le port est retrouvé par numéro de série, vid et pid, jamais par un chemin de device. Sans Teensy,
`ComError` est levée, sauf avec `enable_dummy=True` : le lien devient alors un `DummySerial`,
les envois partent et rien ne revient.

### Envoyer

```python
from usb_com import Messages, pack_message

com.send_bytes(pack_message(Messages.SET_TARGET, 200.0, 0.0, 0.0, 0.0, 0.0, 0.0))
com.send_bytes(pack_message(Messages.STOP))
```

`send_bytes()` encode et écrit la trame. Le dernier message envoyé est gardé pour être renvoyé
si la Teensy répond `NACK` (`record=False` pour ne pas le garder).

### Recevoir

```python
from usb_com import Messages, payload_size, unpack_message

def handle_odometry(data: bytes) -> None:
    if len(data) != payload_size(Messages.ODOMETRY):
        return
    x, y, theta, vx, vy, omega, t_us = unpack_message(Messages.ODOMETRY, data)

com.add_callback(handle_odometry, Messages.ODOMETRY.value)
```

Le thread récepteur démarre avec `Com` (sauf en mode dummy). Un callback reçoit les **données
sans l'ID**. Une trame invalide est ignorée et un `NACK` est renvoyé. La lecture a un timeout de
0,1 s, et une trame arrivée en plusieurs morceaux est réassemblée.

### Heartbeat et arrêt

```python
com.start_heartbeat()   # HEARTBEAT toutes les 0,5 s, sous le watchdog de 3 s de la Teensy
...
com.close()             # arrête les threads, ferme le port
```

---

## Messages (messages.py)

Miroir de `cpp/include/messages.h` : un ID, un format ou un drapeau modifié d'un côté l'est de
l'autre dans le même commit. La table complète des messages est dans
[../cpp/README.md](../cpp/README.md).

- `Messages` : les IDs. `Messages.X.to_bytes()` donne l'octet d'ID.
- `PAYLOAD_FORMATS` : format `struct` des données de chaque message de la base roulante
  (little-endian, sans padding, comme les structures C++). Les messages actionneurs sont
  empaquetés par leurs appelants.
- `pack_message(message, *champs)`, `unpack_message(message, données)`,
  `payload_size(message)`.
- `Estimator`, `StatusFlag`, `TraceFlag` : valeurs et drapeaux de `SET_ESTIMATOR`, `STATUS` et
  `PID_TRACE`.

---

## Tests

```bash
python -m unittest discover common/usb_com/python/tests
```

Depuis la racine du dépôt, sans port série ni Teensy. Ils couvrent :

- COBS ;
- les trames de référence produites par le C++ ;
- les tailles des messages contre les `sizeof` C++ ;
- les trames invalides et la resynchronisation ;
- le port factice.
