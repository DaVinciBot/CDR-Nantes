#  USB Communication - Python

> Implémentation Python du protocole de communication USB pour CDR-Nantes

---

##  Structure du module

```
python/
├── __init__.py
├── messages.py           # Énumération de tous les messages
├── com/
│   ├── __init__.py
│   ├── com.py           # Classe Com principale
│   ├── dummy.py         # Mock pour tests sans hardware
│   └── exceptions.py    # Exceptions personnalisées
└── tools/               # Utilitaires de débogage
```

---

## Classe `Com` (com/com.py)

Gestionnaire principal de la communication USB série avec checksum CRC8 et signature.

### Initialisation

```python
from common.usb_com.python.com.com import Com
from common.usb_com.python.messages import Messages

# Mode hardware (Teensy réelle)
com = Com(
    port="COM5",                    # Port détecté automatiquement
    baudrate=115200,
    serial_number=18421350,         # Numéro de série de la Teensy
    use_crc=True
)

# Mode simulation (port virtuel)
com = Com(
    port="COM1",
    baudrate=115200,
    use_crc=True
)
```

### Envoi de messages

```python
import struct

# Message simple sans payload
com.send_bytes(Messages.RESET_TEENSY.to_bytes())

# Message avec payload structuré
msg = Messages.SET_TARGET_POSITION.to_bytes()
msg += struct.pack('<ddd', 200.0, 0.0, 0.0)  # x, y, theta
com.send_bytes(msg)
```

### Réception avec callbacks

```python
def handle_position(data: bytes):
    """Callback appelé à la réception d'UPDATE_ROLLING_BASIS"""
    x, y, theta = struct.unpack('<ddd', data[:24])
    print(f"Position: X={x:.1f} Y={y:.1f} θ={theta:.3f}")

# Enregistrer le callback
com.add_callback(handle_position, Messages.UPDATE_ROLLING_BASIS.value)

# Démarrer l'écoute (thread séparé)
com.start_listening()
```

---

##  Énumération `Messages` (messages.py)

Liste complète de tous les messages du protocole.

### Messages RPI → Teensy (0-126)

| ID | Nom | Description |
|----|-----|-------------|
| 0 | `SET_TARGET_POSITION` | Définir position cible (x, y, θ) |
| 1 | `SET_PID` | Configuration des gains PID |
| 2 | `SET_ODOMETRIE` | Réinitialiser l'odométrie |
| 3 | `SET_SERVO_ANGLE_I2C` | Servo via PCA9685 (I2C) |
| 4 | `STEPPER_STEP` | Commande moteur pas-à-pas |
| 5 | `SET_SERVO_ANGLE_DETACH` | Servo + détachement auto |
| 6 | `ATTACH_SWITCH` | Activer un switch |
| 7 | `SET_SERVO_ANGLE` | Servo PWM direct |
| 8 | `SET_STEPPER_DRIVER_ACTIVATION_STATE` | Activer/désactiver driver |
| 126 | `RESET_TEENSY` | Redémarrage logiciel |
| 127 | `NACK` | Erreur de transmission |

### Messages Teensy → RPI (128-255)

| ID | Nom | Description |
|----|-----|-------------|
| 128 | `UPDATE_ROLLING_BASIS` | Position courante (x, y, θ) |
| 129 | `SWITCH_STATE_RETURN` | État d'un switch |
| 130-133 | `LIDAR_SCAN_PART1-4` | Données LIDAR (4 paquets) |
| 254 | `PRINT` | Message texte de debug |
| 255 | `UNKNOWN_MSG_TYPE` | Type de message inconnu |

**Exemple :**

```python
from common.usb_com.python.messages import Messages

# Convertir en byte
msg_id = Messages.SET_TARGET_POSITION.to_bytes()

# Obtenir la valeur numérique
value = Messages.UPDATE_ROLLING_BASIS.value  # 128
```

---

##  Classe `DummyCom` (com/dummy.py)

Mock pour les tests sans hardware.

```python
from common.usb_com.python.com.dummy import DummyCom
from common.usb_com.python.messages import Messages

# Créer un mock
com = DummyCom()

# Envoyer un message (enregistré mais pas envoyé)
com.send_bytes(Messages.SET_TARGET_POSITION.to_bytes())

# Simuler une réception
fake_data = struct.pack('<ddd', 100.0, 50.0, 1.57)
com.simulate_receive(Messages.UPDATE_ROLLING_BASIS.value, fake_data)
```

---

##  Format des messages

### Structure générale

```
┌──────────┬──────────────┬──────────┬─────────┬─────────────────────────┐
│  ID msg  │   Payload    │  Taille  │   CRC8  │      Signature          │
│  1 byte  │  N bytes     │  1 byte  │  1 byte │  0xBA 0xDD 0x1C 0xC5    │
└──────────┴──────────────┴──────────┴─────────┴─────────────────────────┘
```

### Exemple : `SET_TARGET_POSITION`

```python
import struct
from common.usb_com.python.messages import Messages

# Construire le message
msg = Messages.SET_TARGET_POSITION.to_bytes()  # ID = 0
msg += struct.pack('<ddd', 200.0, 0.0, 0.0)    # x, y, theta (24 bytes)

# La classe Com ajoute automatiquement :
# - Taille du payload (1 byte)
# - CRC8 (1 byte)
# - Signature (4 bytes : 0xBA 0xDD 0x1C 0xC5)

com.send_bytes(msg)
```

---

## Vérification CRC8

Le CRC8 est calculé sur `[payload + taille]` et vérifié automatiquement :

-  CRC valide → message traité
-  CRC invalide → envoi `NACK(127)` → retransmission

```python
# Activer/désactiver la vérification CRC
com = Com(port="COM1", baudrate=115200, use_crc=True)  # Recommandé
```

---

##  Gestion des erreurs

### Types d'exceptions

```python
from common.usb_com.python.com.exceptions import (
    CommunicationError,     # Erreur générique
    TimeoutError,           # Timeout de réception
    CRCError,               # Checksum invalide
    ConnectionError         # Perte de connexion USB
)

try:
    com.send_bytes(msg)
except CommunicationError as e:
    logger.error(f"Erreur de communication : {e}")
```

### Reconnexion automatique

La classe `Com` tente automatiquement de se reconnecter en cas de déconnexion USB.

---

##  Débogage

### Activer les logs

```python
import logging

logging.basicConfig(level=logging.DEBUG)
com = Com(port="COM1", baudrate=115200)

# Affiche tous les messages envoyés/reçus
```

### Outils disponibles

```bash
# Vérifier les ports USB disponibles
python common/usb_com/python/tools/list_ports.py

# Dump des messages en hexadécimal
python common/usb_com/python/tools/hex_dump.py
```

---

##  Voir aussi

- [Protocole de communication](../../../PROTOCOLE_COMMUNICATION.md)
- [Implémentation C++](../cpp/README.md)
- [Tests de communication](../../../robot1/rasp/test/README.md)
- [Documentation complète](../../../Documentation_CDR_Nantes_2026.md)
