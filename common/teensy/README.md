#  Module Teensy - Classes Python

> Bibliothèque Python pour l'interaction avec les microcontrôleurs Teensy 4.1

---

##  Contenu du module

```
teensy/
├── __init__.py
├── base_teensy.py      # Classe de base pour communication Teensy
├── gpio_teensy.py      # Gestion des GPIO étendus
└── tools/
    └── gpio_manager/   # Gestionnaire de pins et configuration
```

---

##  Classes principales

### `BaseTeensy` (base_teensy.py)

Classe de base pour la communication avec une Teensy via USB.

**Fonctionnalités :**
-  Connexion automatique via numéro de série USB
-  Reset logiciel de la Teensy
-  Gestion des callbacks de messages
-  Détection et reconnexion automatique

**Exemple d'utilisation :**

```python
from common.teensy.base_teensy import BaseTeensy
from common.usb_com.python.messages import Messages

# Connexion à la Teensy
teensy = BaseTeensy(serial_number=18421350, baudrate=115200)

# Envoyer une commande
teensy.send_message(Messages.SET_TARGET_POSITION, data)

# Définir un callback pour les retours
def on_position_update(data):
    print(f"Position reçue: {data}")

teensy.add_callback(on_position_update, Messages.UPDATE_ROLLING_BASIS.value)
```

---

### `GPIOTeensy` (gpio_teensy.py)

Gestion avancée des GPIO pour contrôle des actuateurs et lecture des capteurs.

**Fonctionnalités :**
-  Configuration des pins en entrée/sortie
-  Gestion PWM pour servomoteurs
-  Lecture d'état des switchs
-  Contrôle de drivers de moteurs pas-à-pas

**Exemple d'utilisation :**

```python
from common.teensy.gpio_teensy import GPIOTeensy

# Initialisation
gpio = GPIOTeensy(serial_number=18421350)

# Configurer un servo
gpio.set_servo_angle(pin=14, angle=90, max_angle=180)

# Lire un switch
state = gpio.read_switch(pin=22)

# Contrôler un stepper
gpio.step_motor(pin=8, steps=200, direction=True, speed=1000)
```

---

##  Dépendances

- `pyserial` : Communication USB série
- `common.usb_com.python` : Protocole de communication

---

##  Notes d'implémentation

### Reset de la Teensy

Le reset logiciel s'effectue via l'envoi du message `RESET_TEENSY (126)`. La Teensy redémarre automatiquement et la connexion est rétablie après ~2 secondes.

### Gestion des erreurs

Toutes les classes gèrent automatiquement :
- Les déconnexions USB (tentative de reconnexion)
- Les erreurs CRC (retransmission automatique)
- Les timeouts de communication

---

##  Tests

Des scripts de test sont disponibles dans `robot1/rasp/test/` :

```bash
# Test de connexion basique
python robot1/rasp/test/test_usb_detection.py

# Test de communication complète
python robot1/rasp/test/test_communication.py
```

---

## Voir aussi

- [Protocole de communication](../../PROTOCOLE_COMMUNICATION.md)
- [Documentation complète](../../Documentation_CDR_Nantes_2026.md)
- [Messages USB](../usb_com/python/README.md)
