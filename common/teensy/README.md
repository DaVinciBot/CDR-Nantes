# Module Teensy - Classes Python

> Deux spécialisations de `usb_com.Com` pour parler à une Teensy 4.1. Le protocole lui-même
> (trames, messages, callbacks) est décrit dans [../usb_com/python/](../usb_com/python/README.md).

---

## Contenu du module

```text
teensy/
├── __init__.py
├── base_teensy.py      # BaseComTeensy : Com + reset
├── gpio_teensy.py      # GPIOComTeensy : Com + suivi des GPIO attribuées
└── tools/
    └── gpio_manager/   # GPIOManager : quelle pin sert à quel actionneur
```

---

## `BaseComTeensy` (base_teensy.py)

Un `Com` qui sait redémarrer la Teensy : `reset()` envoie `RESET_TEENSY` (126). Constructeur,
envoi et callbacks sont ceux de `Com`.

```python
from teensy.base_teensy import BaseComTeensy
from usb_com import Messages, pack_message

teensy = BaseComTeensy(logger, 17795370, 5824, 1155, 115200)

teensy.send_bytes(pack_message(Messages.SET_TARGET, 200.0, 0.0, 0.0, 0.0, 0.0, 0.0))

def on_odometry(data: bytes) -> None:
    print(f"Odométrie reçue : {len(data)} octets")

teensy.add_callback(on_odometry, Messages.ODOMETRY.value)
teensy.reset()
```

Côté `robot1/rasp/`, la connexion se fait par `robot1.rasp.comm.init_robot()`, qui lit
`config.json`.

---

## `GPIOComTeensy` (gpio_teensy.py)

Un `Com` qui porte un `GPIOManager` (41 pins) pour tenir la liste des GPIO déclarées et de leur
type d'actionneur, avec les pins I2C fixées (SDA 18, SCL 19). Les commandes d'actionneurs
partent par `send_bytes()`, avec les messages 3 à 8 de `usb_com`.

---

## Dépendances

- `pyserial` : port série
- `usb_com` : protocole (`pip install -e .` à la racine du dépôt)

---

## Notes

- Après `RESET_TEENSY`, la Teensy redémarre et le port USB réapparaît. `Com` ne se reconnecte
  pas tout seul : recréer l'objet.
- Une trame reçue invalide est ignorée et la Teensy reçoit un `NACK`. Sur un `NACK` reçu, `Com`
  renvoie son dernier message.

---

## Tests

Scripts de bring-up face au robot, depuis `robot1/rasp/` :

```bash
python -m tools.bringup.test_usb_detection
python -m tools.bringup.test_communication
```

Voir [../../robot1/rasp/tools/bringup/README.md](../../robot1/rasp/tools/bringup/README.md).
