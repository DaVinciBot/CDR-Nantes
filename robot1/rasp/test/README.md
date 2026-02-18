#  Scripts de Test - Raspberry Pi

> Suite de tests pour validation hardware et communication USB

---

##  Liste des scripts

###  Tests de communication

#### `test_usb_detection.py`
**Objectif :** Détecter automatiquement les Teensy connectées via USB

```bash
python test_usb_detection.py
```

**Sortie attendue :**
```
 Recherche de périphériques USB Teensy...
 Teensy 4.1 détectée :
   - Port : COM5
   - Serial Number : 18421350
   - VID/PID : 5824/1155
```

---

#### `test_communication.py`
**Objectif :** Test complet du protocole de communication (envoi/réception)

```bash
python test_communication.py
```

**Tests effectués :**
-  Connexion USB établie
-  Envoi de `SET_TARGET_POSITION`
-  Réception de `UPDATE_ROLLING_BASIS`
-  Validation CRC8
-  Latence de communication (<10ms)

**Sortie attendue :**
```
[INFO] Connexion établie sur COM5
[INFO] Message envoyé : SET_TARGET_POSITION
[INFO] Réponse reçue : UPDATE_ROLLING_BASIS
[INFO] Position: X=123.5mm Y=67.8mm θ=1.57rad
[SUCCESS] Tous les tests passés ✓
```

---

#### `test_serial_raw.py`
**Objectif :** Test brut de la communication série (debug bas niveau)

```bash
python test_serial_raw.py
```

Affiche les bytes bruts envoyés/reçus en hexadécimal. Utile pour diagnostiquer les problèmes de protocole.

---

###  Tests de mouvement

#### `test_one_motor.py`
**Objectif :** Tester un moteur individuel

```bash
python test_one_motor.py --motor 1 --steps 1000
```

**Options :**
- `--motor` : Numéro du moteur (1, 2 ou 3)
- `--steps` : Nombre de pas à effectuer
- `--speed` : Vitesse en steps/s (défaut: 1000)

**Sortie attendue :**
```
[INFO] Test du moteur 1
[INFO] Commande : 1000 steps à 1000 steps/s
[INFO] Encodeur avant : 0
[INFO] Encodeur après : 1000
[SUCCESS] Moteur 1 fonctionne correctement ✓
```

---

###  Tests de messages

#### `test_length_messages.py`
**Objectif :** Vérifier la taille des payloads pour chaque type de message

```bash
python test_length_messages.py
```

**Sortie attendue :**
```
Message : SET_TARGET_POSITION
  Payload attendu : 24 bytes (3 × double)
  Payload reçu    : 24 bytes
  ✓ Taille correcte

Message : SET_PID
  Payload attendu : 13 bytes (1 × byte + 3 × float)
  Payload reçu    : 13 bytes
  ✓ Taille correcte

...
```

---

#### `test_debug_messages.py`
**Objectif :** Afficher tous les messages reçus de la Teensy

```bash
python test_debug_messages.py --duration 30
```

**Options :**
- `--duration` : Durée d'écoute en secondes (défaut: 10)
- `--filter` : Filtrer par type de message (ex: `UPDATE_ROLLING_BASIS`)

**Sortie attendue :**
```
[17:30:15.123] UPDATE_ROLLING_BASIS: X=0.0 Y=0.0 θ=0.0
[17:30:15.223] UPDATE_ROLLING_BASIS: X=1.2 Y=0.1 θ=0.01
[17:30:15.323] SWITCH_STATE_RETURN: Pin=22 State=HIGH
...
```

---

##  Tests rapides

### Test complet du système

```bash
# 1. Vérifier la connexion
python test_usb_detection.py

# 2. Tester la communication
python test_communication.py

# 3. Tester un mouvement simple
cd ..
python test_simple_traj.py
```

### Diagnostic en cas de problème

```bash
# Afficher les ports disponibles
python test_usb_detection.py --list-all

# Test brut de la liaison série
python test_serial_raw.py

# Vérifier les messages reçus
python test_debug_messages.py --duration 5
```

---

##  Checklist de validation

Avant de tester une nouvelle feature, exécutez dans l'ordre :

- [ ] `test_usb_detection.py` → Teensy détectée
- [ ] `test_communication.py` → Communication OK
- [ ] `test_length_messages.py` → Tailles de messages valides
- [ ] `test_one_motor.py` (pour chaque moteur) → Moteurs fonctionnels
- [ ] `test_simple_traj.py` → Déplacement complet

---

##  Configuration

Les scripts utilisent automatiquement `config.json` pour détecter :
- Le mode (simulation vs hardware)
- Le port série
- Le numéro de série de la Teensy

### Forcer le mode hardware

```bash
export ROBOT_MODE=hardware
python test_communication.py
```

### Forcer le mode simulation

```bash
export ROBOT_MODE=simulation
python test_communication.py
```

---

##  Dépannage

### Problème : "Port série introuvable"

```bash
# Vérifier les ports disponibles
python test_usb_detection.py --list-all

# Vérifier les permissions (Linux)
sudo usermod -a -G dialout $USER
# Puis redémarrer la session
```

### Problème : "CRC invalide"

```bash
# Tester avec CRC désactivé (debug uniquement)
python test_communication.py --no-crc
```

### Problème : "Timeout de réception"

```bash
# Vérifier que la Teensy répond
python test_serial_raw.py

# Augmenter le timeout
python test_communication.py --timeout 5000  # 5 secondes
```

---

##  Ajouter un nouveau test

```python
#!/usr/bin/env python3
"""
Description du test
"""

import sys
from pathlib import Path

# Ajouter le chemin vers common/
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from common.usb_com.python.com.com import Com
from common.usb_com.python.messages import Messages
from robot1.rasp.loader import loader

def main():
    # Initialiser la communication
    com, mode = init_robot(logger)
    
    # Votre test ici
    print("Test en cours...")
    
    # Envoyer un message
    com.send_bytes(Messages.SET_TARGET_POSITION.to_bytes())
    
    print("✓ Test réussi")

if __name__ == "__main__":
    main()
```

---

##  Voir aussi

- [Guide de démarrage rapide](../../../Documentation_CDR_Nantes_2026.md#12-guide-de-démarrage-rapide)
- [Protocole de communication](../../../PROTOCOLE_COMMUNICATION.md)
- [Module USB Com Python](../../../common/usb_com/python/README.md)
