# Simulation Webots - Organisation du projet

## Structure des fichiers

```
simulation/
├── .gitignore              # Exclusions Git (build/, __pycache__, etc.)
├── .simulation_mode        # Marqueur de mode simulation
├── config.json             # Configuration (port COM, baudrate, etc.)
├── loader.py               # Chargeur de modules
├── robot_context.py        # ⭐ Détection auto simulation/hardware
├── webots_com.py           # Classe Com pour ports COM virtuels
│
├── test_unified.py         # ⭐ Script unifié (simulation + hardware)
├── test_communication.py   # Script spécifique simulation
│
├── controllers/
│   └── teensy_controller/  # Contrôleur Webots C++
│       ├── teensy_controller.cpp
│       ├── fake_stepper.cpp
│       ├── Arduino.h
│       ├── ArduinoFake.h/cpp
│       ├── Makefile
│       └── build/         # Fichiers compilés (ignorés par Git)
│
├── worlds/                 # Mondes Webots (.wbt)
├── protos/                 # Prototypes de robots
└── base_roulante/         # Modèles CAD/URDF
```

## Utilisation

### 🎯 Mode automatique (recommandé)

Le script `test_unified.py` détecte automatiquement le contexte :

```bash
# En simulation (depuis simulation/)
cd simulation
python test_unified.py

# Sur le robot réel (depuis robot1/rasp/)
cd robot1/rasp
cp ../../simulation/test_unified.py .
python test_unified.py
```

**Détection automatique** :
- ✅ Fichier `.simulation_mode` présent → Mode SIMULATION
- ✅ Variable `ROBOT_MODE=simulation` → Mode SIMULATION
- ✅ Dossier contient "simulation" → Mode SIMULATION
- ❌ Sinon → Mode HARDWARE

### ⚙️ Configuration manuelle

Forcer le mode via variable d'environnement :

```bash
# Windows PowerShell
$env:ROBOT_MODE="simulation"
python test_unified.py

# Linux/Mac
ROBOT_MODE=simulation python test_unified.py
```

### 🔧 Configuration COM

**Simulation** : Éditer `config.json`
```json
{
    "serial_config": {
        "port": "COM1",
        "baudrate": 115200,
        "enable_crc": true
    }
}
```

**Hardware** : Configuration dans `robot1/rasp/config.json`
```json
{
    "serial_config": {
        "serial_number": 18421350,
        "vid": 5824,
        "pid": 1155,
        "baudrate": 115200
    }
}
```

## Modules clés

### robot_context.py

Détection automatique et création d'instances Com adaptées :

```python
from robot_context import is_simulation, create_com

# Détection
if is_simulation():
    print("Mode simulation Webots")
else:
    print("Mode hardware réel")

# Création automatique de Com
com = create_com(logger=logger)
```

### webots_com.py

Classe `WebotsComBridge` compatible avec l'API de `Com` mais pour ports COM virtuels :

```python
from webots_com import WebotsComBridge

com = WebotsComBridge(
    port='COM1',
    baudrate=115200,
    enable_crc=True,
    logger=logger
)

# API identique à Com
com.add_callback(callback, Messages.UPDATE_ROLLING_BASIS.value)
com.send_bytes(message)
```

## Prérequis

### Simulation
1. **Webots** installé (C:/App/Webots)
2. **Virtual Serial Port Tools** ou **com0com** (COM1 ↔ COM2)
3. Définir le chemin d'origine de webots dans webots_path.mk
3. **Python 3.x** avec `pyserial`, `crc8`

### Hardware
1. **Teensy 4.0/4.1** avec firmware flashé
2. **Câble USB** pour connexion
3. **Python 3.x** avec `pyserial`, `crc8`, `loggerplusplus`

## Workflow de développement

1. **Développer en simulation** :
   ```bash
   cd simulation
   # Lancer Webots avec teensy_controller
   python test_unified.py
   ```

2. **Tester sur hardware** :
   ```bash
   cd robot1/rasp
   # Copier le script unifié si nécessaire
   python test_unified.py
   ```

3. **Même code, contexte différent** ✨

## Dépannage

### Simulation
- ❌ "COM1 non disponible" → Vérifier Virtual Serial Port Tools (COM1↔COM2)
- ❌ "Aucun message reçu" → Vérifier console Webots "[Webots] ✅ COM2 connecté !"
- ❌ "CRC invalide" → Recompiler teensy_controller (Revert ⟲)

### Hardware
- ❌ "No Device found!" → Vérifier connexion USB Teensy
- ❌ "Aucun message reçu" → Reflasher firmware teensy_moteur
- ❌ VID/PID incorrect → Vérifier dans Device Manager (5824:1155)

## Fichiers ignorés par Git

`.gitignore` exclut automatiquement :
- `build/` et `*.exe` (artefacts de compilation)
- `__pycache__/` et `*.pyc` (cache Python)
- `.vs/` et `*.vcxproj.*` (fichiers IDE)
- `webots_path.mk` (généré automatiquement)

## Architecture du protocole

**Format des messages** (identique simulation et hardware) :
```
[Message] [Taille] [CRC8] [Signature: 0xBA 0xDD 0x1C 0xC5]
```

**Classes Com compatibles** :
- `Com` (robot1/rasp) : USB direct avec VID/PID Teensy
- `WebotsComBridge` (simulation/) : Port COM virtuel

**Même API** :
- `send_bytes(message)`
- `add_callback(callback, message_id)`
- Thread de réception automatique
- Validation CRC8

## Contribution

Pour ajouter un nouveau test :

1. Créer le script dans `simulation/`
2. Utiliser `robot_context.create_com()` pour la connexion
3. Le script fonctionnera automatiquement en simulation ET sur hardware

Exemple minimal :
```python
from robot_context import create_com
from loader import loader

Messages = loader.load_class('usb_com', 'Messages')
com = create_com()

# Votre code ici
msg = Messages.SET_TARGET_POSITION.to_bytes()
com.send_bytes(msg)
```

---

📝 **Dernière mise à jour** : Janvier 2026  
🤖 **Projet** : Robot holonome CDR-Nantes
