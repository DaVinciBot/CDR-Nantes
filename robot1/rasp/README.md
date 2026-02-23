# Robot1 / Raspberry Pi - Code Python

## 🎯 Organisation des fichiers

### ⭐ Dossiers principaux

```
robot1/rasp/
├── utils/              # Utilitaires (robot_context, webots_com)
├── simu/               # Tests et outils simulation Webots
├── test/               # Tests hardware réel
├── test_lidar.py       # Test LIDAR rapide (accès direct)
├── switch_mode.py      # CLI: Bascule simulation/hardware
├── loader.py           # Chargement dynamique modules
└── config.json         # Configuration active
```

## 🚀 Démarrage rapide

### Test LIDAR (Webots)

```bash
cd robot1/rasp
python test_lidar.py
```

Récupère les 360 points LIDAR en tuples (angle, distance).

### Bascule simulation/hardware

```bash
# Passer en mode simulation
python switch_mode.py simulation

# Passer en mode hardware
python switch_mode.py hardware
```

## 📚 Imports recommandés

### Nouveau (depuis utils/)

```python
from utils import create_com, is_simulation

# Créer la connexion (auto-détection)
com = create_com()

# Vérifier le mode
if is_simulation():
    print("Mode simulation")
else:
    print("Mode hardware")
```

### Ancien (encore supporté)

```python
# Fonctionne encore mais utilise les fichiers dépréciés
from robot_context import create_com
```

## 📁 Détails des dossiers

### `utils/` - Utilitaires principaux

| Fichier | Description |
|---------|-------------|
| `robot_context.py` | Détection auto simulation/hardware + création Com |
| `webots_com.py` | Wrapper COM pour Webots (ports virtuels) |
| `__init__.py` | Exports simplifiés |

### `simu/` - Simulation Webots

| Fichier | Description |
|---------|-------------|
| `lidar_manager.py` | Gestionnaire LIDAR (360 points) |
| `test_lidar_simple.py` | Test LIDAR avec Stats |
| `test_lidar_raw.py` | Test LIDAR brut |

### `test/` - Tests hardware

Tests pour le robot réel (Teensy, capteurs, moteurs).

## 🔧 Configuration

### Fichiers de configuration

- **`config.json`** : Configuration active (créée par `switch_mode.py`)
- **`simu/config.json`** : Configuration spécifique simulation
- **`loader.py`** : Charge dynamiquement les modules selon config.json

### Exemple config.json (simulation)

```json
{
  "serial_config": {
    "port": "COM1",
    "baudrate": 115200
  }
}
```

## 📝 Exemples de code

### Test simple avec LIDAR

```python
from utils import create_com
from simu.lidar_manager import LidarManager
import time

# Connexion
com = create_com()
lidar = LidarManager(com)

# Attendre scan
while lidar.scans_received == 0:
    time.sleep(0.01)

# Utiliser les points
points = lidar.points  # [(0, dist0), (1, dist1), ...]
print(f"Points reçus: {len(points)}")
```

### Détection automatique du mode

```python
from utils import is_simulation, create_com

if is_simulation():
    print("🎮 Mode simulation Webots")
    com = create_com()  # Retourne WebotsComBridge
else:
    print("🤖 Mode hardware réel")
    com = create_com()  # Retourne Com (Teensy)
```

## ⚠️ Fichiers dépréciés

Les fichiers suivants dans `robot1/rasp/` sont obsolètes:
- `robot_context.py` → Utiliser `utils/robot_context.py`
- `webots_com.py` → Utiliser `utils/webots_com.py`

Des fichiers de redirection (`_deprecated_*.py`) maintiennent la compatibilité.

## 🔗 Voir aussi

- [utils/README.md](utils/README.md) - Documentation détaillée utils/
- [README_MODE_SWITCH.md](README_MODE_SWITCH.md) - Guide switch_mode
- [simu/LIDAR_QUICKSTART.md](simu/LIDAR_QUICKSTART.md) - Guide LIDAR
