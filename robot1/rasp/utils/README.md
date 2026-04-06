# Organisation des fichiers robot1/rasp/

## Structure

```
robot1/rasp/
├── utils/                  # ⭐ Utilitaires principaux
│   ├── __init__.py        # Imports simplifiés
│   ├── robot_context.py   # Détection auto simulation/hardware
│   ├── webots_com.py      # Communication Webots
│   └── README.md
├── lidar/                  # Module LIDAR (GUI + logique + bridge navigation)
├── simu/                   # Tests simulation
│   ├── lidar_manager.py
│   ├── test_lidar_simple.py
│   └── ...
├── test/                   # Tests hardware
│   └── ...
├── switch_mode.py          # Bascule simulation/hardware (standalone)
├── loader.py               # Chargement dynamique modules
├── config.json             # Configuration
├── robot_context.py        # ⚠️  OBSOLÈTE - Voir utils/robot_context.py
└── webots_com.py           # ⚠️  OBSOLÈTE - Voir utils/webots_com.py
```

## Utilisation

### Import recommandé (depuis utils/)

```python
# Nouveau - Recommandé
from utils import create_com, is_simulation

com = create_com()
if is_simulation():
    print("Mode simulation")
```

### Import ancien (encore supporté)

```python
# Ancien - Fonctionnel mais déprécié
from robot_context import create_com
```

### Test LIDAR

```bash
# Depuis robot1/rasp/
python -m lidar.main
```

## Fichiers dans utils/

| Fichier | Description |
|---------|-------------|
| `robot_context.py` | Détection automatique simulation/hardware + création Com |
| `webots_com.py` | Wrapper COM pour simulation Webots |
| `__init__.py` | Exports pour imports simplifiés |

## switch_mode.py

Reste dans `robot1/rasp/` car c'est un outil autonome (script CLI):

```bash
python switch_mode.py simulation
python switch_mode.py hardware
```

## Migration

Pour migrer votre code existant:

**Avant:**
```python
from robot_context import create_com
```

**Après:**
```python
from utils import create_com
```

Les anciens fichiers `robot_context.py` et `webots_com.py` dans `robot1/rasp/` peuvent être conservés pour compatibilité ou supprimés une fois la migration terminée.

