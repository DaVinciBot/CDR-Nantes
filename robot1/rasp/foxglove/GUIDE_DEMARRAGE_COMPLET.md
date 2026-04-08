# Foxglove Bridge — Guide de démarrage complet

## 🎯 Objectif

Afficher en **temps réel** dans Foxglove 4 positions du robot pour **monitoring et débogage**:

1. **🔴 ROBOT (ROUGE)** — Position fusionnée (calculée par Path Finding)
2. **🟢 TARGET (VERT)** — Position cible du Path Finding
3. **🔵 ODOM (BLEU)** — Odométrie brute du Teensy (pour debug)
4. **🟡 LIDAR (JAUNE)** — Position Lidar brute (pour debug)

**Important**: La fusion (Teensy Odom + Lidar + IMU) se fait dans votre **Path Finding/Navigator**, pas dans Foxglove.  
Foxglove est juste un **listener/visualiseur** pour le monitoring en temps réel.

```
PATH FINDING (fusion logique) → FOXGLOVE (monitoring)
```

---

## 📁 Structure des fichiers

```
robot1/rasp/foxglove/
├── foxglove_bridge_advanced.py      ← Bridge principal (classe FoxgloveBridgeAdvanced)
├── foxglove_3d_bridge.py            ← Version complète avec carte CDR (ancien)
├── foxglove_bridge.py               ← Version simple (ancien)
├── test_synthetic_advanced.py       ← Test sans hardware (4 cercles animés)
├── test_integration_advanced.py     ← Exemple intégration avec callbacks
├── ARCHITECTURE_GENERALE.md         ← Types de données + fusion détaillée
├── MESSAGES_A_AJOUTER.md           ← Messages USB manquants à créer
└── README_ADVANCED.md              ← Résumé rapide
```

---

## 🚀 Étape 1: Quick Test (5 min)

### Si vous avez juste Foxglove et Python:

```bash
cd c:\Users\Depot\CDR-Nantes\robot1\rasp\foxglove
python test_synthetic_advanced.py
```

Vous verrez:
- Terminal affichant les positions tous les 20 itérations
- 4 marqueurs animés dans Foxglove (cercles concentriques)
- Plots temporelles en bas

**✓ Si ça marche**: Foxglove et Python ✅  
**✗ Si erreur**: Lire DEBUG section plus bas

---

## 🚀 Étape 2: Intégration réelle (30 min)

### Architecture

```
                         VOS CAPTEURS
                    (Teensy, Lidar, IMU)
                             ↓
                        PATH FINDING
                      (votre algorithme
                    de navigation/fusion)
                             ↓
                      FOXGLOVE (optionnel)
                    (affichage en temps réel)
```

### Prérequis:
1. ✅ Messages USB de base (UPDATE_ROLLING_BASIS) — existants
2. ✅ Nouveaux messages à créer dans messages.py:
   - `LIDAR_POSITION` (134)
   - `IMU_ANGLE` (135)
   - `TARGET_POSITION` (136)

### À faire:

#### 2a. Créer votre Navigator (logique de fusion)
```python
# robot1/rasp/navigator.py (exemple simplifié)

class Navigator:
    def __init__(self, bridge=None):
        self.bridge = bridge  # Référence à Foxglove (optionnel)
        self.odom = (0.0, 0.0)
        self.lidar = (0.0, 0.0)
        self.imu_theta = 0.0
        self.confidence = 0.0
        self.target = (0.0, 0.0)
    
    def update_sensors(self, x_odom, y_odom, x_lidar, y_lidar, conf, theta_imu):
        """Reçoit les données brutes des capteurs."""
        self.odom = (x_odom, y_odom)
        self.lidar = (x_lidar, y_lidar)
        self.confidence = conf
        self.imu_theta = theta_imu
    
    def fuse_position(self):
        """Fusion réelle (où la logique se fait!)."""
        if self.confidence > 0.2:
            # Lidar valide: 60% Lidar + 40% Odom
            x = 0.6 * self.lidar[0] + 0.4 * self.odom[0]
            y = 0.6 * self.lidar[1] + 0.4 * self.odom[1]
        else:
            # Lidar absent: utiliser Odom seul
            x = self.odom[0]
            y = self.odom[1]
        
        return (x, y, self.imu_theta)  # ← IMU pour l'angle!
    
    def navigate(self):
        """Logique navigation (utiliser position fusionnée)."""
        x, y, theta = self.fuse_position()
        
        # Calculs de trajectoire, PID, etc.
        dx = self.target[0] - x
        dy = self.target[1] - y
        distance = hypot(dx, dy)
        
        # ... envoyer commandes au Teensy ...
        
        # Optionnel: publier pour monitoring
        if self.bridge:
            self.bridge.publish_data(
                target_x=self.target[0],
                target_y=self.target[1],
                odom_x=self.odom[0],
                odom_y=self.odom[1],
                lidar_x=self.lidar[0],
                lidar_y=self.lidar[1],
                lidar_confidence=self.confidence,
                imu_theta=self.imu_theta
            )
```

#### 2b. Créer les messages manquants (common/usb_com/python/messages.py)
```python
class Messages(Enum):
    # Existants
    UPDATE_ROLLING_BASIS = 128
    
    # À ajouter
    LIDAR_POSITION = 134
    """Position calculée par Lidar (balises)."""
    
    IMU_ANGLE = 135
    """Angle IMU (gyro + accel)."""
    
    TARGET_POSITION = 136
    """Position cible du Path Finding."""
```

#### 2c. Enregistrer les callbacks et lancer
```python
# robot1/rasp/mon_script_principal.py

from foxglove.foxglove_bridge_advanced import FoxgloveBridgeAdvanced
from navigator import Navigator

# Initialiser le bridge (optionnel, pour monitoring)
bridge = FoxgloveBridgeAdvanced(port=8765).start()

# Créer le Navigator (où la fusion se fait)
navigator = Navigator(bridge=bridge)

# Callbacks depuis COM
def on_rolling_basis(data: bytes):
    if len(data) >= 24:
        x, y, _ = struct.unpack("<ddd", data[:24])
        navigator.update_sensors(x, y, ...)

def on_lidar(data: bytes):
    if len(data) >= 20:
        x, y, conf = struct.unpack("<ddf", data[:20])
        navigator.update_sensors(..., x, y, conf, ...)

def on_imu(data: bytes):
    if len(data) >= 8:
        theta, = struct.unpack("<d", data[:8])
        navigator.update_sensors(..., theta)

com.add_callback(on_rolling_basis, Messages.UPDATE_ROLLING_BASIS.value)
com.add_callback(on_lidar, Messages.LIDAR_POSITION.value)
com.add_callback(on_imu, Messages.IMU_ANGLE.value)

# Boucle principale
while True:
    navigator.navigate()  # ← Fusion + commandes
    time.sleep(0.05)
```

#### 2d. Vérifier dans Foxglove
- ✓ 4 marqueurs visibles
- ✓ Robot rouge = position que Navigator calcule
- ✓ Cible, Odom, Lidar = pour debug

---

## 📊 Données attendues

### Tous les X, Y sont en millimètres (mm)
```python
# Terrain CDR 2026
# (0, 0)      bas-gauche
# (3000, 2000) haut-droite

Position exemple: (1500, 1000) = centre
```

### Angle θ en radians (rad)
```python
θ = 0.0      # Est (direction X+)
θ = π/2      # Nord (direction Y+)
θ = π        # Ouest (direction X-)
θ = -π/2     # Sud (direction Y-)

# Toboggan: θ = 0.3 à 0.5 rad (17-29°)
```

### Confiance Lidar: 0.0 à 1.0
```python
confidence = 0.85  # Bien détecté (seules 2-3 balises)
confidence = 0.5   # Moyen (données noisy)
confidence = 0.0   # Absent (pas de balise visible)
```

---

## 🔍 Flux de données (schéma simplifié)

```
┌─────────────────┐
│ Teensy (USB)    │ ──→ UPDATE_ROLLING_BASIS (x_odom, y_odom, θ_brut)
└─────────────────┘

┌─────────────────┐
│ Lidar (GPIO)    │ ──→ LIDAR_POSITION (x_lidar, y_lidar, confidence)
└─────────────────┘
                           ↓ [Python Rasp]
                    ┌──────────────────────────────┐
                    │ SensorFusionManager          │
                    │ (thread-safe storage)        │
                    │                              │
                    │ • update_odom()              │
                    │ • update_lidar()             │
                    │ • update_imu()               │
                    │ • update_target()            │
                    │                              │
                    │ fuse_position() →            │
                    │   x_fused, y_fused, θ_imu    │
                    └──────────────────────────────┘
                             ↓
┌─────────────────┐      ┌─────────────────┐
│ IMU (Teensy)    │ ──→  │ Foxglove Server │
│                 │      │ ws://8765       │
│ TARGET (Rasp)   │ ──→  │                 │
│                 │      │ • robot/pose    │
└─────────────────┘      │ • robot/tf      │
                         │ • robot/scene   │
                         └─────────────────┘
                              ↓
                         📱 FOXGLOVE
                         (4 marqueurs)
```

---

## 🐛 Débogage

### Test 1: Python importe correctement
```bash
cd robot1/rasp/foxglove
python -c "from foxglove_bridge_advanced import FoxgloveBridgeAdvanced; print('✓ OK')"
```

### Test 2: Server Foxglove démarre
```bash
python -c "
from foxglove_bridge_advanced import FoxgloveBridgeAdvanced
b = FoxgloveBridgeAdvanced()
b.start()
import time; time.sleep(2)
b.stop()
print('✓ Server lancé et arrêté')
"
```

### Test 3: Données arrivent à Foxglove
```bash
python test_synthetic_advanced.py
# Doit afficher:
# 2025-01-08 14:23:45 ... | foxglove_bridge_advanced | Foxglove prêt sur ws://localhost:8765
# [  0] Target(2300,...) | Odom(...) | Lidar(...) | Fused(...) | θ_imu=...
```

### Test 4: Vérifier les logs détaillés
```python
import logging
logging.basicConfig(level=logging.DEBUG)
# Verra plus de détails sur ce qui se passe
```

---

## ⚠️ Erreurs courantes

### 1. "foxglove_websocket non installé"
```bash
pip install foxglove-websocket
```

### 2. "Port 8765 déjà utilisé"
```bash
# Tuer les antennes Foxglove existantes
netstat -ano | findstr :8765
taskkill /PID <PID> /F

# Ou utiliser un autre port
bridge = FoxgloveBridgeAdvanced(port=8766)
```

### 3. "struct.unpack failed"
Format des bytes incorrect. Vérifier:
- Taille du message
- Ordre little-endian (`<`)
- Types (ddd = 3 doubles)

### 4. "Bridge démarré mais aucun marqueur"
Vérifier que Foxglove:
- ✓ Est connecté à ws://localhost:8765
- ✓ A un panel "3D" affiché
- ✓ A le bon frame "world"

---

## ✅ Vérifier l'intégration

Après avoir lancé `test_integration_advanced.py`:

```python
# Dans le script ou terminal Python:
from foxglove.foxglove_bridge_advanced import FoxgloveBridgeAdvanced

bridge = FoxgloveBridgeAdvanced()
bridge.start()

# Publier quelques données
for i in range(10):
    bridge.publish_data(
        target_x=2000, target_y=1000,
        odom_x=1500 + i*10, odom_y=1000,
        lidar_x=1510 + i*10, lidar_y=1005,
        lidar_confidence=0.85,
        imu_theta=0.1
    )
    time.sleep(0.1)

# Vérifier la fusion
snap = bridge._fusion.get_snapshot()
print(f"Odom: {snap['odom']}")
print(f"Lidar: {snap['lidar']}")
print(f"Fused: {bridge._fusion.fuse_position()}")

bridge.stop()
```

---

## 📈 Prochaines étapes avancées

Once everything works:

1. **Kalman Filter** (optimal)
   - Voir `ARCHITECTURE_GENERALE.md` section "Stratégie 3"
   - Nécessite: scipy, numpy

2. **Détection divergence**
   - Si |Lidar - Odom| > 150mm → alarme collision
   - Code exemple fourni

3. **Logging de debug**
   - Enregistrer les 4 positions dans un fichier .csv
   - Pour analyse post-run

4. **Détection obstacle Lidar**
   - Afficher drone scan points
   - Détecter robot adverse

5. **Customization couleurs/tailles**
   - Modifier `ROBOT_COLOR`, `MARKER_COLORS` en haut des fichiers

---

## 📞 Fichiers à consulter

| Situation | Fichier |
|-----------|---------|
| "Comment ça fonctionne?" | `ARCHITECTURE_GENERALE.md` |
| "Quels messages ajouter?" | `MESSAGES_A_AJOUTER.md` |
| "Code complet du bridge" | `foxglove_bridge_advanced.py` |
| "Exemple d'intégration" | `test_integration_advanced.py` |
| "Tester sans hardware" | `test_synthetic_advanced.py` |
| "Résumé rapide" | `README_ADVANCED.md` |

---

## ✨ Résumé

```
┌─ Path Finding (fusion logique) ✅
│
├─ Teensy Odom + Lidar + IMU fusionnés
│
├─ L'angle TOUJOURS de l'IMU (pas de dérive)
│
├─ Foxglove = monitoring/affichage UNIQUEMENT
│  (pas la logique de fusion)
│
└─ 4 marqueurs pour debug en temps réel

Status: ✅ Architecture claire et séparation des responsabilités
         ✅ Fusion dans Path Finding
         ✅ Visualisation dans Foxglove (optionnel)
```

---

**Version**: 2.2  
**State**: Production-ready (Foxglove = viewer, fusion = Path Finding)  
**Auteur**: Architecture Foxglove Advanced

