# Rerun Bridge — Guide de démarrage complet

## 🎯 Objectif

Afficher en **temps réel** dans Rerun **positions historisées** du robot pour **monitoring et débogage avec timeline**:

1. **🔵 BLEU** — Odométrie Teensy brute → `world/robot/odom`
2. **🟢 VERT** — Position fusionnée (calculée par Path Finding) → `world/robot/fused`
3. **🔴 ROUGE** — Position Lidar (trilatération) → `world/lidar/pose`
4. **🟡 ORANGE** — Balises détectées par Lidar → `world/lidar/beacons_detected`

**Important**: La fusion (Teensy Odom + Lidar + IMU) se fait dans votre **Path Finding/Navigator**, pas dans Rerun.  
Rerun est un **visualiseur SDK** pour le monitoring temps réel avec historique + timeline interactive.

```
PATH FINDING (fusion logique) → RERUN (monitoring + timeline)
```

---

## 📁 Structure des fichiers

```
robot1/rasp/
├── rerun/
│   ├── rerun_bridge.py              ← Bridge principal Rerun
│   ├── ARCHITECTURE_GENERALE.md     ← Types de données + fusion détaillée
│   ├── GUIDE_DEMARRAGE_COMPLET.md   ← Ce fichier
│   └── README_ADVANCED.md           ← Résumé rapide
├── lidar/
│   ├── main.py                      ← GUI lidar (logique active)
│   └── launch_rerun.py              ← Launcher vers rerun_bridge.py
└── [autres modules]
```

---

## 🚀 Étape 1: Quick Test (5 min)

### Rerun local avec simulation

```bash
cd e:\CDR\CDR-Nantes\robot1\rasp
python rerun/rerun_bridge.py --mode local --sim
```

**Résultat**:
- Viewer Rerun s'ouvre automatiquement
- 4 robots animés (bleu, vert, rouge, marqueurs)
- Nuage Lidar synthétique
- Timeline en bas calculée automatiquement
- 3 graphiques temporels: Po, Lidar, Fusion

**✓ Si ça marche**: Rerun SDK ✅  
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
                    RERUN SDK
                  (affichage historisé
                   + timeline interactive)
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
    def __init__(self):
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
        import math
        x, y, theta = self.fuse_position()
        
        # Calculs de trajectoire, PID, etc.
        dx = self.target[0] - x
        dy = self.target[1] - y
        distance = math.hypot(dx, dy)
        
        # ... envoyer commandes au Teensy ...
        
        # Publier pour monitoring Rerun
        import rerun as rr
        rr.log("world/robot/odom", rr.Boxes3D(
            centers=[[self.odom[0], self.odom[1], 50]],
            half_sizes=[[150, 150, 50]],
            colors=[[51, 153, 255, 220]]  # Bleu
        ))
        
        rr.log("world/robot/fused", rr.Boxes3D(
            centers=[[x, y, 50]],
            half_sizes=[[150, 150, 50]],
            colors=[[80, 255, 120, 220]]  # Vert
        ))
        
        if self.confidence > 0:
            rr.log("world/lidar/pose", rr.Boxes3D(
                centers=[[self.lidar[0], self.lidar[1], 50]],
                half_sizes=[[150, 150, 50]],
                colors=[[255, 80, 80, 220]]  # Rouge
            ))
        
        # Courbes temporelles
        rr.log("data/fused/x_mm", rr.Scalars(x))
        rr.log("data/fused/y_mm", rr.Scalars(y))
        rr.log("data/fusion/confidence", rr.Scalars(self.confidence))
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

import rerun as rr
from navigator import Navigator

# Initialiser Rerun
rr.init("eurobot_2026", spawn=True)

# Créer le Navigator (où la fusion se fait)
navigator = Navigator()

# Callbacks depuis COM
def on_rolling_basis(data: bytes):
    if len(data) >= 24:
        import struct
        x, y, _ = struct.unpack("<ddd", data[:24])
        navigator.update_sensors(x, y, ...)

def on_lidar(data: bytes):
    if len(data) >= 20:
        import struct
        x, y, conf = struct.unpack("<ddf", data[:20])
        navigator.update_sensors(..., x, y, conf, ...)

def on_imu(data: bytes):
    if len(data) >= 8:
        import struct
        theta, = struct.unpack("<d", data[:8])
        navigator.update_sensors(..., theta)

com.add_callback(on_rolling_basis, Messages.UPDATE_ROLLING_BASIS.value)
com.add_callback(on_lidar, Messages.LIDAR_POSITION.value)
com.add_callback(on_imu, Messages.IMU_ANGLE.value)

# Boucle principale
while True:
    navigator.navigate()  # ← Fusion + commandes + Rerun publish
    time.sleep(0.05)
```

#### 2d. Vérifier dans Rerun Viewer
- ✓ 3 robots visibles (bleu, vert, rouge)
- ✓ Robot vert = position que Navigator calcule (fusion)
- ✓ Bleu = Odom brut, Rouge = Lidar brut (pour debug)
- ✓ Timeline en bas se construit automatiquement

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
confidence = 0.85  # Bien détecté (2-3 balises visibles)
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
                    │ Navigator                    │
                    │ (fusion logique)             │
                    │                              │
                    │ • update_odom()              │
                    │ • update_lidar()             │
                    │ • update_imu()               │
                    │ • fuse_position()            │
                    │ • navigate()                 │
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
cd robot1/rasp
python -c "import rerun as rr; print('✓ Rerun OK')"
python -c "from rerun_bridge import *; print('✓ Bridge OK')"
```

### Test 2: Rerun SDK démarre en local
```bash
python rerun/rerun_bridge.py --mode local --sim
# Viewer doit s'ouvrir automatiquement
# Esc pour quitter
```

### Test 3: Vérifier les données publiées
```bash
python rerun/rerun_bridge.py --mode local --with-lidar
# Doit afficher:
# callbacks odometrie Teensy
# donnees lidar et confiance
# publication continue a 20 Hz
```

### Test 4: Vérifier les logs détaillés
```python
import logging
logging.basicConfig(level=logging.DEBUG)
# Verra plus de détails sur ce qui se passe
```

---

## ⚠️ Erreurs courantes

### 1. "rerun-sdk non installé"
```bash
pip install rerun-sdk
```

### 2. "Pas d'accès au hardware (Teensy, Lidar)"
```bash
# Utiliser --sim pour tester avec simulation
python rerun/rerun_bridge.py --mode local --sim
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

