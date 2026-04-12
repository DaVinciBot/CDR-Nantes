# Architecture Rerun — Logique générale

## ⚠️ IMPORTANT: Rerun = Visualisation + Timeline

**Rerun n'effectue PAS la fusion réelle.**

La fusion (Teensy Odom + Lidar + IMU) se fait dans le **Path Finding**,  
Rerun reçoit juste les données finalisées pour l'affichage/monitoring/replay temps réel et historique.

---

## Vue d'ensemble complète

```
┌───────────────────────────────────────────────────────────────────┐
│                      SOURCES CAPTEURS                             │
├────────────────────┬────────────────────┬────────────────────┐   │
│ UPDATE_ROLLING     │  LIDAR_DATA        │  IMU_ANGLE         │   │
│ BASIS              │                    │                    │   │
│ (Teensy)           │  (Balises)         │  (Accéléro/Gyro)   │   │
│                    │                    │                    │   │
│ x_odom [mm]        │  x_lidar [mm]      │  θ_imu [rad]       │   │
│ y_odom [mm]        │  y_lidar [mm]      │  (SOURCE UNIQUE!)  │   │
│ θ_brut [rad]       │  confidence [0-1]  │                    │   │
│ ✗ (ignoré)         │                    │                    │   │
└────────────────────┴────────────────────┴────────────────────┘   │
                             ↓                                       │
└───────────────────────────────────────────────────────────────────┘
                             ↓
         ┌─────────────────────────────────────┐
         │    PATH FINDING (Fusion Réelle)     │  ← LA LOGIQUE EST ICI!
         │                                     │
         │  • Fusion odométrie + Lidar         │
         │  • Angle toujours IMU               │
         │  • Détection divergence/collision   │
         │  • Décisions de navigation          │
         │  • Sortie: position fusionnée       │
         │            + target atteint?        │
         │            + obstacles détectés?    │
         │                                     │
         │  Stocke: (x_fused, y_fused, θ_imu) │
         └─────────────────────────────────────┘
                             ↓
            ┌────────────────┴────────────────┐
            ↓                                  ↓
    [Navigation/Contrôle]           [Rerun Visualization]
    (commandes Teensy)              (monitoring + timeline)
                                             ↓
                                    ┌──────────────────────┐
                                    │  Rerun Logger        │
                                    │  (rr.log())          │
                                    │                      │
                                    │ Enregistre:          │
                                    │ • world/robot/odom   │
                                    │ • world/robot/fused  │
                                    │ • world/lidar/pose   │
                                    │ • data/fused/*       │
                                    │ • data/lidar/*       │
                                    └──────────────────────┘
                                             ↓
                                       ┌─────────────────┐
                                       │ Foxglove 3D │
                                       │ (WS 8765)   │
                                       └─────────────┘
                                             ↓
           ┌───────────┬──────────┬──────────┬──────────┐
           ↓           ↓          ↓          ↓          ↓
        ROBOT (RED) TARGET(GRN) ODOM(BLUE) LIDAR(YEL) PLOT
        + Flèche     Marqueur   Marqueur   Marqueur   (courbes)
        (fused)      (cible)    (brute)    (brute)    + confiance
        
        ↑ Affichage temps réel du que le Path Finding utilise
```

## Types de données génériques

### 1. **Position2D** (toujours en mm)
```python
@dataclass
class Position2D:
    x: float = 0.0    # mm (repère absolu, bas-gauche = 0,0)
    y: float = 0.0    # mm
    
    def to_m(self) -> tuple[float, float]:
        return (x/1000, y/1000)  # Pour Foxglove 3D
```

### 2. **Orientation1D** (radians)
```python
@dataclass
class Orientation1D:
    theta: float = 0.0  # rad, provient TOUJOURS de l'IMU
```

### 3. **SensorData** (données complètes + métadonnées)
```python
@dataclass
class SensorData:
    position: Position2D          # x, y en mm
    theta: float = 0.0            # rad (optionnel, peut être ignoré)
    timestamp: float = 0.0        # ns
    confidence: float = 1.0       # 0.0-1.0 (pertinence des données)
    source_name: str = "unknown"  # Debug/logs
    
    # Exemple Lidar:
    # SensorData(
    #     position=Position2D(x=1500, y=1000),
    #     theta=0.15,          # Ignoré (on utilise IMU)
    #     confidence=0.85,     # Bien détecté
    #     source_name="lidar"
    # )
```

## Règles de fusion

### ⚠️ LA FUSION SE FAIT DANS LE PATH FINDING, PAS DANS FOXGLOVE

La logique de fusion doit être implémentée dans votre module de **Path Finding**:

```python
# Dans robot1/rasp/pathfinder.py (ou navigation.py)

class Navigator:
    def __init__(self):
        self.odom = (0.0, 0.0)      # Teensy
        self.lidar = (0.0, 0.0)
        self.imu_theta = 0.0
        self.confidence_lidar = 0.0
    
    def update_odom(self, x, y, theta_ignored):
        self.odom = (x, y)
    
    def update_lidar(self, x, y, conf):
        self.lidar = (x, y)
        self.confidence_lidar = conf
    
    def update_imu(self, theta):
        self.imu_theta = theta  # ← SEULE source fiable pour θ
    
    def compute_fused_position(self):
        """Fusion réelle (moyenne pondérée simple)."""
        if self.confidence_lidar > 0.2:
            # Lidar valide
            x_fused = 0.6 * self.lidar[0] + 0.4 * self.odom[0]
            y_fused = 0.6 * self.lidar[1] + 0.4 * self.odom[1]
        else:
            # Lidar absent
            x_fused = self.odom[0]
            y_fused = self.odom[1]
        
        return (x_fused, y_fused, self.imu_theta)
    
    def navigate(self, target_x, target_y):
        """Utilise la position fusionnée pour naviguer."""
        x_fused, y_fused, theta = self.compute_fused_position()
        
        # TODO: calcul trajectoire, détection obstacles, etc.
        # distance = hypot(target_x - x_fused, target_y - y_fused)
        # angle_to_target = atan2(target_y - y_fused, target_x - x_fused)
        # error_theta = normalize_angle(angle_to_target - theta)
        # ...
```

Foxglove **reçoit** cette fusion finalisée pour l'affichage, mais ne la calcule pas.

### ✅ L'angle θ vient TOUJOURS de l'IMU

**Raison**: L'IMU (accéléromètre + gyroscope) est plus stable et moins sujette à la dérive que l'odométrie moteur.

```python
# CORRECT: Fusion dans Path Finding
path_finder = Navigator()
path_finder.update_odom(x_odom, y_odom, theta_brut_ignored)
path_finder.update_lidar(x_lidar, y_lidar, confidence)
path_finder.update_imu(theta_imu)

x_fused, y_fused, theta = path_finder.compute_fused_position()

# Puis publier pour affichage Foxglove:
bridge.publish_data(
    target_x=target.x, target_y=target.y,
    odom_x=x_odom, odom_y=y_odom,          # données brutes
    lidar_x=x_lidar, lidar_y=y_lidar,      # données brutes
    lidar_confidence=confidence,
    imu_theta=theta_imu,
    # ↑ La FUSION s'est déjà faite dans Path Finding
    # ↓ Foxglove affiche juste pour le monitoring
)
```

### 📍 Fusion de position (dans Path Finding)

#### Stratégie 1: Moyenne pondérée simple (rapide)
```python
def fuse_simple(odom_x, odom_y, lidar_x, lidar_y, confidence):
    if confidence > 0.2:
        # Lidar valide: faire confiance à 60%
        x = 0.6 * lidar_x + 0.4 * odom_x
        y = 0.6 * lidar_y + 0.4 * odom_y
    else:
        # Lidar absent: utiliser odométrie seule
        x = odom_x
        y = odom_y
    return (x, y)
```

#### Stratégie 2: Détection de divergence (robustesse)
```python
def fuse_with_collision_check(odom_x, odom_y, lidar_x, lidar_y, confidence):
    dist = hypot(lidar_x - odom_x, lidar_y - odom_y)
    
    # Si écart > 150mm et Lidar confiant → probablement collision
    if dist > 150 and confidence > 0.5:
        logger.warning(f"⚠️ COLLISION PROBABLE: {dist}mm écart Lidar-Odom")
        # Faire confiance au Lidar (obstacle détecté)
        return (lidar_x, lidar_y)
    
    # Sinon, fusion normale
    return fuse_simple(odom_x, odom_y, lidar_x, lidar_y, confidence)

## Données affichées dans Foxglove

### Plot (courbes temporelles)
```json
{
  "x_target": 2000,     // Position cible du Path
  "y_target": 1000,
  "x_odom": 1950,       // Odométrie brute Teensy
  "y_odom": 1005,
  "x_lidar": 1998,      // Position Lidar (balises)
  "y_lidar": 998,
  "x_fused": 1975,      // Fusion finale (60% Lidar + 40% Odom)
  "y_fused": 1001,
  "theta_imu": 0.157    // Angle: TOUJOURS IMU
}
```

### Scene 3D (4 marqueurs visuels)
```
ROBOT (RED):    Cylindre + flèche (position fusionnée)
TARGET (GREEN): Marqueur simple (position cible)
ODOM (BLUE):    Marqueur simple (odométrie brute)
LIDAR (YELLOW): Marqueur simple (position lidar)

θ appliqué au ROBOT seul (autres marqueurs ignorent rotation)
```

## Intégration dans le code

### Architecture recommandée

```
Flux principal:
  Capteurs → Path Finding (fusion) → Navigation → Teensy Moteurs
                   ↓
              Foxglove (monitoring)
```

### 1. Initialiser le bridge (simple, Foxglove = listenner)
```python
from foxglove_bridge_advanced import FoxgloveBridgeAdvanced

bridge = FoxgloveBridgeAdvanced(port=8765).start()
```

### 2. Dans votre Path Finding/Navigator
```python
class Navigator:
    def __init__(self, bridge=None):
        self.bridge = bridge  # Référence au bridge (optionnel)
        self.odom = (0.0, 0.0)
        self.lidar = (0.0, 0.0)
        self.imu_theta = 0.0
        self.target = (0.0, 0.0)
    
    def update_sensors(self, x_odom, y_odom, x_lidar, y_lidar, conf_lidar, theta_imu):
        """Mets à jour les données capteur."""
        self.odom = (x_odom, y_odom)
        self.lidar = (x_lidar, y_lidar)
        self.imu_theta = theta_imu
        self.confidence_lidar = conf_lidar
    
    def compute_fused_position(self):
        """Fusion réelle ici."""
        if self.confidence_lidar > 0.2:
            x_fused = 0.6 * self.lidar[0] + 0.4 * self.odom[0]
            y_fused = 0.6 * self.lidar[1] + 0.4 * self.odom[1]
        else:
            x_fused = self.odom[0]
            y_fused = self.odom[1]
        return (x_fused, y_fused, self.imu_theta)
    
    def set_target(self, x, y):
        self.target = (x, y)
    
    def update_foxglove(self):
        """Publie l'état courant pour monitoring."""
        if not self.bridge:
            return
        
        x_fused, y_fused, theta = self.compute_fused_position()
        
        self.bridge.publish_data(
            target_x=self.target[0],
            target_y=self.target[1],
            odom_x=self.odom[0],           # Données brutes pour debug
            odom_y=self.odom[1],
            lidar_x=self.lidar[0],         # Données brutes pour debug
            lidar_y=self.lidar[1],
            lidar_confidence=self.confidence_lidar,
            imu_theta=self.imu_theta
        )
    
    def navigate(self):
        """Logique navigation (utilise position fusionnée)."""
        x_fused, y_fused, theta = self.compute_fused_position()
        
        # Calculs de navigation
        dx = self.target[0] - x_fused
        dy = self.target[1] - y_fused
        distance_to_target = math.hypot(dx, dy)
        
        # ...
        
        # Publier état pour monitoring
        self.update_foxglove()
```

### 3. Enregistrer les callbacks (optionnel)
```python
navigator = Navigator(bridge=bridge)

# Callbacks depuis COM (Teensy, Lidar, IMU)
def on_rolling_basis(data: bytes):
    if len(data) >= 24:
        x, y, _ = struct.unpack("<ddd", data[:24])
        navigator.odom = (x, y)

def on_lidar_position(data: bytes):
    if len(data) >= 20:
        x, y, conf = struct.unpack("<ddf", data[:20])
        navigator.lidar = (x, y)
        navigator.confidence_lidar = conf

def on_imu_angle(data: bytes):
    if len(data) >= 8:
        theta, = struct.unpack("<d", data[:8])
        navigator.imu_theta = theta

com.add_callback(on_rolling_basis, Messages.UPDATE_ROLLING_BASIS.value)
com.add_callback(on_lidar_position, Messages.LIDAR_POSITION.value)
com.add_callback(on_imu_angle, Messages.IMU_ANGLE.value)

# Boucle principale
while True:
    navigator.navigate()
    # update_foxglove() appelé dedans
    time.sleep(0.05)
```

### 4. Commander le robot
```python
# La fusion décide de la position à utiliser
x_fused, y_fused, theta = navigator.compute_fused_position()

# Envoyer commandes au Teensy basées sur cette position
# (PID control, trajectoire planning, etc.)
```

## Messages manquants à implémenter

Actuellement définis dans `messages.py`:
- ✅ UPDATE_ROLLING_BASIS (128) — Teensy odométrie
- ✅ LIDAR_SCAN_PART1-4 (130-133) — Points laser bruts

À ajouter:
```python
# Dans common/usb_com/python/messages.py

class Messages(Enum):
    # ... existants ...
    
    # Nouveaux (à créer)
    LIDAR_POSITION = 134        # Position calculée par balises
    """Lidar-calculated position from beacon triangulation."""
    
    IMU_ANGLE = 135             # Angle θ (+ optionnel accel/gyro)
    """IMU orientation and angular velocity."""
    
    TARGET_POSITION = 136       # Position cible Path Finding
    """Target position from Path Finding algorithm."""
```

## Debug & Diagnostique

### Vérifier que les données arrivent
```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Les logs afficheront:
# DEBUG | Odom: (1950.0, 1005.0)
# DEBUG | Lidar: (1998.0, 998.0) conf=0.85
# DEBUG | IMU θ: 0.1570 rad (9.0°)
# DEBUG | Target: (2000.0, 1000.0)
```

### Vérifier la fusion
```python
from foxglove_bridge_advanced import FoxgloveBridgeAdvanced

bridge = FoxgloveBridgeAdvanced()
# ... publier des données ...

snap = bridge._fusion.get_snapshot()
print(f"Odom: {snap['odom']}")
print(f"Lidar: {snap['lidar']}")
print(f"Fused: {bridge._fusion.fuse_position()}")
```

## Paramètres à ajuster (dans Path Finding)

### Poids de fusion
```python
# Dans votre Navigator.compute_fused_position()

# Actuel: 60% Lidar, 40% Odom
x_fused = 0.6 * lidar_x + 0.4 * odom_x

# Plus agressif Lidar (80/20):
x_fused = 0.8 * lidar_x + 0.2 * odom_x

# Plus agressif Odom (40/60):
x_fused = 0.4 * lidar_x + 0.6 * odom_x
```

### Seuil de confiance Lidar
```python
# Actuel: confiance > 0.2 → utiliser Lidar
if lidar_confidence > 0.2:

# Plus strict (seulement très confiant):
if lidar_confidence > 0.6:

# Plus tolérant:
if lidar_confidence > 0.1:
```

### Détection divergence (collision probable)
```python
# Distance max acceptable avant alarme
MAX_DIVERGENCE_MM = 150

dist = hypot(lidar_x - odom_x, lidar_y - odom_y)
if dist > MAX_DIVERGENCE_MM and confidence > 0.5:
    logger.warning(f"⚠️ Collision: {dist}mm")
    # Actions: ralentir, éviter, etc.
```

---

## Prochaines étapes

1. ✅ Créer les messages IMU_ANGLE, LIDAR_POSITION, TARGET_POSITION
2. ✅ Implémenter les callbacks correspondants
3. ✅ Créer votre Navigator/PathFinding avec fusion
4. ✅ Intégrer Foxglove pour la visualisation (listener)
5. ⏳ Ajouter Kalman filter si dérive observée
6. ⏳ Ajouter détection d'obstacles Lidar avancée
7. ⏳ Logger les données pour post-analysis
