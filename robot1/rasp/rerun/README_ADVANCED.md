# Rerun Bridge — Architecture de visualisation

> 📌 **Simple et puissant**: Visualisation temps réel en Rerun SDK (local ou serve)

## ⚠️ Clarification: Rerun = Visualisation temps réel, pas Fusion

**Rerun n'effectue PAS la fusion.**

La fusion réelle (Teensy Odom + Lidar + IMU) se fait dans votre **Path Finding/Navigator**.  
Rerun est un **visualiseur SDK** pour le monitoring et débogage en temps réel avec timeline intégrée.

```
Teensy Odom + Lidar + IMU → PATH FINDING (fusion logique)
                                     ↓ (données finalisées)  
                            Rerun SDK rs://buffer
                         (monitoring 3D + timeline)
                                     ↓
                     📊 Terrain + positions historiques
                     + Plots temporelles automatiques
```

## 🚀 Quick Start

### Mode 1: Rerun local (viewport intégré)
```bash
cd e:\CDR\CDR-Nantes\robot1\rasp
python rerun_bridge.py --mode local --sim
# ✅ Viewer s'ouvre automatiquement
```

### Mode 2: Rerun serve (web)
```bash
python rerun_bridge.py --mode serve --port 9876
# Puis ouvrir: http://localhost:9876
```

### Vous verrez:
- **Terrain 3D en haut** : Carte CDR 3000×2000mm avec tous les éléments (balises, caisses, grenier, murs, playmat)
- **Robots colorés** :
  - 🔵 BLEU : Position Teensy odométrie brute → `world/robot/odom`
  - 🔴 ROUGE : Position fusionnée → `world/robot/fused`
  - 🟡 ORANGE : Position Lidar (trilatération) → `world/lidar/pose`
- **Nuage Lidar** : Points verts-bleus projetés en coordonnées monde
- **Abaque temporelle en bas** : Position X/Y, Lidar confidence, écart odom↔lidar

### Mode 3: Test simple (Teensy + Lidar SANS fusion)
```bash
python rerun/test_teensy_lidar_simple.py
```

Affiche:
- 🔵 Position Teensy en bleu
- 🔴 Position Lidar en rouge
- **Pas de fusion**, just affichage brut des 2 sources

Vous devez d'abord créer les messages manquants:
```python
# common/usb_com/python/messages.py
class Messages(Enum):
    # ... existants ...
    LIDAR_POSITION = 134   # Position Lidar calculée
    IMU_ANGLE = 135        # Angle IMU
    TARGET_POSITION = 136  # Position cible Path
```

## 📊 Types de données (architecture générique)

## 📊 Types de données (architecture générique)

**Important**: Foxglove reçoit simplement ces données du Path Finding.  
C'est votre Navigator/Path Finding qui les calcule.

### Position2D (toujours en mm)
```python
position = Position2D(x=1500.0, y=1000.0)  # mm, repère absolu
# Bas-gauche (0,0), Haut-droit (3000, 2000)
```

### Orientation1D (en radians)
```python
orientation = Orientation1D(theta=0.157)  # rad (9°)
# Vient TOUJOURS de l'IMU
```

### SensorData (complet)
```python
SensorData(
    position=Position2D(...),
    theta=0.157,           # Optionnel / ignoré
    timestamp=1234567890,  # ns
    confidence=0.85,       # 0-1, pour Lidar
    source_name="lidar"
)
```

## 🔧 Fusion (dans votre Path Finding, PAS dans Rerun)

Rerun reçoit la fusion finalisée. Voici où la faire:

### Exemple: Navigator avec fusion
```python
class Navigator:
    def __init__(self):
        self.odom = (0.0, 0.0)
        self.lidar = (0.0, 0.0)
        self.imu_theta = 0.0
        self.confidence = 0.0
    
    def fuse_position(self):
        """Fusion réelle (moyenne pondérée)."""
        if self.confidence > 0.2:
            # Lidar valide: 60% + Odom 40%
            x = 0.6 * self.lidar[0] + 0.4 * self.odom[0]
            y = 0.6 * self.lidar[1] + 0.4 * self.odom[1]
        else:
            # Lidar absent: Odom seul
            x = self.odom[0]
            y = self.odom[1]
        
        return (x, y, self.imu_theta)  # ← IMU!
```

### Puis envoyer à Rerun
```python
# Dans votre boucle principale
import rerun as rr

x, y, theta = navigator.fuse_position()

# Publier la position du robot
rr.log("world/robot/odom", rr.Boxes3D(
    centers=[[navigator.odom[0], navigator.odom[1], 50]],
    half_sizes=[[150, 150, 50]],
    colors=[[51, 153, 255, 220]]  # Bleu
))

# Publier la position fusionnée
rr.log("world/robot/fused", rr.Boxes3D(
    centers=[[x, y, 50]],
    half_sizes=[[150, 150, 50]],
    colors=[[80, 255, 120, 220]]  # Vert
))

# Publier position Lidar
if navigator.confidence > 0:
    rr.log("world/lidar/pose", rr.Boxes3D(
        centers=[[navigator.lidar[0], navigator.lidar[1], 50]],
        half_sizes=[[150, 150, 50]],
        colors=[[255, 80, 80, 220]]  # Rouge
    ))

# Points temporels (courbes automatiques)
rr.log("data/fused/x_mm", rr.Scalars(x))
rr.log("data/fused/y_mm", rr.Scalars(y))
rr.log("data/fused/theta_deg", rr.Scalars(math.degrees(theta)))
rr.log("data/lidar/confidence", rr.Scalars(navigator.confidence))
```

**Où les robots affichent**:
- 🔵 BLEU: Odométrie Teensy brute → `world/robot/odom`
- 🟢 VERT: Fusion calculée → `world/robot/fused`
- 🔴 ROUGE: Position Lidar (trilatération) → `world/lidar/pose`

## 🎨 Visualisation Rerun

| Couleur | Source | Entity Path | Affichage |
|---------|--------|-------------|-----------|
| 🔵 BLEU | Odom | `world/robot/odom` | Cylindre + Flèche (θ_imu) |
| 🟢 VERT | Fused | `world/robot/fused` | Cylindre + Flèche (fusion) |
| 🔴 ROUGE | Lidar | `world/lidar/pose` | Cylindre + Flèche (trilatéra) |
| 🟡 ORANGE | Beacons | `world/lidar/beacons_detected` | Diamants en 3D |

**Timeline (courbes)** - Automatique:
- `data/teensy/x_mm`, `data/teensy/y_mm` → Position brute
- `data/lidar/confidence` → Validation Lidar
- `data/fused/x_mm`, `data/fused/y_mm`, `data/fused/theta_deg` → Fusion
- `data/fusion/*` → Écarts

## 📚 Documentation détaillée

Voir `ARCHITECTURE_GENERALE.md` pour:
- Schémas détaillés
- Tous les types de données
- Stratégies de fusion avancées
- Diagnostic et debug
- Messages à créer

## 🔄 Intégration callback par callback

### Actuellement supporté ✓
```python
com.add_callback(on_rolling_basis, Messages.UPDATE_ROLLING_BASIS.value)
# → bridge._fusion.update_odom(x, y)
```

### À ajouter une fois nouveaux messages créés
```python
com.add_callback(on_lidar, Messages.LIDAR_POSITION.value)
# → bridge._fusion.update_lidar(x, y, confidence)

com.add_callback(on_imu, Messages.IMU_ANGLE.value)
# → bridge._fusion.update_imu(theta)

com.add_callback(on_target, Messages.TARGET_POSITION.value)
# → bridge._fusion.update_target(x, y)
```

## 🧪 Vérifier que tout fonctionne

```python
# Dans test_integration_advanced.py
integrator = RobotDataIntegrator(bridge)

# Vérifier fusion
snap = bridge._fusion.get_snapshot()
print(f"Odom: {snap['odom']}")
print(f"Lidar: {snap['lidar']}")
x, y, theta = bridge._fusion.fuse_position()
print(f"Fused: ({x:.0f}, {y:.0f}) @ θ={theta:.3f}")
```

## ⚙️ Paramètres ajustables

Voir `foxglove_bridge_advanced.py`:

```python
# Ligne 320 — Poids fusion (60/40 par défaut)
x_fused = 0.6 * lidar.x + 0.4 * odom.x

# Ligne 305 — Seuil confiance Lidar
if lidar_data.confidence > 0.2:  # > 0.2 = "valide"

# Ligne 380 — Fréquence envoi Foxglove
await asyncio.sleep(0.05)  # 20 Hz (50ms)
```

## 📞 Support

- **Bug/Question** → Regarder `ARCHITECTURE_GENERALE.md` (section Debug)
- **Custom fusion** → Modifier `SensorFusionManager.fuse_position()`
- **Nouveau capteur** → Ajouter `update_*()` à `SensorFusionManager`

---

**Statut**: ✅ Prêt à l'intégration  
**Blocké par**: Création des messages IMU_ANGLE, LIDAR_POSITION, TARGET_POSITION dans messages.py  
**Prochaine étape**: Tester avec `test_synthetic_advanced.py` puis adapter callbacks réels
