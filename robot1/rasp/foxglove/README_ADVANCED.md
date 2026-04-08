# Foxglove Bridge Advanced — Synthèse

## ⚠️ Clarification: Foxglove = Monitoring, pas Fusion

**Foxglove n'effectue PAS la fusion.**

La fusion réelle (Teensy Odom + Lidar + IMU) doit se faire dans votre **Path Finding/Navigator**.  
Foxglove est juste un **listener** pour l'affichage et le débogage en temps réel.

```
Teensy Odom + Lidar + IMU → PATH FINDING (fusion logique)
                                     ↓ (données finalisées)
                              Foxglove (visualisation)
                                     ↓
                         📺 4 marqueurs visibles
                         pour monitoring/debug
```

## 🚀 Quick Start

### 1. Test synthétique (rapide, sans hardware)
```bash
cd c:\Users\Depot\CDR-Nantes\robot1\rasp\foxglove
python test_synthetic_advanced.py
```

Puis:
1. Ouvrez Foxglove (http://localhost:8765)
2. Vous verrez:
   - **ROBOT ROUGE** : tourne (fusion 60% Lidar + 40% Odom)
   - **VERT** : cercle externe (target)
   - **BLEU** : cercle moyen (odométrie Teensy)
   - **JAUNE** : cercle inner (Lidar)

### 2. Intégration réelle (avec hardware)
```bash
python test_integration_advanced.py
```

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

## 🔧 Fusion (dans votre Path Finding, PAS dans Foxglove)

Foxglove reçoit la fusion finalisée. Voici où la faire:

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

### Puis envoyer à Foxglove
```python
# Dans votre boucle principale
x, y, theta = navigator.fuse_position()

bridge.publish_data(
    target_x=..., target_y=...,
    odom_x=navigator.odom[0],
    odom_y=navigator.odom[1],
    lidar_x=navigator.lidar[0],
    lidar_y=navigator.lidar[1],
    lidar_confidence=navigator.confidence,
    imu_theta=navigator.imu_theta
    # ↑ Foxglove reçoit les données brutes + fusion info
    # ↑ Affiche les 4 marqueurs pour monitoring
)
```

**Où les 4 marqueurs affichent**:
- 🔴 ROBOT (ROUGE): La fusion que Path Finding a calculée
- 🟢 TARGET (VERT): Position cible (debug)
- 🔵 ODOM (BLEU): Odométrie brute Teensy (debug)
- 🟡 LIDAR (JAUNE): Position Lidar (debug)

## 🎨 Visualisation Foxglove

| Couleur | Source | Affichage |
|---------|--------|-----------|
| 🔴 ROUGE | Fused (fusion) | **Cylindre + Flèche** (θ_imu) |
| 🟢 VERT | Target | Petit cylindre (marqueur) |
| 🔵 BLEU | Odom | Petit cylindre (marqueur) |
| 🟡 JAUNE | Lidar | Petit cylindre (marqueur) |

**Plot (courbes)**:
- x_target, y_target
- x_odom, y_odom
- x_lidar, y_lidar (+ confidence bar)
- x_fused, y_fused
- theta_imu

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
