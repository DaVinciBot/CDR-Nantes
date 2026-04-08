# Messages à ajouter pour Foxglove Advanced

## Situation actuelle

Dans `common/usb_com/python/messages.py`:

```python
class Messages(Enum):
    # Teensy → Rasp (128-255)
    UPDATE_ROLLING_BASIS = 128    # ✅ Odométrie Teensy (x, y, θ_brut)
    LIDAR_SCAN_PART1-4 = 130-133  # ✅ Points laser bruts (angle, distance)
    PRINT = 254
    UNKNOWN_MSG_TYPE = 255
```

## Manquants

Pour que FoxgloveBridgeAdvanced fonctionne complètement, il faut créer:

### 1. LIDAR_POSITION (134)
**Source**: Lidar (côté Rasp, calcul via balises)  
**Direction**: Rasp → Foxglove (envoyer position calculée)  
**Format proposé**:
```
Header: 1 byte = 134
x: double (8 bytes) — position X en mm
y: double (8 bytes) — position Y en mm
confidence: float (4 bytes) — 0.0-1.0, fiabilité de la position
timestamp: uint64 (8 bytes) — ns depuis démarrage
Total: 29 bytes minimum
```

**Exemple de callback**:
```python
def on_lidar_position(data: bytes):
    if len(data) < 20:
        return
    x, y = struct.unpack("<dd", data[:16])
    confidence = struct.unpack("<f", data[16:20])[0]
    bridge._fusion.update_lidar(x, y, confidence=confidence)
```

### 2. IMU_ANGLE (135)
**Source**: IMU (accéléro + gyro, côté Teensy)  
**Direction**: Teensy → Rasp  
**Format proposé**:
```
Header: 1 byte = 135
theta: double (8 bytes) — angle en radians
angular_velocity: double (8 bytes) — dθ/dt (optionnel, pour Kalman)
timestamp: uint64 (8 bytes) — ns depuis démarrage
Total: 25 bytes minimum
```

**Exemple de callback**:
```python
def on_imu_angle(data: bytes):
    if len(data) < 8:
        return
    theta = struct.unpack("<d", data[:8])[0]
    bridge._fusion.update_imu(theta)
```

### 3. TARGET_POSITION (136)
**Source**: Path Finding (côté Rasp)  
**Direction**: Rasp → Foxglove (affichage position cible)  
**Format proposé**:
```
Header: 1 byte = 136
x: double (8 bytes) — position cible X en mm
y: double (8 bytes) — position cible Y en mm
is_reached: uint8 (1 byte) — 0/1, si position atteinte
timestamp: uint64 (8 bytes) — ns depuis démarrage
Total: 25 bytes minimum
```

**Exemple de callback**:
```python
def on_target_position(data: bytes):
    if len(data) < 16:
        return
    x, y = struct.unpack("<dd", data[:16])
    bridge._fusion.update_target(x, y)
```

---

## ✅ Points importants

### Unités
- **X, Y**: toujours en **millimètres (mm)**
- **θ**: toujours en **radians (rad)**
- **timestamp**: en **nanosecondes (ns)** ou **millisecondes (ms)**

### Ordre des bytes
- **Format**: `<` = Little-endian (standard Intel)
- **Doubles**: 8 bytes
- **Floats**: 4 bytes
- **Integers**: selon size

### Exemple structs
```python
# Écriture (Teensy C++)
struct data {
    double x;
    double y;
    float confidence;
};

# Lecture (Rasp Python)
x, y, conf = struct.unpack("<ddf", data[:20])
```

---

## 📋 Checklist d'implémentation

- [ ] **Côté Teensy C++** (si c'est le Teensy qui envoie IMU_ANGLE):
  - [ ] Créer `IMU_ANGLE = 135` dans `firmware/messages.h`
  - [ ] Implémenter `send_imu_angle(theta, dtheta)`
  - [ ] Envoyer tous les X ms (ex: 50ms = 20 Hz)

- [ ] **Côté Teensy C++** (si balises gérées localement):
  - [ ] Créer `LIDAR_POSITION = 134` dans `firmware/messages.h`
  - [ ] Implémenter `send_lidar_position(x, y, confidence)`

- [ ] **Côté Rasp Python** (`common/usb_com/python/messages.py`):
  - [ ] Ajouter `LIDAR_POSITION = 134`
  - [ ] Ajouter `IMU_ANGLE = 135`
  - [ ] Ajouter `TARGET_POSITION = 136`
  - [ ] `to_bytes()` pour tous

- [ ] **Côté Rasp Python** (`robot1/rasp/foxglove/test_integration_advanced.py`):
  - [ ] Décommenter les callbacks (lignes ~60-80)
  - [ ] Tester avec données synthétiques d'abord

---

## Variante: Si messages lidar/imu sont ailleurs

Si `LIDAR_POSITION` et `IMU_ANGLE` existent **déjà** sous d'autres noms, il suffit de:

1. Chercher les définitions existantes:
   ```bash
   grep -r "LIDAR\|IMU\|imu" common/usb_com/python/
   ```

2. Adapter les callbacks dans `test_integration_advanced.py`:
   ```python
   com.add_callback(
       integrator.on_lidar_position,
       Messages.LIDAR_DATA.value  # ou le vrai nom
   )
   ```

3. Adapter les structures unpack dans les callbacks.

---

## Temporaire: Si vous manquez IMU_ANGLE

Si le Teensy n'a pas d'IMU mais que vous avez l'angle ailleurs, vous pouvez:

1. **Option A**: Calculer θ depuis l'odométrie (moins fiable mais fonctionne)
   ```python
   def compute_theta_from_odom(x1, y1, x0, y0):
       return math.atan2(y1-y0, x1-x0)
   ```

2. **Option B**: Utiliser un **gyro/accéléro USB** externe (compatible)

3. **Option C**: Laisser θ = 0 (robot ne tourne pas, test basique)

---

## Test avant/après

### Avant (actuellement):
```python
# Seule source: UPDATE_ROLLING_BASIS
bridge.publish_data(
    target_x=0, target_y=0,
    odom_x=1950, odom_y=1005,
    lidar_x=0, lidar_y=0, lidar_confidence=0.0,
    imu_theta=0.0
)
# ✓ Fonctionne, mais marqueurs Lidar/Target vides
```

### Après (une fois nouveaux messages):
```python
# Toutes sources:
bridge.publish_data(
    target_x=2000, target_y=1000,
    odom_x=1950, odom_y=1005,
    lidar_x=1998, lidar_y=998, lidar_confidence=0.85,
    imu_theta=0.157  # ← Vient de l'IMU!
)
# ✓ 4 marqueurs visibles
# ✓ Robot fusion visible
# ✓ Courbes plot complètes
```

---

## Questions courantes

**Q1: L'ordre des bytes est important?**  
R: Oui! `<ddf` = little-endian, double, double, float. Doit correspondre exactement.

**Q2: Puis-je ajouter plus de champs?**  
R: Oui, mais attention à la taille totale et à l'ordre.

**Q3: Fusion sur Teensy ou Rasp?**  
R: Actuellement: fusion sur Rasp (plus simple). Vous pouvez ajouter une fusion Teensy aussi.

**Q4: timeout si message n'arrive pas?**  
R: Non, le bridge utilise la dernière valeur connue. Les seuils de confiance assurent pas de données pourries.
