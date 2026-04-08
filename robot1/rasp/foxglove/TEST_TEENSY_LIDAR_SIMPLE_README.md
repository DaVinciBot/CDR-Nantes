# Test Simple: Teensy + Lidar vers Foxglove

**Objectif**: Recevoir les données brutes du Teensy et les données calculées du Lidar, les afficher dans Foxglove **SANS fusion**.

---

## 1. Architecture

Ce test **appelle directement** le PoseEngine du Lidar:

```
Teensy (odométrie)
    ↓ [USB callback]
    → Affiche en 🔴 ROUGE

RPLidar (capteur)
    ↓ [lidar_logic.py: PoseEngine → trilatération 3 balises]
    → get_latest_pose() → (x, y, theta, confidence)
    → Affiche en 🟢 VERT
```

**Avantage**: Pas besoin de créer des messages USB pour Lidar. Le `PoseEngine` fait déjà la trilatération.

---

## 2. Démarrer le test

```bash
cd robot1/rasp/foxglove
python test_teensy_lidar_simple.py
```

**Attendu**:
```
TEST SIMPLE: Teensy + Lidar → Foxglove (SANS fusion)
✓ Mode: [simulation|rasp5]
✓ Thread Lidar démarré
✓ Callback Teensy (UPDATE_ROLLING_BASIS) enregistré
📡 Écoute Teensy + Lidar...

Lancez Foxglove: ws://localhost:8765
Vous devriez voir:
  🔴 ROBOT (ROUGE) = Teensy odométrie brute
  🟢 TARGET (VERT) = Lidar position calculée (trilatération)
```

---

## 3. Vérifier dans Foxglove

1. Ouvrir Foxglove Studio
2. Connexion → `ws://localhost:8765`
3. Ajouter panel **3D Scene**
4. Vous devriez voir **2 marqueurs qui bougent indépendamment**:
   - 🔴 **ROBOT (ROUGE)** = Teensy odométrie brute
   - 🟢 **TARGET (VERT)** = Lidar position (calculée par PoseEngine)

---

## 4. Comprendre le Lidar

**Fichiers impliqués:**

| Fichier | Rôle |
|---------|------|
| `robot1/rasp/lidar/lidar_logic.py` | PoseEngine = trilatération via 3 balises, retourne `PoseState(x, y, theta, confidence)` |
| `robot1/rasp/lidar/lidar_navigation_bridge.py` | Packétise pour pathfinding (non utilisé ici) |
| `test_teensy_lidar_simple.py` | **Vous êtes ici**: Affiche Teensy + Lidar dans Foxglove |

**Flux Lidar:**
1. RPLidar sur COM5 envoie des points bruts
2. `lidar_logic.py` détecte les balises (A, B, C) via clustering
3. PoseEngine fait trilatération → retourne `(x, y, theta, confidence)`
4. Ici on appelle `get_latest_pose()` dans la boucle main

---

## 5. Déboguer

**Les données Lidar ne bougent pas?**
- Vérifier que le Lidar est connecté sur COM5 (ou adapter dans `lidar_logic.py`)
- Vérifier que les balises sont visibles et à bonne distance (50-12000 mm)
- Lancer d'abord le GUI Lidar (`robot1/rasp/lidar/lidar_gui.py` ou `main.py`) pour confirmer que la trilatération fonctionne

**Erreur d'import?**
```python
# Si l'import échoue, vérifier les chemins en haut du fichier:
sys.path.insert(0, str(Path(__file__).parent.parent))
sys.path.insert(0, str(Path(__file__).parent))
```

**Logging detaillé:**
```bash
python test_teensy_lidar_simple.py 2>&1 | grep -E "Teensy|Lidar|LIDAR"
```

---

## 6. Ajouter un 3e source (IMU, par exemple)

Une fois que vous voyez Teensy + Lidar bouger:

1. Créer un callback IMU (ex: `on_imu_angle(theta)`)
2. Ajouter dans `_publish()`:
```python
def _publish(self) -> None:
    # ... existant ...
    self.bridge.publish_data(
        odom_x=self.odom["x"],
        odom_y=self.odom["y"],
        target_x=self.lidar_pose["x"],
        target_y=self.lidar_pose["y"],
        imu_theta=self.imu_theta,  # ← Nouveau!
        lidar_x=??? ou lidar_y=???  # ← 4e source si besoin
    )
```

---

## 7. Prochaine étape: FUSION dans Navigator

Une fois que vous voyez 2-3 sources en Foxglove:

1. Créez `Navigator` class dans `robot1/rasp/navigator.py`
2. Intégrez `fuse_position()` qui combine Teensy + Lidar
3. La fusion **N'EST PAS** ici, c'est dans Navigator!

Exemple:
```python
class Navigator:
    def fuse_position(self):
        if self.lidar_confidence > 0.2:
            x_fused = 0.6 * self.lidar_x + 0.4 * self.teensy_x
            y_fused = 0.6 * self.lidar_y + 0.4 * self.teensy_y
        else:
            x_fused = self.teensy_x
            y_fused = self.teensy_y
        return (x_fused, y_fused, self.imu_theta)
```

Puis dans la boucle main:
```python
nav = Navigator(teensy_data, lidar_data)
fused_pos = nav.fuse_position()
bridge.publish_data(target_x=fused_pos[0], target_y=fused_pos[1], ...)
```

---

## Notes

- **Pas de fusion ici** = debug brut pour voir les 2 sources
- **Lidar via `get_latest_pose()`** = utilise directement le PoseEngine
- **Une fois ça fonctionne** = déplacer la fusion dans Navigator (chemin recommandé)
- **Foxglove = visualization only** = pas de logique ici, juste affichage



