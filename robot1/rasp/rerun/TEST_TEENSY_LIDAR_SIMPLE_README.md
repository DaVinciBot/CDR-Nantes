# Test Simple: Teensy + Lidar vers Rerun

**Objectif**: Recevoir les données brutes du Teensy et les données calculées du Lidar, les afficher dans **Rerun** **SANS fusion**.

---

## 1. Architecture

Ce test **appelle directement** le PoseEngine du Lidar:

```
Teensy (odométrie)
    ↓ [USB callback]
    → Affiche en � BLEU

RPLidar (capteur)
    ↓ [lidar_logic.py: PoseEngine → trilatération 3 balises]
    → get_latest_pose() → (x, y, theta, confidence)
    → Affiche en 🔴 ROUGE
    
FUSION (optionnel)
    ↓ [combinaison pondérée]
    → Affiche en 🟢 VERT
```

**Avantage**: 
- Pas besoin de créer des messages USB pour Lidar. Le `PoseEngine` fait déjà la trilatération.
- Visualisation **en temps réel** avec timeline pour rejouer l'historique
- Interface web accessible depuis PC distant (mode serveur)

---

## 2. Démarrer le test

### Option A: Mode simulation (pas de hardware)

```bash
cd robot1/rasp/
python rerun/rerun_bridge.py --mode local --sim
```

Cela ouvre automatiquement le Viewer avec un robot qui tourne en cercle.

### Option B: Mode serveur (Rasp vers PC distant)

**Sur la Raspberry Pi:**
```bash
cd robot1/rasp/
python rerun/rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876
```

**Sur votre PC portable:**
1. Obtenir l'adresse IP de la Rasp: `ip addr` ou `hostname -I` sur la Rasp
2. Ouvrir navigateur: `http://192.168.X.X:9876` (remplacer par l'IP de la Rasp)
3. La visualization s'affiche en direct dans le navigateur

### Option C: Avec hardware Lidar + Teensy

```bash
python rerun/rerun_bridge.py --mode serve --with-lidar --port 9876
```

**Attendu:**
```
INFO | ✓ Balises depuis terrain_jeu: [...]
INFO | Carte statique publiée — 3 balises | 6 supports | 54 caisses
INFO | 🌐 Mode SERVE activé
INFO |    Serveur Rerun WebSocket sur 0.0.0.0:9876
INFO |    Accédez depuis un navigateur : http://[RaspIP]:9876
INFO | ✓ Callback Teensy enregistré (mode rasp5)
INFO | Lidar polling activé
INFO | ▶ Boucle de publication à 20 Hz
```

---

## 3. Visualiser dans Rerun

### Mode LOCAL (auto-ouverture du Viewer)

Le Viewer s'ouvre automatiquement. Vous devriez voir:

```
├── world/
│   ├── map/
│   │   ├── playmat (textured quad)
│   │   ├── walls (4 murs)
│   │   ├── beacons (3 cylinders)
│   │   ├── crates (54 boîtes jaunes/bleues)
│   │   └── ...
│   ├── robot/
│   │   └── odom (🔵 BLEU) = Teensy odométrie brute
│   ├── lidar/
│   │   ├── cloud (nuage de points)
│   │   ├── beacons_detected (diamants orange)
│   │   └── pose (🔴 ROUGE) = Lidar position calculée
│   ├── robot/
│   │   └── fused (🟢 VERT) = Position fusionnée
│   └── pathfinding/
│       ├── target (🟨 point jaune)
│       └── trajectory (ligne bleue pointillée)
└── data/ (courbes temporelles)
    ├── teensy/
    ├── lidar/
    ├── fused/
    └── fusion/
```

**Panneau supérieur**: Vue 3D interactive du terrain (zoom, rotate, pan)

**Panneau inférieur**: Graphiques temporels (4 courbes temps-réel)

### Mode SERVE (navigateur)

1. Attendre que le serveur se lance: `Serveur Rerun WebSocket sur 0.0.0.0:9876`
2. Ouvrir navigateur: `http://192.168.1.100:9876` (remplacer par IP Rasp)
3. Interface identique au mode LOCAL mais **accès distant**

---

## 4. Comprendre les sources

| Source | Couleur | Calculé par | Fichier |
|--------|--------|---|---|
| Teensy odométrie | 🔵 BLEU | Teensy (encodeurs roues) | `rerun/rerun_bridge.py` |
| Lidar trilatération | 🔴 ROUGE | `PoseEngine` (3 balises) | `lidar/lidar_logic.py` |
| Fusion pondérée | 🟢 VERT | Combinaison 60% Lidar + 40% Teensy | `rerun/rerun_bridge.py` |

**Flux Lidar:**
```
RPLidar (COM5)
  ↓ pts bruts (angle, distance, intensité)
  ↓ lidar_logic.py:
    1. Détection balises via clustering
    2. Trilatération (3 balises → position)
    3. PoseEngine.latest_pose → (x, y, theta, confidence)
  ↓ rerun_bridge.py → publish dans Rerun
```

---

## 5. Le fichier test_teensy_lidar_simple.py

**Avant (Foxglove):**
```python
from foxglove_bridge import FoxgloveBridgeAdvanced
bridge = FoxgloveBridgeAdvanced()
bridge.publish_data(robot_x=..., target_x=..., ...)
```

**Maintenant (Rerun):**
```python
import rerun as rr
rr.log("world/robot/odom", rr.Points3D(...))
rr.log("world/lidar/pose", rr.Points3D(...))
# Timeline automatique + web UI
```

Le test est maintenant **plus simple et plus puissant** (replay historique, interface web).

## 6. Déboguer

### Les données Lidar ne bougent pas?

```bash
# 1. Vérifier que le Lidar est connecté
ls -la /dev/ttyUSB*

# 2. Vérifier que les balises sont visibles (GUI Lidar)
python lidar/main.py  # Ou lidar_gui.py pour interface graphique

# 3. Vérifier les logs du bridge
python rerun/rerun_bridge.py --mode local --sim 2>&1 | grep -i lidar
```

### Erreur d'import SYS.PATH?

Le fichier `rerun_bridge.py` ajoute le chemin parent automatiquement:
```python
sys.path.insert(0, str(_DIR.parent))  # Ajouter /robot1/rasp/
from terrain_jeu import BeaconLayout
```

Si ça échoue encore, vérifier:
```bash
cd robot1/rasp/
python -c "from terrain_jeu import BeaconLayout; print(BeaconLayout.BEACONS)"
```

### Mode serveur ne répond pas?

```bash
# Port déjà utilisé?
lsof -i :9876  # Voir quel processus utilise le port

# Pare-feu?
# S'assurer que le trafic sur le port 9876 n'est pas bloqué
# Sur la Rasp: aucune config firewall par défaut (OK)
# Sur votre PC: vérifier que le trafic entrant est autorisé
```

### Timeline vide dans Rerun?

- Timeline se remplit automatiquement au fur et mesure que les données arrivent (20 Hz)
- Si elle reste vide: vérifier que `publish_loop()` tourne
- Vérifier les logs pour `Frame @ T` messages

---

## 7. Accès depuis PC portable

### Sur Raspberry Pi:

```bash
# Découvrir l'IP
hostname -I
# Sortie: 192.168.1.100

# Lancer le bridge
python rerun/rerun_bridge.py --mode serve --port 9876
```

### Sur votre PC:

1. **Même réseau (WiFi/Ethernet)?**
   - Oui → `http://192.168.1.100:9876`
   - Non → Utiliser tunnel SSH:
   ```bash
   ssh -L 9876:localhost:9876 user@192.168.1.100
   # Puis ouvrir: http://localhost:9876
   ```

2. Ouvrir navigateur (Chrome, Firefox, Edge, Safari)

3. Visualisation live + timeline interactive

## 8. Passer à la fusion de position

Une fois que vous voyez Teensy (bleu) + Lidar (rouge) bouger séparément:

**Fusion automatique:**
```python
# Dans rerun_bridge.py, _publish() publie déjà:
fused_x = 0.6 * lidar_x + 0.4 * teensy_x  (si lidar_confidence > 0.2)
fused_y = 0.6 * lidar_y + 0.4 * teensy_y

# Affichage en vert (C_FUSED = [80, 255, 120, 255])
_log_robot("world/robot/fused", fused_x, fused_y, fused_theta, C_FUSED)
```

**Pour personnaliser les poids:**

Éditer `rerun_bridge.py` ligne ~475:
```python
# Actuellement (60% Lidar, 40% Teensy):
fused_x = 0.6*lx+0.4*ox

# Exemple: 80% Lidar, 20% Teensy
fused_x = 0.8*lx+0.2*ox
```

## 9. Prochaine étape: PathFinding

Une fois la fusion stable:

1. Créer une cible (point jaune dans la visualisation)
2. Lancer le pathfiding
3. Voir la trajectoire calculée (ligne bleue)
4. Exécuter le mouvement sur le robot

Les données circuleront toutes via la même visualisation Rerun!

## Notes

- **Pas de fusion ici** = debug brut pour voir les 2 sources (lisez la section **8** pour fusion)
- **Lidar via `get_latest_pose()`** = utilise directement le PoseEngine (robuste)
- **Rerun = visualization + timeline** = rejouez l'historique des capteurs
- **Mode serveur = accès web** = visualisez depuis PC pendant que la Rasp fait tourner le robot
- **Performance = 20 Hz** = boucle temps-réel, fluide même au travers d'une liaison web



