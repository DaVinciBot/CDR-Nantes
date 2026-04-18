# Rerun Visualisation — Eurobot 2026

**Rerun** remplace **Foxglove** (legacy). Interface web moderne, timeline interactive, réplay historique.

---

## 🚀 Démarrage rapide  

### Mode 1: Simulation locale (tests, pas de hardware)

```bash
cd robot1/rasp/
python rerun/rerun_bridge.py --mode local --sim
```

→ Viewer s'ouvre directement. Robot en cercle, données fictives.

### Mode 2: Serveur web (Rasp → PC distant)

**Sur la Raspberry Pi:**
```bash
cd robot1/rasp/
python rerun/rerun_bridge.py --mode serve --host 0.0.0.0 --port 9876
```

**Sur votre PC (même réseau):**
```
http://192.168.1.100:9876  (remplacer par IP Rasp)
```

### Mode 3: Avec hardware (Teensy + Lidar)

```bash
python rerun/rerun_bridge.py --mode serve --with-lidar --port 9876
```

Active polling du Lidar. Vérifier que:
- ✅ Teensy connecté (auto-détecté)
- ✅ Lidar sur COM5 (ou adapter `lidar/lidar_logic.py`)
- ✅ Balises visibles en simulation Webots

---

## 📁 Lancements alternatifs

### Via helper script (depuis `/lidar/`)

```bash
cd robot1/rasp/lidar/
python launch_rerun.py --mode local --sim
python launch_rerun.py --mode serve --port 9876
```

### Via Python (sans helper externe)

```python
import subprocess
import sys

subprocess.run([
    sys.executable,
    "rerun/rerun_bridge.py",
    "--mode", "serve",
    "--with-lidar",
    "--port", "9876",
])
```

---

## 🎨 Interface Rerun

**Panneau 3D (haut):**
- Terrain textured + obstacles statiques (balises, murs, caisses)
- 🔵 Robot bleu = odométrie Teensy brute
- 🔴 Robot rouge = position Lidar calculée (trilatération 3 balises)
- 🟢 Robot vert = position fusionnée (60% Lidar + 40% Teensy)
- 🟨 Point jaune = cible PathFinding (si actif)
- 📍 Nuage de points = Lidar raw (grisâtre→ cyan selon intensité)
- 🏷️ Diamants orange = balises détectées par Lidar

**Panneau temporel (bas):**
- Courbes 4x1 (4 graphes indépendants)
- Position Teensy X/Y/theta
- Lidar confidence / nb_beacons / nb_points
- Position fusionnée X/Y
- Écart odom↔lidar

**Timeline (gauche):**
- Barre temporelle = historique des données (recherchercher/replay)
- Avancer/reculer temporel = changer la visualisation

---

## 🔧 Configuration matériel

### Teensy

Auto-détecté sur:
- Windows: `COM*` port COM
- Linux/Mac: `/dev/ttyACM*` ou `/dev/ttyUSB*`

Adapter chemins dans `utils.py` si besoin.

### Lidar (RPLidar A1)

Connecté sur: `COM5` (Windows) ou `/dev/ttyUSB0` (Linux)

Adapter dans `lidar/lidar_logic.py`:
```python
lidar = RPLidar("/dev/ttyUSB0")  # Linux
# ou
lidar = RPLidar("COM5")           # Windows
```

### Balises

Positions définie dans `terrain_jeu.py`:
```python
class BeaconLayout:
    BEACONS = {
        1: (-50.0, 1000.0),     # Balise A
        2: (3050.0, 1950.0),    # Balise B
        3: (3050.0, 50.0),      # Balise C
    }
```

Rerun charge automatiquement depuis là (source-of-truth pattern).

---

## 🌐 Accès depuis PC portable (recommandé en comp\u00e9tition)

### Étape 1: Trouver l'IP Rasp

**Sur la Rasp:**
```bash
hostname -I
# Sortie: 192.168.1.100
```

### Étape 2: Lancer Rerun sur Rasp

```bash
python rerun/rerun_bridge.py --mode serve --port 9876
```

### Étape 3: Ouvrir navigateur PC

Navigateur quelconque (Chrome, Firefox, Edge), entrer:
```
http://192.168.1.100:9876
```

**Résultat:**
- Visualisation live du terrain + robot
- Timeline rechercherble
- Zéro latence (WebSocket local réseau)
- Stable même en WiFi

---

## 📊 Monitoring en temps réel

### Voir écart odom ↔ Lidar

Courbe **data/fusion/ecart_odom_lidar_mm** affiche la distance erreur.

- **Faible** (< 100 mm) = fusion fiable
- **Élevée** (> 500 mm) = vérifier calibration balises

### Voir confidence Lidar

Courbe **data/lidar/confidence** (0.0 à 1.0):

- **> 0.8** = trilatération précise
- **0.3-0.8** = modéré, fusion pondere
- **< 0.3** = confiance basse, utiliser Teensy seul

### Voir nb_beacons détectées

Courbe **data/lidar/nb_beacons**:

- **3** = idéal (trilatération possible)
- **1-2** = dégradé (position calculée mais incertain)
- **0** = pas de balises, utiliser Teensy

---

## ⚙️ Paramétrage fusion

**Fichier:** `rerun/rerun_bridge.py`, ligne ~480

```python
# Actuellement: 60% Lidar + 40% Teensy
fused_x = 0.6*lidar_x + 0.4*teensy_x

# Pour changer: 80% Lidar + 20% Teensy
fused_x = 0.8*lidar_x + 0.2*teensy_x
```

Redémarrer le service et observer l'effet sur robot vert.

---

## 🐛 Dépannage

| Problème | Cause | Solution |
|----------|-------|----------|
| Serveur ne répond pas (ERR_CONNECTION_REFUSED) | Port 9876 utilisé | `lsof -i :9876` puis tuer le vieux process ou changer `--port 9877` |
| Navigateur affiche blanc | JavaScript bloqué | Autoriser JavaScript (même origine) |
| Robot ne bouge pas | Teensy/Lidar déconnecté | Chercher logs `Teensy indisponible` |
| Nuage Lidar vide | Balises pas détectées | Lancer `lidar/lidar_gui.py` pour debug |
| Timeline ne remplit pas | Process arrêté | Vérifier logs du terminal Rerun |
| Lidar en mode simulation mais robot bleu ne bouge pas ? | Simulation activée sans --sim | Relancer avec `--mode local --sim` |

---

## 📚 Prochaines étapes

1. **Valider le matériel:** `python test_local.py` (simulation) → OK?
2. **Tester Teensy:** `python rerun/rerun_bridge.py --mode serve --port 9876` → PC se connecte?
3. **Intégrer Lidar:** `python rerun/rerun_bridge.py --mode serve --with-lidar` → robot rouge bouge?
4. **Fusion:** Observer robot vert = fusionné (bleu+rouge)
5. **PathFinding:** Créer cible → voir trajectoire (bleue) → mouvement
6. **Compétition:** Lancer sur Rasp, observer depuis PC portable dans mode serveur

---

## 📖 Documentation détaillée

- [../../archive_rasp/rerun_docs/TEST_TEENSY_LIDAR_SIMPLE_README.md](../../archive_rasp/rerun_docs/TEST_TEENSY_LIDAR_SIMPLE_README.md) — Archive test simple Teensy+Lidar sans fusion
- [ARCHITECTURE_GENERALE.md](ARCHITECTURE_GENERALE.md) — Architecture globale (Rerun, Teensy, Lidar, PathFinding)
- [GUIDE_DEMARRAGE_COMPLET.md](GUIDE_DEMARRAGE_COMPLET.md) — Guide complet (plus détaillé)

---

## 🔗 Fichiers clés

| Fichier | Rôle |
|---------|------|
| `rerun/rerun_bridge.py` | **Bridge principal** Teensy+Lidar→Rerun |
| `lidar/launch_rerun.py` | Launcher depuis lidar/ |
| `test_local.py` | Test simulation locale |
| `terrain_jeu.py` | Positions balises (source-of-truth) |
| `lidar/lidar_logic.py` | PoseEngine trilatération |
| `lidar/main.py` | GUI debug Lidar |

---

## ✅ Checklist démarrage compé

- [ ] Rerun démarré sur Rasp: `python rerun/rerun_bridge.py --mode serve --port 9876`
- [ ] PC portable accède: `http://RaspIP:9876`
- [ ] Robot bleu (Teensy) bouge
- [ ] Robot rouge (Lidar) positionné
- [ ] Robot vert (fusion) entre les deux
- [ ] Courbes temporelles montent
- [ ] Timeline se remplit
- [ ] → Prêt pour PathFinding + compé! 🎉
