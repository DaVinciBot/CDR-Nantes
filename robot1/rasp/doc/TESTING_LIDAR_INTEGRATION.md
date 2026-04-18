# Guide de Test: LiDAR + Correction d'Odométrie

## 🚀 Démarrage Rapide

### Pré-requis
- ✅ RPLidar branché en USB
- ✅ Teensy branchée en USB
- ✅ Balises visibles et calibrées (positions dans `BEACONS_BY_ID`)
- ✅ Environnement Python configuré (numpy, rplidar, etc.)

### Test Progressif (5 niveaux)

```bash
cd /path/to/robot1/rasp

# NIVEAU 1: Vérifier que LiDAR est détecté (basique)
python3 test_lidar_correction_integration.py --level 1 --duration 10

# NIVEAU 2: Balises détectées?
python3 test_lidar_correction_integration.py --level 2 --duration 20

# NIVEAU 3: SVD calcule une correction?
python3 test_lidar_correction_integration.py --level 3 --duration 30

# NIVEAU 4: Filtre complémentaire fonctionne? (HORS-LIGNE)
python3 test_lidar_correction_integration.py --level 4 --duration 5

# NIVEAU 5: Intégration complète (Teensy + LiDAR)
python3 test_lidar_correction_integration.py --level 5 --duration 60
```

---

## 📊 Interprétation des Résultats

### Niveau 1: Détection LiDAR Basique ✓
**Attendus:**
- Frames acquises: > 100 (pour 10s)
- Points/frame: 100-400 typiques
- Distance: 300-6000mm

**Si échoue:**
- [ ] LiDAR pas détecté → Vérifier port USB: `ls /dev/ttyUSB*`
- [ ] Aucun scan reçu → Vérifier permission série: `sudo chmod 666 /dev/ttyUSB0`

---

### Niveau 2: Balises Détectées ✓
**Attendus:**
- Frames avec balises: > 30% (au moins quelques balises)
- Balises/frame: 0-3 (3 balises max)
- Candidats uniques: 2-4 (clusters de balises)

**Si échoue:**
- [ ] Balises pas visibles → Vérifier visibilité physique
- [ ] Seuils trop stricts → Modifier `BEACON_QUAL_MIN`, `BEACON_MIN_RETURNS_PER_CLUSTER` dans `lidar_logic.py`
- [ ] Positions dans `BEACONS_BY_ID` incorrectes → Recalibrer via mesure laser

---

### Niveau 3: SVD Correction ✓
**Attendus:**
- Corrections réussies: > 50% des frames
- Confiance moyenne: > 0.60
- Beacons utilisés: 2-3 par correction

**Si échoue:**
- [ ] < 2 balises → Vérifier niveau 2 d'abord
- [ ] Confiance basse → RMS residual trop haut (mauvaise géométrie?)
- [ ] `_compute_corrected_pose()` crash → Vérifier format candidats

---

### Niveau 4: Filtre Complémentaire ✓
**Attendus:**
- Alpha @ 0.0 conf: 0.85 ✓
- Alpha @ 0.5 conf: ~0.55 ✓
- Alpha @ 1.0 conf: 0.25 ✓
- Monotonie: alpha décroît ✓

**Si échoue:**
- [ ] Alpha invalide → Bug dans formule (vérifier `robot.py` L78-112)
- [ ] Non-monotone → Transition linéaire cassée

---

### Niveau 5: Intégration Complète ✓
**Attendus:**
- Poses Teensy reçues: > 500 (pour 60s @ 10 Hz)
- Poses corrigées: > 20
- Corrections Teensy envoyées: > 10
- Intervalle corrections: 1.0-2.0s

**Si échoue:**
- [ ] Aucune pose Teensy → Vérifier communication Teensy
- [ ] Aucune correction calculée → Revoir niveaux 2-3
- [ ] Intervalle > 2s → Throttle pas respecté

---

## 🔧 Troubleshooting

### Erreur: "Port série non trouvé"
```bash
# Lister ports disponibles
ls -la /dev/ttyUSB*

# Donner permission
sudo chmod 666 /dev/ttyUSB0
```

### Erreur: "Import RPLidar failed"
```bash
# Installer dépendances
pip install rplidar numpy scipy
```

### Erreur: "Pas de balises détectées"
```bash
# Vérifier BEACONS_BY_ID dans terrain_jeu.py
# Positions doivent être en MONDE (mm)

# Exemple pour table CDR 2026:
BEACONS_BY_ID = {
    1: (250, 2850),    # Coin bas-gauche
    2: (2750, 2850),   # Coin bas-droit
    3: (1500, 100),    # Haut centre
}
```

### Erreur: "SVD confidence trop basse"
```python
# Dans lidar_logic.py, réduire seuils:
BEACON_FIT_MAX_RMS_MM = 200.0  # Augmenter tolérance RMS
POSE_CORRECTION_MIN_CONFIDENCE = 0.50  # Baisser seuil confiance
```

---

## 📈 Exemple de Sortie Réussie

```
NIVEAU 1: DÉTECTION LIDAR BASIQUE
  ✓ LiDAR DÉTECTÉ ET FONCTIONNEL
  Frames acquises: 195
  Points totaux: 48920
  Points/frame: 250.9
  Distance: 310mm - 5950mm

NIVEAU 2: DÉTECTION DE BALISES
  ✓ BALISES DÉTECTÉES
  Frames avec balises: 52/195
  Balises/frame (moyenne): 1.50
  Candidats uniques: 3

NIVEAU 3: CORRECTION SVD UMEYAMA
  ✓ SVD CORRECTION FONCTIONNE
  Corrections réussies: 78/195 (40.0%)
  Confiance moyenne: 0.72
  Beacons utilisés: [['1', '2'], ['2', '3'], ...]

NIVEAU 4: FILTRE COMPLÉMENTAIRE
  ✓ Low confidence: alpha=0.850 (expected 0.850)
  ✓ Boundary low: alpha=0.850 (expected 0.850)
  ✓ Medium confidence: alpha=0.550 (expected 0.550)
  ✓ Boundary high: alpha=0.250 (expected 0.250)
  ✓ High confidence: alpha=0.250 (expected 0.250)
  ✓ Monotonie: alpha décroît avec confiance

NIVEAU 5: INTÉGRATION COMPLÈTE
  Durée: 60s | Frames: 1200
  Poses Teensy reçues: 650
  Poses corrigées (SVD): 120
  Corrections envoyées à Teensy: 42
  Intervalle corrections: 1.42s (target: 1-2s)
  Confiance moyenne: 0.68
```

---

## 📋 Checklist Pré-Deployment

Avant de tester en match:

- [ ] **Niveau 1 PASS**: LiDAR détecté
- [ ] **Niveau 2 PASS**: Balises détectées (min 50% frames)
- [ ] **Niveau 3 PASS**: SVD corrections (min 30% success)
- [ ] **Niveau 4 PASS**: Filtre adaptatif correct
- [ ] **Niveau 5 PASS**: Intégration Teensy OK
- [ ] Intervalle corrections 1-2s respecté
- [ ] Confiance moyenne > 0.60
- [ ] Aucun crash Python
- [ ] Debug logs activés pour première course

---

## 🎯 Commandes Pratiques

### Test rapide avant match (5 min)
```bash
# Run all levels with short duration
for level in 1 2 3 4 5; do
  echo "NIVEAU $level..."
  python3 test_lidar_correction_integration.py --level $level --duration 10
done
```

### Acquérir 1 minute de données brutes
```bash
# Capture scan data + Teensy odometry
python3 test_lidar_correction_integration.py --level 5 --duration 60
```

### Test avec visualisation (si GUI disponible)
```bash
# Launch Rerun GUI (separate terminal)
cd lidar/
python3 launch_rerun.py

# Run test (another terminal)
python3 test_lidar_correction_integration.py --level 3 --duration 30
```

---

## 📚 Références

- **SVD Umeyama**: [ODOMETRY_CORRECTION_IMPLEMENTATION.md](ODOMETRY_CORRECTION_IMPLEMENTATION.md)
- **API LiDAR**: [lidar_logic.py](lidar/lidar_logic.py) - Public functions (L780+)
- **Filtre complémentaire**: [robot.py](robot.py#L78-L112)
- **Configuration**: [terrain_jeu.py](terrain_jeu.py) - `BEACONS_BY_ID`

---

## 💡 Conseils

1. **Commencer par niveau 1**: Si LiDAR marche pas, les autres niveaux sont inutiles
2. **Observer pendant 20s**: Quelques secondes suffisent pour détecter beacons
3. **Vérifier positions balises**: Si `BEACONS_BY_ID` faux, SVD sera toujours en erreur
4. **Monitor confiance SVD**: Si < 0.5, augmenter `BEACON_FIT_MAX_RMS_MM`
5. **Première sortie**: Garder throttle corrections à 2.0s (conservative)

---

## 📞 Debugging Avancé

### Afficher tous les logs
```bash
python3 -c "
import logging
logging.basicConfig(level=logging.DEBUG)
" && python3 test_lidar_correction_integration.py --level 3 --duration 20
```

### Vérifier constantes
```bash
python3 -c "
from lidar.lidar_logic import BEACON_WINDOW_ANGLE_RAD, POSE_CORRECTION_MIN_CONFIDENCE
print(f'Window angle: {BEACON_WINDOW_ANGLE_RAD}')
print(f'Min confidence: {POSE_CORRECTION_MIN_CONFIDENCE}')
"
```

### Dump candidats bruts
```bash
# Modifier lidar/lidar_logic.py pour afficher _extract_beacon_candidates_fast()
# Ajouter: print(f"Candidates: {candidates}")
```

---

**Bon test! 🚀**

