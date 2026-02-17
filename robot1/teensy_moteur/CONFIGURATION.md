# Configuration Robot Holonomique 3 Roues

## Vue d'ensemble

Robot holonomique à 3 roues omnidirectionnelles avec contrôle PID position et moteurs pas-à-pas NEMA 17.

## Disposition des Roues

```
         Y+ (avant)
          ^
          |
     [W2] | [W1]    ← Roues omnidirectionnelles
      240°| 120°
          |
   X- ----+---- X+
          |
         [W3]
          0°
          |
         Y- (arrière)
```

### Positions Physiques

- **W1** : Haut Droite
- **W2** : Haut Gauche  
- **W3** : Arrière (centre)

## Cinématique Inverse

### Équations de Mouvement

Pour convertir les vitesses du robot (vx, vy, ω) en vitesses des roues (w1, w2, w3) :

```cpp
// Roue 1 (Haut Droite - 120°)
w1 = -(0.5 * vx - 0.866 * vy - ω)
// Coefficients : cos(120°) = -0.5, sin(120°) = +0.866

// Roue 2 (Haut Gauche - 240°)
w2 = -(0.5 * vx + 0.866 * vy - ω)
// Coefficients : cos(240°) = -0.5, sin(240°) = -0.866

// Roue 3 (Arrière - 0°)
w3 = vx + ω
// Coefficients : cos(0°) = +1.0, sin(0°) = 0
```

### Paramètres Géométriques

- **ROBOT_RADIUS** : 156.9 mm (distance centre → axe roue)
- **WHEEL_DIAMETER** : 60.0 mm (28mm principal + 32mm rouleaux)

## Moteurs Pas-à-Pas

### Configuration

| Moteur | Position | Angle | STEP | DIR | ENABLE |
|--------|----------|-------|------|-----|--------|
| W1     | Haut Droite | 120° | 2  | 3   | 4      |
| W2     | Haut Gauche | 240° | 5  | 6   | 7      |
| W3     | Arrière     | 0°   | 8  | 9   | 10     |

### Spécifications

- **Type** : NEMA 23
- **Steps/rev** : 200 (1.8° par step)
- **Microstepping** : 32
- **Total steps/rev** : 6400 (200 × 32)
- **MAX_SPEED** : 20000 steps/s
- **MAX_ACCELERATION** : 10000 steps/s²


## Normalisation Proportionnelle des Vitesses

### Principe

Pour préserver la géométrie du mouvement holonomique, si une roue dépasse MAX_SPEED, toutes les roues sont réduites du même ratio :

```cpp
double max_wheel_speed = max(|w1|, |w2|, |w3|)
if (max_wheel_speed > MAX_SPEED) {
    scale = MAX_SPEED / max_wheel_speed
    w1 *= scale
    w2 *= scale
    w3 *= scale
}
```

**Avantage** : Le robot ralentit en conservant sa trajectoire au lieu de dévier.

### Exemple

```
Vitesses calculées : w1=8000, w2=10000, w3=15000
MAX_SPEED = 12000
scale = 12000/15000 = 0.8

Après normalisation : w1=6400, w2=8000, w3=12000 ✓
```

## Capteurs Odométrie

### Simulation (Webots)

- **GPS Mock** : Lecture directe position Webots (précision ~2-3mm)
- **IMU Mock** : Orientation depuis capteurs Webots
- **Fusion** : 80% optique + 20% encodeurs

### Robot Réel (à venir)

- **PAA5100** : Capteur optique haute précision (<1mm)
- **BNO08x** : IMU 9 axes pour orientation
- **Encodeurs** : Rotatifs sur moteurs pas-à-pas


## Filtre Vitesse

- **Alpha** : 0.3 (lissage léger)
- **Formule** : `filtered = 0.3*new + 0.7*old`

Réduit les à-coups lors de changements de consignes.

## Axes et Référentiels

### Référentiel Monde (Webots)

- **X+** : Droite
- **Y+** : Avant
- **θ+** : Rotation antihoraire

### Transformation Robot → Monde

```cpp
vx_robot = cos(θ) * vx_world + sin(θ) * vy_world
vy_robot = -sin(θ) * vx_world + cos(θ) * vy_world
```

## Notes de Développement

### Historique Tuning PID

- **v1** : Kp=5.1, Ki=0.5 → Oscillations + suivi GPS Mock
- **v2** : Kp=5.1, Ki=0.0 → Erreur 6mm, dérive perpendiculaire
- **v3** : Kp=8.0, Ki=0.0 → Correction dérives (en test)

### Problèmes Résolus

1. **Saturation asymétrique** : Normalisation proportionnelle préserve direction
2. **Erreur compilation C++** : Remplacement max()→fmax(), abs()→fabs()
3. **Dérive Y→X** : Augmentation Kp pour correction plus agressive

---

**Dernière mise à jour** : 17 février 2026  
**Version firmware** : Simulation Webots R2023b
