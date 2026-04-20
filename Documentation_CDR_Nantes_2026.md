#  Documentation Technique — DaVinciBot Nantes (CDR)

> Robot holonome autonome à 3 roues omnidirectionnelles — Coupe de France de Robotique

---

## 🔴 CRITICAL UPDATES (April 20, 2026)

**Status**: ✅ Ready for Match (3 critical bugs fixed)

| Issue | Fix | Status |
|-------|-----|--------|
| `test_program.py:L12` | Import changed: `GestionnaireLidar` → `LidarInterface` | ✅ FIXED |
| `lidar_logic.py:L607` | beacon_ids type fixed: `List[str]` → `List[int]` | ✅ FIXED |
| `lidar_gui.py` | Dead beacon detection code removed (~180 lines) | ✅ CLEANED |

**For complete codebase status**, see:  
👉 [`robot1/rasp/doc/CODEBASE_ANALYSIS_2026_UNIFIED.md`](robot1/rasp/doc/CODEBASE_ANALYSIS_2026_UNIFIED.md) — **Single source of truth** for Python state

---

##  Table des matières

1. [Vue d'ensemble du projet](#1-vue-densemble-du-projet)
2. [Architecture matérielle](#2-architecture-matérielle)
3. [Organisation du dépôt](#3-organisation-du-dépôt)
4. [Protocole de communication USB](#4-protocole-de-communication-usb)
5. [Système de contrôle moteur (Teensy Moteur)](#5-système-de-contrôle-moteur-teensy-moteur)
6. [Cinématique holonomique](#6-cinématique-holonomique)
7. [Odométrie et fusion de capteurs](#7-odométrie-et-fusion-de-capteurs)
8. [Contrôle PID](#8-contrôle-pid)
9. [Teensy Actuateur](#9-teensy-actuateur)
10. [Simulation Webots](#10-simulation-webots)
11. [Code Raspberry Pi](#11-code-raspberry-pi)
12. [Guide de démarrage rapide](#12-guide-de-démarrage-rapide)


---

## 1. Vue d'ensemble du projet

Le robot DaVinciBot Nantes est conçu pour la **Coupe de France de Robotique**, compétition de 90 secondes où des robots entièrement autonomes s'affrontent sur une table de jeu dont les règles changent chaque année.

### Stack technologique

| Composant | Technologie | Rôle |
|---|---|---|
| Cerveau principal | Raspberry Pi — Python 3 | Prise de décision, navigation globale |
| Contrôleur moteur | Teensy 4.1 — C++/PlatformIO | Asservissement temps réel (PID + steppers) |
| Contrôleur actuateurs | Teensy 4.1 — C++/PlatformIO | Servos, pas-à-pas, LCD, switchs |
| Communication | USB série — protocole maison CRC8 | Liaison Raspberry Pi ↔ Teensy |
| Simulation | Webots R2025a | Tests et développement hors robot |

---

## 2. Architecture matérielle

### 2.1 Vue globale

```
┌─────────────────────────────────────────────────────────┐
│                    RASPBERRY PI                         │
│         (Python 3 — Décision & Navigation)              │
│                                                         │
│   config.json  │  loader.py  │  test_*.py               │
└────────┬────────────────────────────┬───────────────────┘
         │ USB Série (115200 baud)    │ USB Série (115200 baud)
         │ CRC8 + Signature           │ CRC8 + Signature
         ▼                            ▼
┌─────────────────┐        ┌─────────────────────────┐
│ TEENSY 4.1      │        │ TEENSY 4.1               │
│ Base Motrice    │        │ Actuateurs               │
│                 │        │                         │
│ • PID X/Y/θ    │        │ • Servos (I2C PCA9685)  │
│ • Cinématique  │        │ • Steppers (A4988)      │
│ • Odométrie    │        │ • LCD I2C               │
│ • KaribouMotion│        │ • Switchs               │
└────────┬────────┘        └──────────────┬──────────┘
         │                               │
    ┌────▼────┐                    ┌─────▼──────┐
    │ 3 NEMA  │                    │   Actua-   │
    │ 23 +    │                    │   teurs    │
    │ Encodeurs│                   │            │
    └─────────┘                    └────────────┘
```

### 2.2 Base mobile — Disposition des roues

```
             Y+ (avant)
              ^
              |
    [W2 240°] | [W1 120°]
              |
    X- ───────+──────── X+
              |
          [W3 0°]
              |
             Y- (arrière)
```

| Roue | Position physique | Angle cinématique | STEP | DIR | EN |
|------|------------------|-------------------|------|-----|----|
| W1   | Haut Droite      | 120°              | 2    | 3   | 4  |
| W2   | Haut Gauche      | 240°              | 5    | 6   | 7  |
| W3   | Arrière          | 0°                | 8    | 9   | 10 |

### 2.3 Paramètres physiques

| Paramètre | Valeur |
|---|---|
| Rayon robot (centre → axe roue) | 156.9 mm |
| Diamètre roue effectif | 60.0 mm |
| Steps/révolution (NEMA 23) | 200 |
| Microstepping | 32 |
| Steps totaux/tour | 6 400 |
| Vitesse max | 20 000 steps/s |
| Accélération max | 10 000 steps/s² |

---

## 3. Organisation du dépôt

```
CDR-Nantes/
│
├── common/                        # Bibliothèques partagées
│   ├── teensy/                    # Classes Python pour Teensy
│   │   ├── base_teensy.py         # Classe de base (reset, com)
│   │   ├── gpio_teensy.py         # Gestion GPIO étendue
│   │   └── tools/gpio_manager/    # Gestionnaire de pins
│   └── usb_com/
│       ├── cpp/                   # Implémentation C++ du protocole
│       │   ├── include/com.h
│       │   ├── include/messages.h # Définition de tous les messages
│       │   ├── include/crc.h
│       │   └── src/
│       └── python/                # Implémentation Python du protocole
│           ├── com/com.py         # Classe Com principale
│           ├── com/dummy.py       # Stub de test sans hardware
│           └── messages.py        # Enum Messages
│
├── robot1/                        # Instance du robot
│   ├── rasp/                      # Code Raspberry Pi
│   │   ├── config.json            # Config USB (simulation / hardware)
│   │   ├── loader.py              # Chargeur dynamique de modules
│   │   ├── robot_context.py       # Détection auto simulation/hardware
│   │   ├── webots_com.py          # Pont COM virtuel Webots
│   │   ├── switch_mode.py         # Bascule sim ↔ hardware
│   │   └── test/                  # Scripts de test hardware
│   ├── teensy_moteur/             # Firmware contrôleur moteur
│   │   ├── src/main.cpp
│   │   ├── include/config.h       # Pins, PID gains, géométrie
│   │   └── lib/
│   │       ├── holonomic_basis/   # Classe principale de navigation
│   │       ├── KaribouMotion/     # Librairie stepper maison
│   │       └── pid/               # Contrôleur PID
│   └── teensy_actuator/           # Firmware contrôleur actuateurs
│       └── src/main.cpp
│
└── simulation/                    # Simulation Webots
    ├── controllers/teensy_controller/
    │   ├── teensy_controller.cpp  # Point d'entrée Webots
    │   ├── fake_stepper.cpp       # Mock steppers → moteurs Webots
    │   ├── ArduinoFake.h/.cpp     # Stubs Arduino
    │   └── sensors/               # Mocks GPS, IMU
    └── worlds/my_world.wbt        # Monde Webots
```

---

## 4. Protocole de communication USB

### 4.1 Format des messages

```
┌──────────┬──────────────┬──────────┬─────────────────────────┐
│  ID msg  │   Payload    │  Taille  │   CRC8  │  Signature    │
│  1 byte  │  N bytes     │  1 byte  │  1 byte │  4 bytes      │
│          │              │          │         │  BA DD 1C C5  │
└──────────┴──────────────┴──────────┴─────────────────────────┘
```

- **CRC8** calculé sur `[payload + taille]`
- **Signature** : `0xBA 0xDD 0x1C 0xC5` (constante des deux côtés)
- En cas de CRC invalide : envoi de `NACK (127)` → retransmission automatique

### 4.2 Table des messages

| ID | Nom | Direction | Payload |
|----|-----|-----------|---------|
| 0 | `SET_TARGET_POSITION` | RPI → Teensy M | `double x, y, theta` |
| 1 | `SET_PID` | RPI → Teensy M | `byte axis, float kp, ki, kd` |
| 2 | `SET_ODOMETRIE` | RPI → Teensy M | `double x, y, theta` |
| 3 | `SET_SERVO_ANGLE_I2C` | RPI → Teensy A | `byte pin, uint16 angle, max_angle` |
| 4 | `STEPPER_STEP` | RPI → Teensy A | `int steps, bool dir, int speed, byte pins` |
| 5 | `SET_SERVO_ANGLE_DETACH` | RPI → Teensy A | `byte pin, angle, delay` |
| 6 | `ATTACH_SWITCH` | RPI → Teensy A | `byte pin` |
| 7 | `SET_SERVO_ANGLE` | RPI → Teensy A | `byte pin, uint16 angle, max_angle` |
| 8 | `SET_STEPPER_DRIVER_ACTIVATION_STATE` | RPI → Teensy A | `byte pin, bool state` |
| 126 | `RESET_TEENSY` | RPI → Teensy | — |
| 127 | `NACK` | Bidirectionnel | — |
| 128 | `UPDATE_ROLLING_BASIS` | Teensy M → RPI | `double x, y, theta` |
| 129 | `SWITCH_STATE_RETURN` | Teensy A → RPI | `byte pin, bool state` |
| 130-133 | `LIDAR_SCAN_PART1-4` | Teensy → RPI | 90× `uint16` distances + timestamp |
| 254 | `PRINT` | Teensy → RPI | ASCII text |
| 255 | `UNKNOWN_MSG_TYPE` | Teensy → RPI | `byte type_id` |

### 4.3 Exemple d'utilisation Python

```python
import struct
from loader import loader
from robot_context import init_robot

Messages = loader.load_class('usb_com', 'Messages')
com, mode = init_robot(logger)

# Envoyer une position cible
msg = Messages.SET_TARGET_POSITION.to_bytes()
msg += struct.pack('<ddd', 200.0, 0.0, 0.0)  # x, y, theta en mm/rad
com.send_bytes(msg)

# Recevoir la position courante
def on_position(data: bytes):
    x, y, theta = struct.unpack('<ddd', data[:24])
    print(f"X={x:.1f}mm  Y={y:.1f}mm  θ={theta:.3f}rad")

com.add_callback(on_position, Messages.UPDATE_ROLLING_BASIS.value)
```

---

## 5. Système de contrôle moteur (Teensy Moteur)

### 5.1 Architecture logicielle

```
loop() — 600Hz environ
    │
    ├─ com->handle_callback()      ← Réception messages USB
    └─ Envoi UPDATE_ROLLING_BASIS  ← Télémétrie vers RPI

IntervalTimer COMPUTE — 100 Hz (10 ms)
    ├─ holonomic_basis->update_odometry()   ← Encodeurs + GPS + IMU
    ├─ holonomic_basis->handle(target, com) ← Calcul PID + cinématique
    ├─ holonomic_basis->execute_movement()  ← Steps relatifs → moteurs
    └─ holonomic_basis->compute_steppers()  ← Profils trapézoïdaux

IntervalTimer STEP — 25 kHz (40 µs)
    └─ holonomic_basis->step_steppers()     ← Génération impulsions STEP
```

### 5.2 Librairie KaribouMotion

Librairie de contrôle stepper développée en interne pour remplacer TeensyStep4. Fournit :

- **Profils trapézoïdaux** : accélération/décélération douce
- **Synchronisation** : `StepperGroup` assure que les 3 moteurs arrivent simultanément
- **Speed scaling** : ratio dynamique pour préserver la trajectoire
- **Non-bloquant** : génération de pulses par interruption (25 kHz)

```cpp
// Exemple d'utilisation
StepperGroup group(wheel1, wheel2, wheel3);
group.setTargetsRel(steps1, steps2, steps3);  // Déplacement relatif
group.startMove();    // Calcule les scales, lance le mouvement

// En ISR rapide :
group.step();         // Génère les pulses physiques
// En ISR lente :
group.compute();      // Met à jour les profils de vitesse
```

---

## 6. Cinématique holonomique

### 6.1 Cinématique inverse (Robot → Roues)

Conversion des vitesses souhaitées `(vx, vy, ω)` en vitesses de roues :

```
w1 = -(−0.5·vx + 0.866·vy − ω·R)   [Roue 1 — 120°]
w2 = -(−0.5·vx − 0.866·vy − ω·R)   [Roue 2 — 240°]
w3 =    1.0·vx                + ω·R  [Roue 3 — 0°  ]
```

Toutes les vitesses sont en **steps/seconde**. Le facteur de conversion :

```
speed_factor = (steps_per_rev × microsteps) / (π × wheel_diameter)
```

### 6.2 Cinématique directe (Roues → Robot) — pour l'odométrie

```
vx   = (2·w3 − w1 − w2) / 3
vy   = (w1 − w2) / √3
ω·R  = (w1 + w2 + w3) / 3
```

### 6.3 Normalisation des vitesses

Pour préserver la direction de déplacement quand une roue sature :

```cpp
double max_speed = max(|w1|, |w2|, |w3|);
if (max_speed > MAX_SPEED) {
    double scale = MAX_SPEED / max_speed;
    w1 *= scale; w2 *= scale; w3 *= scale;
}
```

---

## 7. Odométrie et fusion de capteurs

### 7.1 Sources de données

| Capteur | Mode hardware | Mode simulation |
|---------|--------------|-----------------|
| Encodeurs (position steppers) | Positions absolues des moteurs | `wb_position_sensor_get_value()` |
| Optique PAA5100 (flux optique) | Vrai capteur SPI (counts → mm) | Mock GPS Webots (m → mm) |
| IMU BNO08x (Game Rotation Vector) | I2C, quaternion → yaw | Mock InertialUnit Webots |

### 7.2 Fusion complémentaire

```
X, Y final = 20% encodeurs + 80% optique  (si capteur optique valide)
θ final    = IMU (prioritaire)  ou  intégration encodeurs (fallback)
```

Filtres appliqués :
- **Outlier rejection** : magnitude > 15 mm/cycle rejetée
- **Filtre repos** : en simulation, GPS ignoré si robot immobile (bruit ±2 mm)
- **Rotation pure** : mouvements X/Y optiques ignorés si rotation détectée

---

## 8. Contrôle PID

### 8.1 Implémentation

La classe `PID` inclut :
- **Anti-windup** : clamping de l'intégrale dans `[minOutput/ki, maxOutput/ki]`
- **Deadband lisse** : zone morte linéaire autour de zéro (friction statique)
- **Garde sur dt** : clamping `[dtMin=1ms, dtMax=20ms]`


### 8.2 Zone morte globale

Le robot s'arrête si : `distance_error < 0.75 mm` **ET** `angle_error < 0.02 rad (~1.1°)`

---

## 9. Teensy Actuateur

### 9.1 Gestion des actionneurs

Tous les actionneurs sont stockés dans un tableau `void* actuators[48]` indexé par numéro de pin. L'instanciation est **lazy** (création à la première utilisation).

| Message reçu | Action |
|---|---|
| `SET_SERVO_ANGLE` | Attache et commande un servo direct |
| `SET_SERVO_ANGLE_I2C` | Servo via PCA9685 (I2C, 12 bits PWM) |
| `SET_SERVO_ANGLE_DETACH` | Servo + détachement après délai |
| `STEPPER_STEP` | Moteur pas-à-pas via driver A4988 |
| `SET_STEPPER_DRIVER_ACTIVATION_STATE` | Active/désactive un driver |
| `ATTACH_SWITCH` | Configure un switch + envoie état initial |

### 9.2 Surveillance des switchs

Dans `loop()`, tous les pins déclarés comme switch sont relus. Un `SWITCH_STATE_RETURN` est envoyé automatiquement si l'état change.

---

## 10. Simulation Webots

### 10.1 Architecture simulation

La simulation reproduit fidèlement le code du Teensy en substituant :

| Réel | Simulation |
|------|------------|
| `stepper.h` (KaribouMotion) | `fake_stepper.cpp` → `wb_motor_set_velocity()` |
| PAA5100 (capteur optique) | `Mock_PAA5100.h` → GPS Webots |
| BNO08x (IMU) | `Mock_BNO085.h` → InertialUnit Webots |
| `Serial` (USB) | `SerialMock` → port COM virtuel Windows |

### 10.2 Prérequis simulation

1. **Webots R2025a** installé
2. **Virtual Serial Port Tools** (ou com0com) : paire COM1 ↔ COM2
3. Créer `webots_path.mk` dans `simulation/controllers/teensy_controller/` :

```makefile
WEBOTS_HOME = C:/Program Files/Webots
```

### 10.3 Compilation du contrôleur

```bash
cd simulation/controllers/teensy_controller
make clean && make
```

### 10.4 Lancement

```bash
# Terminal 1 : Lancer Webots avec my_world.wbt
# Terminal 2 :
cd robot1/rasp
python switch_mode.py simulation
python test_program.py
```

---

## 11. Code Raspberry Pi

### 11.1 Détection automatique simulation/hardware

`robot_context.py` détecte le contexte par ordre de priorité :

1. Variable d'environnement `ROBOT_MODE=simulation|hardware`
2. Fichier marqueur `.simulation_mode` dans le répertoire courant
3. `config.json` : `port=COM1` → simulation, `serial_number` présent → hardware
4. Chemin contenant "simulation" → simulation

```python
from robot_context import init_robot
com, mode = init_robot(logger)
# mode == "SIMULATION" ou "HARDWARE"
```

### 11.2 Basculer entre les modes

```bash
python switch_mode.py simulation   # Configure config.json pour COM1
python switch_mode.py hardware     # Configure config.json pour Teensy USB
```

### 11.3 Chargeur de modules

`loader.py` charge dynamiquement les classes depuis `config.json`, permettant de pointer vers différentes implémentations sans modifier le code :

```python
from loader import loader
Com      = loader.load_class('usb_com', 'Com')
Messages = loader.load_class('usb_com', 'Messages')
config   = loader.get_config('serial_config')
```

---

## 12. Guide de démarrage rapide

### 12.1 Sur le robot réel

```bash
# 1. Installer les dépendances Python
pip install pyserial crc8 loggerplusplus

# 2. Passer en mode hardware
cd robot1/rasp
python switch_mode.py hardware

# 3. Vérifier la détection de la Teensy
python test/test_usb_detection.py

# 4. Tester la communication
python test/test_communication.py

# 5. Lancer un test de mouvement
python test_simple_traj.py
```

### 12.2 En simulation Webots

```bash
# 1. Créer webots_path.mk avec votre chemin Webots
# 2. Compiler le contrôleur
cd simulation/controllers/teensy_controller
make

# 3. Lancer Webots > ouvrir simulation/worlds/my_world.wbt

# 4. Dans un autre terminal :
cd robot1/rasp
python switch_mode.py simulation
python test_program.py
```

### 12.3 Envoyer une commande minimale

```python
import struct, logging
from loader import loader
from robot_context import init_robot

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

Messages = loader.load_class('usb_com', 'Messages')
com, _ = init_robot(logger)

# Aller à X=200mm, Y=0, θ=0
msg = Messages.SET_TARGET_POSITION.to_bytes()
msg += struct.pack('<ddd', 200.0, 0.0, 0.0)
com.send_bytes(msg)
```

---

*Documentation générée pour le projet CDR-Nantes — DaVinciBot (ESILV) — Mise à jour : Février 2026*
