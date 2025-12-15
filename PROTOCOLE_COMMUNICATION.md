```mermaid
sequenceDiagram
    participant RPI as 🖥️ Raspberry Pi
    participant USB as 🔌 USB Com
    participant TM as 🤖 Teensy Moteur
    participant TA as 🦾 Teensy Actuateur
    participant HW as ⚙️ Hardware

    Note over RPI,HW: 🚀 SÉQUENCE COMPLÈTE DE FONCTIONNEMENT

    %% ===== INITIALISATION =====
    rect rgb(240, 248, 255)
        Note over RPI,HW: 📋 Phase d'initialisation
        RPI->>USB: Charger config.json
        RPI->>USB: Initialiser communication série
        USB->>TM: Test connexion (PING)
        TM->>USB: ACK
        USB->>TA: Test connexion (PING)
        TA->>USB: ACK
        RPI->>USB: Configuration PID
        USB->>TM: SET_PID(kp, ki, kd)
    end

    %% ===== CONTRÔLE MOUVEMENT =====
    rect rgb(240, 255, 240)
        Note over RPI,HW: 🚗 Phase de mouvement
        RPI->>USB: SET_TARGET_POSITION(x, y, θ)
        USB->>TM: Message binaire + CRC8
        TM->>HW: Calcul holonomique
        HW->>TM: Feedback encodeurs
        TM->>USB: UPDATE_ROLLING_BASIS(x, y, θ)
        USB->>RPI: Position actuelle
        
        loop Contrôle en boucle fermée
            TM->>TM: Calcul PID (X, Y, Theta)
            TM->>HW: Commandes moteurs
            HW->>TM: Retour encodeurs
            TM->>USB: UPDATE_ROLLING_BASIS
            USB->>RPI: Position mise à jour
        end
    end

    %% ===== CONTRÔLE ACTUATEURS =====
    rect rgb(255, 248, 240)
        Note over RPI,HW: 🦾 Phase actuateurs
        RPI->>USB: SET_SERVO_ANGLE(pin, angle)
        USB->>TA: Message servo
        TA->>HW: PWM Servo (PCA9685)
        
        RPI->>USB: STEPPER_STEP(pin, steps)
        USB->>TA: Message stepper
        TA->>HW: A4988 Driver
        
        HW->>TA: Switch state
        TA->>USB: SWITCH_STATE_RETURN
        USB->>RPI: État capteur
    end

    %% ===== GESTION ERREURS =====
    rect rgb(255, 240, 240)
        Note over RPI,HW: ⚠️ Gestion d'erreurs
        alt Message corrompu
            USB->>TM: Message avec mauvais CRC
            TM->>USB: NACK(127)
            USB->>RPI: Erreur CRC
        else Message inconnu
            USB->>TA: Type message inconnu
            TA->>USB: UNKNOWN_MSG_TYPE(255)
            USB->>RPI: Erreur protocole
        end
    end
```

## 📨 PROTOCOLE DE COMMUNICATION

### 🔢 **IDs des Messages**

| Direction | Plage | Description | Exemples |
|-----------|-------|-------------|----------|
| RPI → Teensy | 0-126 | Commandes | `SET_TARGET_POSITION(0)` |
| Bidirectionnel | 127 | Erreur | `NACK(127)` |
| Teensy → RPI | 128-255 | Retours | `UPDATE_ROLLING_BASIS(128)` |

### 🔐 **Format des Messages**

```
[HEADER][DATA][CRC8][END_SIGNATURE]
│       │     │     └─ 0xBA,0xDD,0x1C,0xC5
│       │     └─ Vérification intégrité
│       └─ Données binaires
└─ Type de message (byte)
```

### ⚙️ **Messages Principaux**

#### 🚗 **Base Mobile (Teensy Moteur)**
- **SET_TARGET_POSITION(0)** : `{double x, double y, double theta}`
- **SET_PID(1)** : `{float kp, float ki, float kd, byte axis}`
- **UPDATE_ROLLING_BASIS(128)** : `{double x, double y, double theta}`

#### 🦾 **Actuateurs (Teensy Actuateur)**
- **SET_SERVO_ANGLE(7)** : `{byte pin, int angle}`
- **STEPPER_STEP(4)** : `{byte pin, int steps, int speed}`
- **SWITCH_STATE_RETURN(129)** : `{byte pin, bool state}`

### 🔧 **Configuration Communication**
- **Baudrate** : 115200 bps
- **Timeout** : 1000ms
- **CRC** : CRC8 polynomial
- **Buffer** : 256 bytes