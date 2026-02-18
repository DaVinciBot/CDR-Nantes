#  USB Communication - C++

> Implémentation C++ du protocole de communication USB pour Teensy 4.1

---

##  Structure du module

```
cpp/
├── library.json          # Configuration PlatformIO
├── include/
│   ├── com.h            # Classe Com principale
│   ├── messages.h       # Énumération des messages
│   └── crc.h            # Calcul CRC8
└── src/
    ├── com.cpp
    ├── messages.cpp
    └── crc.cpp
```

---

##  Classe `Com` (include/com.h)

Gestionnaire de communication série pour Teensy avec validation CRC8.

### Initialisation

```cpp
#include "com.h"
#include "messages.h"

// Dans setup()
Com com(&Serial, true);  // Serial1, Serial2, etc. avec CRC activé
com.init(115200);        // Baudrate
```

### Envoi de messages

#### Message simple

```cpp
// Message sans payload
com.send_message(Messages::RESET_TEENSY);
```

#### Message avec payload

```cpp
// Exemple : UPDATE_ROLLING_BASIS (double x, y, theta)
double x = 123.45;
double y = 67.89;
double theta = 1.57;

byte payload[24];
memcpy(payload, &x, sizeof(double));
memcpy(payload + 8, &y, sizeof(double));
memcpy(payload + 16, &theta, sizeof(double));

com.send_message(Messages::UPDATE_ROLLING_BASIS, payload, 24);
```

### Réception de messages

#### Via callbacks

```cpp
// Définir un handler
void handle_target_position(byte* data, byte size) {
    double x, y, theta;
    memcpy(&x, data, sizeof(double));
    memcpy(&y, data + 8, sizeof(double));
    memcpy(&theta, data + 16, sizeof(double));
    
    Serial.printf("Nouvelle cible: X=%.2f Y=%.2f θ=%.3f\n", x, y, theta);
}

// Enregistrer le callback (dans setup())
com.add_callback(Messages::SET_TARGET_POSITION, handle_target_position);

// Dans loop()
com.handle_callbacks();  // Traite les messages reçus
```

---

##  Énumération `Messages` (include/messages.h)

### Définition

```cpp
enum class Messages : byte {
    // RPI → Teensy (0-126)
    SET_TARGET_POSITION = 0,
    SET_PID = 1,
    SET_ODOMETRIE = 2,
    SET_SERVO_ANGLE_I2C = 3,
    STEPPER_STEP = 4,
    SET_SERVO_ANGLE_DETACH = 5,
    ATTACH_SWITCH = 6,
    SET_SERVO_ANGLE = 7,
    SET_STEPPER_DRIVER_ACTIVATION_STATE = 8,
    
    RESET_TEENSY = 126,
    NACK = 127,
    
    // Teensy → RPI (128-255)
    UPDATE_ROLLING_BASIS = 128,
    SWITCH_STATE_RETURN = 129,
    LIDAR_SCAN_PART1 = 130,
    LIDAR_SCAN_PART2 = 131,
    LIDAR_SCAN_PART3 = 132,
    LIDAR_SCAN_PART4 = 133,
    
    PRINT = 254,
    UNKNOWN_MSG_TYPE = 255
};
```

### Utilisation

```cpp
// Convertir en byte
byte msg_id = static_cast<byte>(Messages::SET_TARGET_POSITION);

// Comparer
if (received_id == static_cast<byte>(Messages::UPDATE_ROLLING_BASIS)) {
    // Traiter
}
```

---

##  CRC8 (include/crc.h)

### Calcul automatique

```cpp
#include "crc.h"

byte data[] = {0x01, 0x02, 0x03};
byte crc = CRC8::compute(data, 3);

// Vérification
bool is_valid = CRC8::verify(data, 3, crc);  // true
```

Le CRC8 utilise le polynôme **0x07** (standard CRC-8-ITU).

### Intégration dans Com

La classe `Com` gère automatiquement le CRC :
-  Ajout du CRC à l'envoi
-  Vérification à la réception
-  Envoi de `NACK` si CRC invalide

---

##  Format des messages

### Structure en mémoire

```
Offset    Contenu           Type        Taille
─────────────────────────────────────────────────
0         ID message        byte        1
1         Payload           byte[]      N
N+1       Taille payload    byte        1
N+2       CRC8              byte        1
N+3       Signature[0]      byte        1  (0xBA)
N+4       Signature[1]      byte        1  (0xDD)
N+5       Signature[2]      byte        1  (0x1C)
N+6       Signature[3]      byte        1  (0xC5)
```

### Constantes

```cpp
// Dans com.h
#define END_SIGNATURE_SIZE 4
const byte END_SIGNATURE[END_SIGNATURE_SIZE] = {0xBA, 0xDD, 0x1C, 0xC5};
```

---

##  Exemple complet : Teensy Moteur

```cpp
#include <Arduino.h>
#include "com.h"
#include "messages.h"

Com com(&Serial, true);

void handle_set_target(byte* data, byte size) {
    if (size != 24) return;  // 3 doubles = 24 bytes
    
    double x, y, theta;
    memcpy(&x, data, 8);
    memcpy(&y, data + 8, 8);
    memcpy(&theta, data + 16, 8);
    
    // Déplacer le robot
    move_to(x, y, theta);
}

void setup() {
    Serial.begin(115200);
    com.init(115200);
    
    // Enregistrer les callbacks
    com.add_callback(Messages::SET_TARGET_POSITION, handle_set_target);
}

void loop() {
    // Traiter les messages reçus
    com.handle_callbacks();
    
    // Envoyer la position courante (10 Hz)
    static unsigned long last_send = 0;
    if (millis() - last_send > 100) {
        double x = get_current_x();
        double y = get_current_y();
        double theta = get_current_theta();
        
        byte payload[24];
        memcpy(payload, &x, 8);
        memcpy(payload + 8, &y, 8);
        memcpy(payload + 16, &theta, 8);
        
        com.send_message(Messages::UPDATE_ROLLING_BASIS, payload, 24);
        last_send = millis();
    }
}
```

---

##  Gestion des erreurs

### Timeout de réception

```cpp
// Dans com.h
#define RECEIVE_TIMEOUT_MS 1000  // 1 seconde

// La classe Com arrête d'attendre après le timeout
```

### Messages inconnus

Si un message avec un ID inconnu est reçu, la Teensy envoie automatiquement :

```cpp
// Envoi automatique par la classe Com
com.send_message(Messages::UNKNOWN_MSG_TYPE, &unknown_id, 1);
```

### CRC invalide

```cpp
// Envoi automatique d'un NACK
com.send_message(Messages::NACK);
```

---

##  Installation (PlatformIO)

### Dans `platformio.ini`

```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino

lib_deps =
    file:///<chemin_absolu>/CDR-Nantes/common/usb_com/cpp
```

### Ou copier manuellement

```bash
cp -r common/usb_com/cpp/* robot1/teensy_moteur/lib/usb_com/
```

---

##  Débogage

### Afficher les messages reçus

```cpp
// Activer le mode debug (définir avant #include "com.h")
#define COM_DEBUG
#include "com.h"

// Affiche dans Serial les messages bruts
```

### Moniteur série

```bash
# PlatformIO
pio device monitor -b 115200

# Arduino IDE
Outils > Moniteur série > 115200 baud
```

---

##  Voir aussi

- [Protocole de communication](../../../PROTOCOLE_COMMUNICATION.md)
- [Implémentation Python](../python/README.md)
- [Documentation Teensy Moteur](../../../robot1/teensy_moteur/CONFIGURATION.md)
- [Documentation complète](../../../Documentation_CDR_Nantes_2026.md)
