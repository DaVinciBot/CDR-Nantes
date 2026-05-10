/*
 * This file is dedicated to the configuration of the robot: all the constants
 * and the pinout are defined here.
 * ADAPTED FOR HOLONOMIC 3-WHEEL BASE WITH MKS SERVO57D OVER RS485
 */

// Default position  
#define START_X 0.0
#define START_Y 0.0
#define START_THETA 0.0

//  MOTEUR W1 — Serial1 (Haut Droite 120°) 
#define W1_SERIAL       Serial1
#define W1_TX_PIN       1
#define W1_RX_PIN       0
#define W1_ADDR         0x01   // Adresse fixe — 1 moteur par port

// MOTEUR W2 — Serial2 (Haut Gauche 240°) 
#define W2_SERIAL       Serial2
#define W2_TX_PIN       8
#define W2_RX_PIN       7
#define W2_ADDR         0x01

//MOTEUR W3 — Serial3 (Arrière 0°)
#define W3_SERIAL       Serial6
#define W3_TX_PIN       24
#define W3_RX_PIN       25
#define W3_ADDR         0x01

//  SENSORS PINS 
// IMU BNO085 (I2C)  
#define PIN_SDA 18
#define PIN_SCL 19
#define BNO085_RESET_PIN -1  // -1 si relié au Reset de la Teensy ou non utilisé


// ── Capteur optique PAA5100JE 
#define OPTICAL_SCALE       0.0423   // Counts → mm (à calibrer sur terrain réel)

// Capteur optique PAA5100/PMW3901 (SPI)
#define PAA5100_CS_PIN 10
// MOSI = 11, MISO = 12, SCK = 13 (Obligatoire, géré par le matériel)

#define HAUTEUR_MM 25.0   // Hauteur du capteur par rapport au sol (mm)
#define SEUIL_BRUIT 2     // Seuil de bruit pour filtrage capteur optique


// ── Debug verbose ─────────────────────────────────────────────────────────────
// Décommenter pour activer tous les printf de debug
// #define DEBUG_VERBOSE
// #define DEBUG_RS485

// MKS CONFIGURATION 
#define MKS_BAUDRATE 115200
#define MKS_MSTEP 32
#define MKS_MAX_RPM 100.0
#define MKS_ACC 5
#define MKS_COUNTS_PER_REV 16384.0

#define COUNTS_TO_MM ((WHEEL_DIAMETER * PI) / MKS_COUNTS_PER_REV)

// Vitesse max utile en RPM (consigne logicielle)
#define MAX_SPEED_RPM 100.0

//  ROBOT GEOMETRY 
#define ROBOT_RADIUS 156.9  // mm - Distance du centre aux roues
#define WHEEL_DIAMETER  60.0 // mm - Diameter effectif (28mm main + 2mm rouleaux)

//  PID CONTROLLERS 
// PID X (déplacement horizontal) - Augmenté pour corriger les dérives
#define KP_X 1.0
#define KI_X 0.0  
#define KD_X 0.0

// PID Y (déplacement vertical) - Augmenté pour corriger les dérives
#define KP_Y 1.0
#define KI_Y 0.0  
#define KD_Y 0.0

// PID THETA (rotation)
#define KP_THETA 1.0
#define KI_THETA 0.0
#define KD_THETA 0.0

// PID IDs pour la communication
#define X_PID_ID 0
#define Y_PID_ID 1
#define THETA_PID_ID 2

//  CONTROL LOOP 
// Asservissement échantillonnage fréquence (en microsecondes)
#define ASSERVISSEMENT_FREQUENCY 10000  // 10ms = 100Hz

// Fréquence d'exécution des mouvements moteurs (en microsecondes)
#define MOVEMENT_FREQUENCY 5000         // 5ms = 200Hz

//  COMMUNICATION 
// Com baudrate
#define BAUDRATE 115200

// WATCHDOG & TIMEOUT MOUVEMENT
#define WATCHDOG_TIMEOUT_MS     3000   // 3s sans message = crash Python
#define ROBOT_MAX_SPEED_MM_S    200.0  // Vitesse max réelle estimée (mm/s)
#define ROBOT_MAX_RAD_S         2.0    // Vitesse angulaire max (rad/s)
#define MOVEMENT_TIMEOUT_MARGIN 2.0    // Multiplicateur sécurité (x2 le temps théorique)
#define MOVEMENT_TIMEOUT_MIN_MS 2000   // Minimum 2s même pour petits déplacements