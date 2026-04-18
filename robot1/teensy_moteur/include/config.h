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

//MOTEUR W3 — Serial6 (Arrière 0°)
#define W3_SERIAL       Serial6
#define W3_TX_PIN       24
#define W3_RX_PIN       25
#define W3_ADDR         0x01

// ====== SENSOR BOARD (teensy_capteur) COMMUNICATION ======
// teensy_capteur TX1 (pin 16) → teensy_moteur Serial4 RX (pin 15)
// teensy_capteur RX1 (pin 17) ← teensy_moteur Serial4 TX (pin 14) [optional]
#define SENSOR_BOARD_SERIAL  Serial4
#define SENSOR_BOARD_BAUD    115200

// MKS CONFIGURATION 
#define MKS_BAUDRATE 115200
#define MKS_MSTEP 32
#define MKS_MAX_RPM 100.0
#define MKS_ACC 5
#define MKS_COUNTS_PER_REV 16384.0

#define COUNTS_TO_MM ((WHEEL_DIAMETER * PI) / MKS_COUNTS_PER_REV)

// Vitesse max utile en RPM (consigne logicielle)
#define MAX_SPEED_RPM 1500.0

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
