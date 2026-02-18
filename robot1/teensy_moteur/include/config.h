/*
 * This file is dedicated to the configuration of the robot: all the constants
 * and the pinout are defined here.
 * ADAPTED FOR HOLONOMIC 3-WHEEL BASE WITH STEPPER MOTORS (NO ENCODERS)
 */

// Default position  
#define START_X 0.0
#define START_Y 0.0
#define START_THETA 0.0

// ========== STEPPER MOTOR 1 (Haut Droite - 120°) ==========
#define W1_STEP_PIN 2
#define W1_DIR_PIN 3
#define W1_ENABLE_PIN 4

// ========== STEPPER MOTOR 2 (Haut Gauche - 240°) ==========
#define W2_STEP_PIN 5
#define W2_DIR_PIN 6
#define W2_ENABLE_PIN 7

// ========== STEPPER MOTOR 3 (Arrière - 0°) ==========
#define W3_STEP_PIN 8
#define W3_DIR_PIN 9
#define W3_ENABLE_PIN 11  // Modifié (était 10, conflit avec CS_PIN capteur optique)

// ========== STEPPER MOTORS CONFIGURATION ==========
// Stepper motor specifications
#define STEPS_PER_REVOLUTION 200      // 200 steps = 1.8° per step (standard NEMA 17)
#define MICROSTEPS 32                  // Microstepping (1, 2, 4, 8, 16, 32) - RÉDUIT POUR TEST
#define TOTAL_STEPS_PER_REV (STEPS_PER_REVOLUTION * MICROSTEPS)

// Speed and acceleration limits
#define MAX_SPEED 20000              // steps/second - Réduit pour éviter glissement
#define MAX_ACCELERATION 10000.0         // steps/second² - Réduit pour accélération douce

// ========== ROBOT GEOMETRY ==========
#define ROBOT_RADIUS 156.9  // mm - Distance du centre aux roues
#define WHEEL_DIAMETER  60.0 // mm - Diameter effectif (28mm main + 2mm rouleaux)

// ========== PID CONTROLLERS ==========
// PID X (déplacement horizontal) - Augmenté pour corriger les dérives
#define KP_X 6.0
#define KI_X 0.0  
#define KD_X 0.15 

// PID Y (déplacement vertical) - Augmenté pour corriger les dérives
#define KP_Y 6.0
#define KI_Y 0.0  
#define KD_Y 0.15

// PID THETA (rotation)
#define KP_THETA 120.0
#define KI_THETA 3.0
#define KD_THETA 0.5

// PID IDs pour la communication
#define X_PID_ID 0
#define Y_PID_ID 1
#define THETA_PID_ID 2

// ========== CONTROL LOOP ==========
// Asservissement échantillonnage fréquence (en microsecondes)
#define ASSERVISSEMENT_FREQUENCY 10000  // 10ms = 100Hz

// Fréquence d'exécution des mouvements moteurs (en microsecondes)
#define MOVEMENT_FREQUENCY 5000         // 5ms = 200Hz

// ========== COMMUNICATION ==========
// Com baudrate
#define BAUDRATE 115200

// ========== SENSORS PINS ==========
// IMU BNO085 (I2C)
#define PIN_SDA 18
#define PIN_SCL 19
#define BNO085_RESET_PIN -1  // -1 si relié au Reset de la Teensy ou non utilisé

// Capteur optique PAA5100/PMW3901 (SPI)
#define PAA5100_CS_PIN 10
#define HAUTEUR_MM 25.0   // Hauteur du capteur par rapport au sol (mm)
#define SEUIL_BRUIT 2     // Seuil de bruit pour filtrage capteur optique

// ========== PWM (non utilisé pour steppers, mais gardé pour compatibilité) ==========
#define PWM_FREQUENCY 40000
#define MAX_PWM 240