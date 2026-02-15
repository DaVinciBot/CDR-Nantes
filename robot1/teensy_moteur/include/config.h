/*
 * This file is dedicated to the configuration of the robot: all the constants
 * and the pinout are defined here.
 * ADAPTED FOR HOLONOMIC 3-WHEEL BASE WITH STEPPER MOTORS (NO ENCODERS)
 */

// Default position  
#define START_X 0.0
#define START_Y 0.0
#define START_THETA 0.0

// ========== STEPPER MOTOR 1 (Front-Right - 30°) ==========
#define W1_STEP_PIN 2
#define W1_DIR_PIN 3
#define W1_ENABLE_PIN 4

// ========== STEPPER MOTOR 2 (Front-Left - 150°) ==========
#define W2_STEP_PIN 5
#define W2_DIR_PIN 6
#define W2_ENABLE_PIN 7

// ========== STEPPER MOTOR 3 (Back - 270°) ==========
#define W3_STEP_PIN 8
#define W3_DIR_PIN 9
#define W3_ENABLE_PIN 10

// ========== STEPPER MOTORS CONFIGURATION ==========
// Stepper motor specifications
#define STEPS_PER_REVOLUTION 200      // 200 steps = 1.8° per step (standard NEMA 17)
#define MICROSTEPS 32                  // Microstepping (1, 2, 4, 8, 16, 32) - RÉDUIT POUR TEST
#define TOTAL_STEPS_PER_REV (STEPS_PER_REVOLUTION * MICROSTEPS)

// Speed and acceleration limits
#define MAX_SPEED 35000.0            // steps/second - Augmenté pour éviter saturation en diagonale
#define MAX_ACCELERATION 25000.0     // steps/second² - Augmenté pour meilleure réactivité

// ========== ROBOT GEOMETRY ==========
#define ROBOT_RADIUS 156.9  // mm - Distance du centre aux roues
#define WHEEL_DIAMETER  60.0 // mm - Diameter effectif (28mm main + 2mm rouleaux)

// ========== PID CONTROLLERS ==========
// PID X (déplacement horizontal) - Réduit pour mouvements plus doux
#define KP_X 4.1
#define KI_X 0.0  
#define KD_X 0.1 

// PID Y (déplacement vertical) - Réduit pour mouvements plus doux
#define KP_Y 4.1
#define KI_Y 0.0  
#define KD_Y 0.1

// PID THETA (rotation) - Réduit pour rotations plus fluides
#define KP_THETA 120
#define KI_THETA 3.0
#define KD_THETA 0.2

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

// ========== PWM (non utilisé pour steppers, mais gardé pour compatibilité) ==========
#define PWM_FREQUENCY 40000
#define MAX_PWM 240