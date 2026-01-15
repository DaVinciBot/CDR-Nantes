/*
 * This file is dedicated to the configuration of the robot: all the constants
 * and the pinout are defined here.
 * ADAPTED FOR HOLONOMIC 3-WHEEL BASE WITH STEPPER MOTORS (NO ENCODERS)
 */

// Default position  
#define START_X 0.0
#define START_Y 0.0
#define START_THETA 0.0

// ========== STEPPER MOTOR 1 (Front - 0°) ==========
#define W1_STEP_PIN 2
#define W1_DIR_PIN 3
#define W1_ENABLE_PIN 4

// ========== STEPPER MOTOR 2 (Back-Left - 120°) ==========
#define W2_STEP_PIN 5
#define W2_DIR_PIN 6
#define W2_ENABLE_PIN 7

// ========== STEPPER MOTOR 3 (Back-Right - 240°) ==========
#define W3_STEP_PIN 8
#define W3_DIR_PIN 9
#define W3_ENABLE_PIN 10

// ========== STEPPER MOTORS CONFIGURATION ==========
// Stepper motor specifications
#define STEPS_PER_REVOLUTION 200      // 200 steps = 1.8° per step (standard NEMA 17)
#define MICROSTEPS 32                  // Microstepping (1, 2, 4, 8, 16, 32) - RÉDUIT POUR TEST
#define TOTAL_STEPS_PER_REV (STEPS_PER_REVOLUTION * MICROSTEPS)

// Speed and acceleration limits
#define MAX_SPEED 25000              // steps/second - RÉDUIT POUR TEST
#define MAX_ACCELERATION 12500.0        // steps/second² - RÉDUIT POUR TEST

// ========== ROBOT GEOMETRY ==========
#define ROBOT_RADIUS 156.9  // mm - Distance du centre aux roues
#define WHEEL_DIAMETER  60.0 // mm - Diameter of the wheels

// ========== PID CONTROLLERS ==========
// PID X (déplacement horizontal)
#define KP_X 1.2
#define KI_X 0.02
#define KD_X 0.10

// PID Y (déplacement vertical)
#define KP_Y 1.2
#define KI_Y 0.02
#define KD_Y 0.10

// PID THETA (rotation)
#define KP_THETA 4.0
#define KI_THETA 0.1
#define KD_THETA 0.20

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