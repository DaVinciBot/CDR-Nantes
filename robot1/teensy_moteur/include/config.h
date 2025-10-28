/*
 * This file is dedicated to the configuration of the robot: all the constants
 * and the pinout are defined here.
 * ADAPTED FOR HOLONOMIC 3-WHEEL BASE
 */

// Default position  
#define START_X 0.0
#define START_Y 0.0
#define START_THETA 0.0

// Motor Wheel 1 (Front - 0°)
#define W1_ENCA 12
#define W1_ENCB 11
#define W1_PWM 5
#define W1_IN2 3
#define W1_IN1 4

// Motor Wheel 2 (Back-Left - 120°)
#define W2_ENCA 14
#define W2_ENCB 13
#define W2_PWM 2
#define W2_IN2 1
#define W2_IN1 0

// Motor Wheel 3 (Back-Right - 240°)
#define W3_ENCA 16  // NOUVEAU - A définir selon ton PCB
#define W3_ENCB 15  // NOUVEAU - A définir selon ton PCB
#define W3_PWM 6    // NOUVEAU - A définir selon ton PCB
#define W3_IN2 7    // NOUVEAU - A définir selon ton PCB
#define W3_IN1 8    // NOUVEAU - A définir selon ton PCB

// Creation Holonomic Basis
// Motor
#define MAX_PWM 240

// Encoder
#define ENCODER_RESOLUTION 1024
#define ROBOT_RADIUS 15.0  // cm - Distance du centre aux roues (à mesurer sur ton robot)
#define WHEEL_DIAMETER 5.9 // cm - Diameter of the wheels

// PIDs - 3 PID pour X, Y et THETA
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

// PWM frequency
#define PWM_FREQUENCY 40000

// Asservissement échantillonnage fréquence
#define ASSERVISSEMENT_FREQUENCY 5000

// Com baudrate
#define BAUDRATE 115200