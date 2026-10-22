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
// IMU BNO085 (I2C, Wire)  
#define PIN_SDA 18
#define PIN_SCL 19
#define BNO085_RESET_PIN -1  // -1 si relié au Reset de la Teensy ou non utilisé
// Report period of the game rotation vector: one heading per control tick.
// A faster report fills the event queue, a slower one leaves ticks without.
#define IMU_REPORT_PERIOD_US ASSERVISSEMENT_FREQUENCY

// Optical odometry sensor SparkFun Qwiic OTOS (I2C)
// Bus NOT fixed yet with the electronics: Wire (SDA 18 / SCL 19, shared with
// the BNO085), Wire1 (SDA 17 / SCL 16) or Wire2 (SDA 25 / SCL 24). Each bus
// uses its default pins.
#define OTOS_WIRE            Wire1
#define OTOS_I2C_CLOCK_HZ    400000
// Scalars written at every boot (the sensor forgets them at power-off);
// refused outside [0.95, 1.05]. To calibrate on the playing surface.
#define OTOS_LINEAR_SCALAR   1.0f
#define OTOS_ANGULAR_SCALAR  1.0f
// Angle from the robot x axis to the sensor x axis (rad). The lever arm from
// the centre of rotation is NOT set here: the EKF compensates it.
#define OTOS_MOUNT_ANGLE_RAD 0.0f


// ── Debug verbose ─────────────────────────────────────────────────────────────
// Décommenter pour activer tous les printf de debug
// #define DEBUG_VERBOSE
// #define DEBUG_RS485

// MKS CONFIGURATION 
#define MKS_BAUDRATE 115200
#define MKS_MSTEP 32
#define MKS_MAX_RPM 350.0
// Internal ramp of the MKS drivers, a safety net only: the acceleration limiter
// of the control loop shapes the command, the drivers must follow it. Assuming
// the manual's ramp (+1 rpm every (256 - acc) x 50 us), acc = 256 - 20000 / rpm_per_s:
// 225 gives ~640 rpm/s, twice MAX_WHEEL_ACCEL_RPM_S. Also used by stop().
#define MKS_ACC 225
#define MKS_COUNTS_PER_REV 16384.0

#define COUNTS_TO_MM ((WHEEL_DIAMETER * PI) / MKS_COUNTS_PER_REV)

// Software wheel speed ceiling, rpm. 1 m/s needs 318.3 rpm with 60 mm wheels;
// the extra ~30 rpm leaves room for rotation during a full-speed translation
// (the proportional normalisation scales the whole command down as soon as one
// wheel exceeds this).
#define MAX_SPEED_RPM 350.0

// Wheel acceleration limit, rpm/s: 1 m/s^2 at the wheel (318.3) rounded up.
// Applied to the three wheels with the same factor, so the direction holds.
#define MAX_WHEEL_ACCEL_RPM_S 320.0

//  ROBOT GEOMETRY
// WARNING: INDEPENDENT homonym of ROBOT_RADIUS_MM in robot1/rasp/world.py
// (120 mm). Here: a physical quantity, center to wheel axis, used by the
// holonomic kinematics and the rotational odometry. There: the obstacle
// inflation radius for the A*. NEVER "align" them.
// Value still to be calibrated on the real chassis (160.0 here, 156.9 in tag
// archive/precoupe-2026-05, ~158 measured by mechanics).
#define ROBOT_RADIUS 160.0  // mm - Distance du centre aux roues
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
#define ASSERVISSEMENT_FREQUENCY 15000  // 15 ms = 66.7 Hz

// Fréquence d'exécution des mouvements moteurs (en microsecondes)
#define MOVEMENT_FREQUENCY 5000         // 5ms = 200Hz

//  POSE ESTIMATOR
// Both estimators are compiled in; this one runs at boot. The other can be
// selected at run time, robot at rest (Holonomic_Basis::select_estimator).
#define POSE_ESTIMATOR_COMPLEMENTARY 0  // 20 % encoders / 80 % optical (2026)
#define POSE_ESTIMATOR_EKF6          1  // 6-state EKF, lib/kalman
#define POSE_ESTIMATOR_DEFAULT POSE_ESTIMATOR_COMPLEMENTARY

//  COMMUNICATION 
// Com baudrate
#define BAUDRATE 115200

// WATCHDOG & TIMEOUT MOUVEMENT
#define WATCHDOG_TIMEOUT_MS     3000   // 3s sans message = crash Python
#define ROBOT_MAX_SPEED_MM_S    1000.0 // Translation speed target, mm/s
// Rotation speed target, rad/s. A chosen command, below the physical ceiling
// (MAX_SPEED_RPM in pure rotation: ~6.9 rad/s with ROBOT_RADIUS = 160).
#define ROBOT_MAX_RAD_S         3.0
#define ROBOT_MAX_ACCEL_MM_S2   1000.0 // Translation acceleration, mm/s^2 (braking distance)
#define MOVEMENT_TIMEOUT_MARGIN 2.0    // Multiplicateur sécurité (x2 le temps théorique)
#define MOVEMENT_TIMEOUT_MIN_MS 2000   // Minimum 2s même pour petits déplacements