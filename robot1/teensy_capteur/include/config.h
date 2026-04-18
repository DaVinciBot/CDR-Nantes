/*
 * config.h - Configuration générale pour le projet de capteur de position
 */

// Default position  
#define START_X 0.0
#define START_Y 0.0
#define START_THETA 0.0

//  SENSORS PINS 
// IMU BNO085 (I2C)  
#define PIN_SDA 18
#define PIN_SCL 19
#define BNO085_RESET_PIN -1  // -1 si relié au Reset de la Teensy ou non utilisé

// Capteur optique PAA5100/PMW3901 (SPI)
#define PAA5100_CS_PIN 10
// MOSI = 11, MISO = 12, SCK = 13 (Obligatoire, géré par le matériel)

#define HAUTEUR_MM 25.0   // Hauteur du capteur par rapport au sol (mm)
#define SEUIL_BRUIT 2     // Seuil de bruit pour filtrage capteur optique
#define OPTICAL_SCALE 0.0423f  // Real robot: counts to mm conversion
#define OPTICAL_MOUNT_ANGLE 0.0f  // Sensor mounting angle (rad)
#define OPTICAL_OFFSET_X 0.0f  // Centrifuge compensation (mm)
#define OPTICAL_OFFSET_Y 0.0f  // Centrifuge compensation (mm)

//  ROBOT GEOMETRY 
#define ROBOT_RADIUS 156.9  // mm - Distance du centre aux roues
#define WHEEL_DIAMETER  60.0 // mm - Diameter effectif (28mm main + 2mm rouleaux)

// PID IDs pour la communication
#define X_PID_ID 0
#define Y_PID_ID 1
#define THETA_PID_ID 2

//  COMMUNICATION 
// Com baudrate
#define BAUDRATE 115200

// Lien série vers teensy_moteur
#define MOTOR_BOARD_SERIAL Serial4
#define MOTOR_BOARD_BAUD 115200
