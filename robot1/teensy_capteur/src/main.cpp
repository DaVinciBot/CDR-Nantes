/**
 * teensy_capteur - Sensor Board (Teensy 4.1)
 * 
 * Dedicated sensor processing:
 * - PAA5100 Optical Flow Sensor (SPI)
 * - BNO085 9-Axis IMU (I2C)
 * 
 * Sends processed sensor data (dx, dy, dtheta) to teensy_moteur via MOTOR_BOARD_SERIAL
 * Format: [0xAA] [dx:float] [dy:float] [dtheta:float] [0xBB] (14 bytes)
 * Rate: 100 Hz
 */

#include <Arduino.h>
#include <config.h>
#include <Wire.h>
#include <SPI.h>
#include <cmath>

// Optical sensor library (Bitcraze PMW3901)
#include <Bitcraze_PMW3901.h>

// IMU library (Adafruit BNO085)
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// ============= MATH CONSTANTS =============
#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif
#define PI M_PI

// ============= PROTOCOL CONSTANTS =============
#define SYNC_START 0xAA
#define SYNC_END 0xBB

// ============= SENSOR CONSTANTS (from config.h) =============
// Optical flow sensor calibration (adjust for real robot vs simulation)
#define OPTICAL_SCALE 0.0423f  // Real robot: counts to mm conversion
#define OPTICAL_MOUNT_ANGLE 0.0f  // Sensor mounting angle (rad)
#define OPTICAL_OFFSET_X 0.0f  // Centrifuge compensation (mm)
#define OPTICAL_OFFSET_Y 0.0f  // Centrifuge compensation (mm)

// ============= SENSOR OBJECTS =============
Bitcraze_PMW3901 pmw3901(PAA5100_CS_PIN);
Adafruit_BNO08x bno085(BNO085_RESET_PIN);

// ============= SENSOR STATE =============
struct SensorState {
    bool optical_ready;
    bool imu_ready;
    float imu_yaw_offset;
    float previous_yaw;
    bool first_yaw_read;
    uint32_t optical_valid_count;
    uint32_t optical_outlier_count;
};

SensorState sensors = {false, false, 0.0f, 0.0f, true, 0, 0};

// ============= COMMUNICATION HELPERS =============
union FloatBytes {
    float value;
    uint8_t bytes[4];
};

// ============= SENSOR INITIALIZATION =============

/**
 * Initialize optical flow sensor (PAA5100)
 * Returns: true if successful, false if sensor not responding
 */
bool init_optical_sensor() {
    // Use 100ms timeout to avoid blocking setup
    uint32_t start_time = millis();
    
    while (!pmw3901.begin() && (millis() - start_time) < 100) {
        delay(10);
    }
    
    if (pmw3901.begin()) {
        Serial.println("[OK] Optical sensor (PAA5100) initialized");
        return true;
    } else {
        Serial.println("[ERR] Optical sensor (PAA5100) not responding!");
        return false;
    }
}

/**
 * Initialize IMU (BNO085)
 * Returns: true if successful, false if sensor not responding
 */
bool init_imu_sensor() {
    // Ensure I2C is running at Fast Mode
    Wire.setClock(400000);
    
    if (!bno085.begin_I2C()) {
        Serial.println("[ERR] IMU (BNO085) I2C begin failed!");
        return false;
    }
    
    // Enable game rotation vector at 100Hz
    bno085.enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 10000us = 100Hz
    
    // Calibration: capture initial yaw offset (500ms timeout)
    Serial.println("[INFO] IMU calibrating (500ms)...");
    uint32_t calib_start = millis();
    sh2_SensorValue_t sv;
    bool calibrated = false;
    
    while ((millis() - calib_start) < 500) {
        if (bno085.getSensorEvent(&sv)) {
            if (sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
                // Quaternion to Yaw conversion (formula from holonomic_basis)
                float r = sv.un.gameRotationVector.real;
                float i = sv.un.gameRotationVector.i;
                float j = sv.un.gameRotationVector.j;
                float k = sv.un.gameRotationVector.k;
                
                // atan2(2.0f*(r*k + i*j), 1.0f - 2.0f*(j*j + k*k))
                sensors.imu_yaw_offset = atan2(2.0f*(r*k + i*j), 1.0f - 2.0f*(j*j + k*k));
                calibrated = true;
                break;
            }
        }
        delay(10);
    }
    
    if (calibrated) {
        Serial.print("[OK] IMU calibrated, yaw_offset=");
        Serial.println(sensors.imu_yaw_offset);
    } else {
        Serial.println("[WARN] IMU calibration timeout, using offset=0.0");
        sensors.imu_yaw_offset = 0.0f;
    }
    
    return true;
}

// ============= SENSOR READING FUNCTIONS =============

/**
 * Read optical sensor and convert to mm displacement (robot frame)
 * Based on holonomic_basis::update_optical_odometry()
 */
void read_optical_sensor(float& dx, float& dy, float dtheta = 0.0f) {
    if (!sensors.optical_ready) {
        dx = 0.0f;
        dy = 0.0f;
        return;
    }
    
    int16_t delta_x = 0, delta_y = 0;
    pmw3901.readMotionCount(&delta_x, &delta_y);
    
    // Ignore first read (often spurious)
    static bool first_read = true;
    if (first_read) {
        first_read = false;
        dx = 0.0f;
        dy = 0.0f;
        return;
    }
    
    // Convert counts to mm
    float dx_mm = (float)delta_x * OPTICAL_SCALE;
    float dy_mm = (float)delta_y * OPTICAL_SCALE;
    
    // Apply sensor mount angle transformation
    float cos_mnt = cos(OPTICAL_MOUNT_ANGLE);
    float sin_mnt = sin(OPTICAL_MOUNT_ANGLE);
    dx = dx_mm * cos_mnt - dy_mm * sin_mnt;
    dy = dx_mm * sin_mnt + dy_mm * cos_mnt;

    // Centrifugal compensation from robot rotation around sensor offset.
    dx -= -OPTICAL_OFFSET_Y * dtheta;
    dy -=  OPTICAL_OFFSET_X * dtheta;
    
    // Outlier rejection: magnitude > 15mm per cycle = impossible (max ~3mm at 100Hz)
    float magnitude = sqrt(dx*dx + dy*dy);
    if (magnitude > 15.0f) {
        sensors.optical_outlier_count++;
        dx = 0.0f;
        dy = 0.0f;
        return;
    }
    
    // Noise filter: magnitude < 2mm = sensor noise, ignore
    if (magnitude < 2.0f) {
        dx = 0.0f;
        dy = 0.0f;
        return;
    }
    
    sensors.optical_valid_count++;
}

/**
 * Read IMU quaternion and convert to yaw angle
 * Returns: delta-yaw (change in heading, radians)
 * Based on holonomic_basis quaternion conversion formula
 */
float read_imu_yaw() {
    if (!sensors.imu_ready) {
        return 0.0f;
    }
    
    sh2_SensorValue_t sv;
    if (!bno085.getSensorEvent(&sv)) {
        return 0.0f;
    }
    
    if (sv.sensorId != SH2_GAME_ROTATION_VECTOR) {
        return 0.0f;
    }
    
    // Quaternion to Yaw conversion (same formula as holonomic_basis)
    float r = sv.un.gameRotationVector.real;
    float i = sv.un.gameRotationVector.i;
    float j = sv.un.gameRotationVector.j;
    float k = sv.un.gameRotationVector.k;
    
    float yaw = atan2(2.0f*(r*k + i*j), 1.0f - 2.0f*(j*j + k*k));
    yaw -= sensors.imu_yaw_offset;
    
    // First read: initialize baseline
    if (sensors.first_yaw_read) {
        sensors.first_yaw_read = false;
        sensors.previous_yaw = yaw;
        return 0.0f;
    }
    
    // Calculate delta yaw, normalized to [-PI, PI]
    float dtheta = yaw - sensors.previous_yaw;
    
    // Normalize angle to [-PI, PI]
    if (dtheta > PI) {
        dtheta -= 2.0f * PI;
    } else if (dtheta < -PI) {
        dtheta += 2.0f * PI;
    }
    
    sensors.previous_yaw = yaw;
    return dtheta;
}

// ============= PACKET TRANSMISSION =============

/**
 * Send sensor data packet to teensy_moteur via MOTOR_BOARD_SERIAL
 * Format: [0xAA] [dx:float] [dy:float] [dtheta:float] [0xBB]
 */
void send_sensor_data(float dx, float dy, float dtheta) {
    FloatBytes fbx, fby, fbt;
    fbx.value = dx;
    fby.value = dy;
    fbt.value = dtheta;
    
    MOTOR_BOARD_SERIAL.write(SYNC_START);
    MOTOR_BOARD_SERIAL.write(fbx.bytes, 4);
    MOTOR_BOARD_SERIAL.write(fby.bytes, 4);
    MOTOR_BOARD_SERIAL.write(fbt.bytes, 4);
    MOTOR_BOARD_SERIAL.write(SYNC_END);
}

// ============= SETUP & LOOP =============

void setup() {
    Serial.begin(115200);  // USB debug
    MOTOR_BOARD_SERIAL.begin(MOTOR_BOARD_BAUD);
    
    delay(1000);
    Serial.println("\n=== teensy_capteur STARTING ===");
    
    // Initialize SPI for optical sensor
    SPI.begin();
    delay(100);
    sensors.optical_ready = init_optical_sensor();
    
    // Initialize I2C for IMU
    Wire.begin();
    delay(100);
    sensors.imu_ready = init_imu_sensor();
    
    Serial.println("=== Initialization Complete ===\n");
}

void loop() {
    // Read sensors
    float dx = 0.0f, dy = 0.0f, dtheta = 0.0f;
    
    dtheta = read_imu_yaw();
    read_optical_sensor(dx, dy, dtheta);
    
    // Send to teensy_moteur
    send_sensor_data(dx, dy, dtheta);
    
    // Debug output (~10Hz)
    static uint32_t debug_cnt = 0;
    if (++debug_cnt >= 10) {  // 10 cycles × 10ms = 100ms = 10Hz
        debug_cnt = 0;
        Serial.print("dx=");
        Serial.print(dx, 3);
        Serial.print(" dy=");
        Serial.print(dy, 3);
        Serial.print(" dtheta=");
        Serial.print(dtheta, 4);
        Serial.print(" | opt_valid=");
        Serial.print(sensors.optical_valid_count);
        Serial.print(" opt_outlier=");
        Serial.println(sensors.optical_outlier_count);
    }
    
    delay(10);  // 100Hz
}