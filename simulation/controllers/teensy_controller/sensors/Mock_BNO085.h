// Mock_BNO085.h - Émulation BNO085 pour Webots avec Game Rotation Vector
#pragma once
#include <cstdio>

#ifdef WEBOTS_SIMULATION
#include <webots/robot.h>
#include <webots/inertial_unit.h>

// Forward declaration pour TwoWire (non utilisé en simulation mais requis pour la signature)
class TwoWire;
#else
#include <Adafruit_BNO08x.h>
#endif

#ifdef WEBOTS_SIMULATION

// ============================================================================
// CONSTANTES SH2 (compatibles avec la vraie librairie)
// ============================================================================
#define SH2_GAME_ROTATION_VECTOR 0x08  // Game Rotation Vector (Gyro + Accel)
#define SH2_ARVR_STABILIZED_RV   0x28  // Ancien (avec Magnéto) - NE PLUS UTILISER

// ============================================================================
// STRUCTURES (compatibles avec Adafruit_BNO08x)
// ============================================================================

// Structure quaternion (compatible SH2)
struct sh2_Quaternion_t {
    float i = 0.0f;
    float j = 0.0f;
    float k = 0.0f;
    float real = 1.0f;
};

// Structure Game Rotation Vector
struct sh2_GameRotationVector_t {
    float i;
    float j;
    float k;
    float real;
};

// Structure ARVR Stabilized (ancienne, gardée pour compatibilité)
struct sh2_ARVRStabilizedRV_t {
    float i;
    float j;
    float k;
    float real;
};

// Union principale (comme dans la vraie librairie SH2)
union sh2_SensorData_t {
    sh2_GameRotationVector_t gameRotationVector;
    sh2_ARVRStabilizedRV_t arvrStabilizedRV;
};

// Structure complète SensorValue (comme Adafruit)
struct sh2_SensorValue_t {
    uint8_t sensorId;           // ID du rapport (SH2_GAME_ROTATION_VECTOR, etc.)
    uint8_t sequence;
    uint8_t status;
    uint64_t timestamp;
    sh2_SensorData_t un;        // Union des données
};

// ============================================================================
// CLASSE MOCK BNO085
// ============================================================================
class Adafruit_BNO08x {
private:
    WbDeviceTag imu_sensor;
    double yaw_offset;          // Offset calibration (direction "avant" au démarrage)
    bool calibrated;

public:
    Adafruit_BNO08x(int reset_pin = -1) : yaw_offset(0.0), calibrated(false) {
        imu_sensor = wb_robot_get_device("IMU");
        if (imu_sensor) {
            wb_inertial_unit_enable(imu_sensor, 10); // 10ms = 100Hz
            printf("✅ Mock BNO08x: IMU Webots activée\n");
        } else {
            printf("❌ Mock BNO08x: IMU introuvable dans Webots\n");
        }
    }

    bool begin_I2C(uint8_t i2c_addr = 0x4A, TwoWire *wire = nullptr) {
        return (imu_sensor != 0);
    }

    void enableReport(uint16_t report_type, uint32_t interval_us = 10000) {
        // En simulation, on ignore (Webots gère déjà la fréquence)
        if (report_type == SH2_GAME_ROTATION_VECTOR) {
            printf("📡 Mock: Game Rotation Vector activé (simulation)\n");
        }
    }

    // ✅ MÉTHODE PRINCIPALE : getSensorEvent (compatible avec le vrai BNO08x)
    bool getSensorEvent(sh2_SensorValue_t* value) {
        if (!imu_sensor || !value) return false;

        // Lecture Webots (Euler angles)
        const double* rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
        double yaw = rpy[2]; // Yaw en radians

        // Calibration au premier appel
        if (!calibrated) {
            yaw_offset = yaw;
            calibrated = true;
            printf("🧭 Mock IMU: Calibration initiale (yaw_offset = %.3f rad)\n", yaw_offset);
        }

        // Yaw relatif (Game Rotation Vector : 0° = position de départ)
        double yaw_relative = yaw - yaw_offset;

        // Normalisation [-π, +π]
        while (yaw_relative >  M_PI) yaw_relative -= 2.0 * M_PI;
        while (yaw_relative < -M_PI) yaw_relative += 2.0 * M_PI;

        // Conversion Euler -> Quaternion (rotation pure autour de Z)
        value->sensorId = SH2_GAME_ROTATION_VECTOR;
        value->un.gameRotationVector.i = 0.0f;
        value->un.gameRotationVector.j = 0.0f;
        value->un.gameRotationVector.k = (float)sin(yaw_relative / 2.0);
        value->un.gameRotationVector.real = (float)cos(yaw_relative / 2.0);

        return true;
    }

    // ✅ MÉTHODE LEGACY : getQuat (pour compatibilité avec ancien code)
    bool getQuat(sh2_Quaternion_t &quat) {
        sh2_SensorValue_t value;
        if (getSensorEvent(&value)) {
            quat.i = value.un.gameRotationVector.i;
            quat.j = value.un.gameRotationVector.j;
            quat.k = value.un.gameRotationVector.k;
            quat.real = value.un.gameRotationVector.real;
            return true;
        }
        return false;
    }

    // Méthode directe pour récupérer le yaw
    double getYaw() {
        if (!imu_sensor) return 0.0;
        const double* rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
        return rpy[2] - yaw_offset;
    }

    // Compatibilité API
    bool wasReset() { return false; }
    void hardwareReset() {}
};

// Alias pour compatibilité avec le nom utilisé dans teensy_controller.cpp
typedef Adafruit_BNO08x Adafruit_BNO085;

#endif // WEBOTS_SIMULATION