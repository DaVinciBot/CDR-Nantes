// Mock_BNO085.h - Émulation de l'IMU BNO085 pour Webots
// Ce fichier remplace Adafruit_BNO085.h sur le robot réel
#pragma once
#include <cstdio>

#ifdef WEBOTS_SIMULATION
#include <webots/robot.h>
#include <webots/inertial_unit.h>
#else
#include <Adafruit_BNO085.h>
#endif

#ifdef WEBOTS_SIMULATION
// Structure pour quaternions (compatible Adafruit)
struct sh2_Quaternion_t {
    float i = 0.0f;
    float j = 0.0f;
    float k = 0.0f;
    float real = 1.0f;
};

class Adafruit_BNO085 {
private:
    WbDeviceTag imu_sensor;

public:
    Adafruit_BNO085(int reset_pin = -1) {
        // Récupérer l'InertialUnit de Webots
        imu_sensor = wb_robot_get_device("IMU");
        if (imu_sensor) {
            wb_inertial_unit_enable(imu_sensor, 32); // 32ms
            printf("✅ Mock BNO085: IMU Webots activée\n");
        } else {
            printf("❌ Mock BNO085: IMU introuvable dans Webots\n");
        }
    }

    bool begin_I2C(uint8_t i2c_addr = 0x4A) {
        return (imu_sensor != 0);
    }

    void enableReport(uint16_t report_type, uint32_t interval_us = 10000) {
        // Stub : Webots gère déjà la fréquence avec wb_inertial_unit_enable()
    }

    bool getQuat(sh2_Quaternion_t &quat) {
        if (!imu_sensor) {
            return false;
        }

        // Webots retourne les angles d'Euler [roll, pitch, yaw]
        const double* rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
        
        // Conversion Euler -> Quaternion (simplifiée pour yaw uniquement)
        // Pour un robot 2D, on ne s'intéresse qu'au yaw (rotation autour de Z)
        double yaw = rpy[2]; // Angle autour de Z
        
        // Quaternion pour rotation pure autour de Z :
        // q = [0, 0, sin(yaw/2), cos(yaw/2)]
        quat.i = 0.0f;
        quat.j = 0.0f;
        quat.k = (float)sin(yaw / 2.0);
        quat.real = (float)cos(yaw / 2.0);

        return true;
    }

    // Méthode alternative : retourner directement l'angle yaw
    double getYaw() {
        if (!imu_sensor) return 0.0;
        const double* rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
        return rpy[2]; // Yaw en radians
    }

    // Pour compatibilité avec l'API Adafruit
    bool wasReset() { return false; }
    void hardwareReset() {}
};
#endif