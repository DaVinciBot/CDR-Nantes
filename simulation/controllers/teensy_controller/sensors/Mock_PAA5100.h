// Mock_PAA5100.h - Emulation RÉALISTE du capteur optique PAA5100 via GPS Webots
// Le vrai PAA5100 donne des déplacements relatifs dans le REPÈRE DU ROBOT, pas du monde
#pragma once
#include <cstdio>
#include <cmath>
#ifdef WEBOTS_SIMULATION
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#else
#include <PAA5100.h>
#endif

class PAA5100 {
private:
    WbDeviceTag gps_sensor;
    WbDeviceTag imu_sensor;  // Besoin de l'IMU pour connaître l'orientation
    double last_x_world = 0.0;
    double last_y_world = 0.0;
    double last_timestamp = -1.0;  // Pour détecter nouvelles données GPS
    bool first_read = true; 
    
    // Accumulateurs pour éviter de perdre des deltas entre lectures
    double accumulated_dx_robot = 0.0;
    double accumulated_dy_robot = 0.0;

public:
    int16_t delta_x_internal = 0;
    int16_t delta_y_internal = 0;

    PAA5100() {
        gps_sensor = wb_robot_get_device("GPS");
        imu_sensor = wb_robot_get_device("IMU");
        
        if (gps_sensor) {
            // Réduction à 1ms pour avoir des updates fréquents
            wb_gps_enable(gps_sensor, 1); 
            printf("✅ Mock PAA5100: GPS Webots activé (1ms)\n");
        } else {
            printf("❌ Mock PAA5100: GPS introuvable\n");
        }
        
        if (imu_sensor) {
            wb_inertial_unit_enable(imu_sensor, 1);  // 1ms aussi
            printf("✅ Mock PAA5100: IMU pour orientation activé\n");
        } else {
            printf("⚠️ Mock PAA5100: IMU introuvable (orientation sera 0)\n");
        }
    }

    bool begin() {
        return (gps_sensor != 0);
    }

    // Le vrai PAA5100 retourne des déplacements dans le repère du CAPTEUR (donc du robot)
    void readMotion(int16_t& out_x, int16_t& out_y) {
        if (!gps_sensor) {
            out_x = 0;
            out_y = 0;
            return;
        }

        // 1. Vérifier si le GPS a de nouvelles données
        //double current_timestamp = wb_gps_get_speed_vector(gps_sensor)[0];
        const double* pos = wb_gps_get_values(gps_sensor);
        double current_x_world = pos[0] * 1000.0;  // m -> mm
        double current_y_world = pos[1] * 1000.0;
        
        // 2. Lire orientation du robot (yaw)
        double robot_yaw = 0.0;
        if (imu_sensor) {
            const double* rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
            robot_yaw = rpy[2];  // yaw (rotation autour de Z)
        }

        if (first_read) {
            last_x_world = current_x_world;
            last_y_world = current_y_world;
            last_timestamp = wb_robot_get_time();
            accumulated_dx_robot = 0.0;
            accumulated_dy_robot = 0.0;
            delta_x_internal = 0;
            delta_y_internal = 0;
            first_read = false;
            out_x = 0;
            out_y = 0;
            return;
        }

        // 3. Calculer déplacement dans le repère MONDE
        double dx_world = current_x_world - last_x_world;
        double dy_world = current_y_world - last_y_world;
        
        // Seulement si le robot a vraiment bougé (éviter les deltas nuls répétés)
        if (fabs(dx_world) > 0.0001 || fabs(dy_world) > 0.0001) {
            // 4. Transformer en repère ROBOT (rotation inverse)
            double cos_yaw = cos(robot_yaw);
            double sin_yaw = sin(robot_yaw);
            
            // Rotation inverse : Monde -> Robot
            double dx_robot = cos_yaw * dx_world + sin_yaw * dy_world;
            double dy_robot = -sin_yaw * dx_world + cos_yaw * dy_world;
            
            // Accumuler pour ne pas perdre de précision
            accumulated_dx_robot += dx_robot;
            accumulated_dy_robot += dy_robot;
            
            // Mise à jour pour prochain cycle
            last_x_world = current_x_world;
            last_y_world = current_y_world;
        }
        
        // 5. Retourner les deltas accumulés et réinitialiser
        delta_x_internal = (int16_t)round(accumulated_dx_robot);
        delta_y_internal = (int16_t)round(accumulated_dy_robot);
        
        out_x = delta_x_internal;
        out_y = delta_y_internal;
        
        // Réinitialiser l'accumulateur après lecture
        accumulated_dx_robot = 0.0;
        accumulated_dy_robot = 0.0;
    }

    void reset() {
        first_read = true;
        delta_x_internal = 0;
        delta_y_internal = 0;
        last_x_world = 0.0;
        last_y_world = 0.0;
        last_timestamp = -1.0;
        accumulated_dx_robot = 0.0;
        accumulated_dy_robot = 0.0;
    }
};