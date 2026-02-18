// Mock_PAA5100.h - Émulation PAA5100 pour Webots UNIQUEMENT
#pragma once
#include <cstdio>
#include <cmath>
#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/inertial_unit.h>

class PAA5100 {
private:
    WbDeviceTag gps_sensor;
    WbDeviceTag imu_sensor;
    double last_x_world = 0.0;
    double last_y_world = 0.0;
    bool first_read = true;
    
    // Accumulateurs pour les fractions de mm (comme le vrai capteur)
    double accumulated_x = 0.0;
    double accumulated_y = 0.0;

public:
    PAA5100() {
        gps_sensor = wb_robot_get_device("GPS");
        imu_sensor = wb_robot_get_device("IMU");
        
        if (gps_sensor) {
            wb_gps_enable(gps_sensor, 1); // 1ms update
            printf("✅ Mock PAA5100: GPS Webots activé (1ms)\n");
        } else {
            printf("❌ Mock PAA5100: GPS introuvable\n");
        }
        
        if (imu_sensor) {
            wb_inertial_unit_enable(imu_sensor, 1);
            printf("✅ Mock PAA5100: IMU pour orientation activé\n");
        } else {
            printf("⚠️  Mock PAA5100: IMU introuvable\n");
        }
    }

    bool begin() {
        return (gps_sensor != 0);
    }

    // ✅ Retourne des deltas en mm dans le repère CAPTEUR
    //    Le capteur est supposé monté à 0° (aligné avec le robot)
    void readMotion(int16_t& out_x, int16_t& out_y) {
        if (!gps_sensor) {
            out_x = 0;
            out_y = 0;
            return;
        }

        // 1. Position GPS actuelle (monde)
        const double* pos = wb_gps_get_values(gps_sensor);
        double current_x_world = pos[0] * 1000.0;  // m → mm
        double current_y_world = pos[1] * 1000.0;

        // 2. Orientation robot
        double robot_yaw = 0.0;
        if (imu_sensor) {
            const double* rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_sensor);
            robot_yaw = rpy[2];
        }

        // 3. Première lecture : initialisation
        if (first_read) {
            last_x_world = current_x_world;
            last_y_world = current_y_world;
            accumulated_x = 0.0;
            accumulated_y = 0.0;
            first_read = false;
            out_x = 0;
            out_y = 0;
            return;
        }

        // 4. Delta dans le monde
        double dx_world = current_x_world - last_x_world;
        double dy_world = current_y_world - last_y_world;
        
        // 5. Transformation Monde → Robot (= repère capteur car monté à 0°)
        double cos_yaw = cos(robot_yaw);
        double sin_yaw = sin(robot_yaw);
        double dx_capteur = cos_yaw * dx_world + sin_yaw * dy_world;
        double dy_capteur = -sin_yaw * dx_world + cos_yaw * dy_world;
        
        // 5.5. FILTRAGE DU BRUIT GPS (zone morte)
        //      Le GPS Webots a un bruit résiduel de ±2mm même au repos (magnitude ~2-3mm)
        //      On filtre les mouvements < 2.0mm/cycle pour éviter l'accumulation de bruit
        //      Les vrais mouvements en déplacement sont >5mm donc non affectés
        const double NOISE_THRESHOLD = 2.0;  // mm - Seuil calibré sur bruit GPS Webots observé
        double magnitude = sqrt(dx_capteur*dx_capteur + dy_capteur*dy_capteur);
        
        if (magnitude < NOISE_THRESHOLD) {
            // Mouvement trop faible = bruit GPS → ignorer
            dx_capteur = 0.0;
            dy_capteur = 0.0;
        }
        
        // 6. ACCUMULATION des fractions (comme le vrai PAA5100)
        //    Le vrai capteur accumule les micro-mouvements et n'incrémente
        //    son compteur que quand il dépasse 1 count (≈0.03mm)
        //    Ici on simule le même principe pour éviter de perdre la précision
        accumulated_x += dx_capteur;
        accumulated_y += dy_capteur;
        
        // 7. Extraction de la partie entière (counts transmis)
        out_x = (int16_t)floor(accumulated_x);
        out_y = (int16_t)floor(accumulated_y);
        
        // 8. Conservation de la fraction pour le prochain cycle
        accumulated_x -= out_x;
        accumulated_y -= out_y;

        // 9. Sauvegarder position pour prochain cycle
        last_x_world = current_x_world;
        last_y_world = current_y_world;
    }

    void reset() {
        first_read = true;
        last_x_world = 0.0;
        last_y_world = 0.0;
        accumulated_x = 0.0;
        accumulated_y = 0.0;
    }
};