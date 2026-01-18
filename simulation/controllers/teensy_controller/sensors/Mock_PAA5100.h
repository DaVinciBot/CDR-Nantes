// Mock_PAA5100.h - Emulation du capteur optique PAA5100 via GPS Webots
#pragma once
#include <cstdio>
#ifdef WEBOTS_SIMULATION
#include <webots/robot.h>
#include <webots/gps.h>
#else
#include <PAA5100.h>
#endif

class PAA5100 {
private:
    WbDeviceTag gps_sensor;
    double last_x = 0.0;
    double last_y = 0.0;
    bool first_read = true; 

public:
    int16_t delta_x_internal = 0;
    int16_t delta_y_internal = 0;

    PAA5100() {
        gps_sensor = wb_robot_get_device("GPS");
        if (gps_sensor) {
            wb_gps_enable(gps_sensor, 16); // 16ms
            printf("✅ Mock PAA5100: GPS Webots activé\n");
        } else {
            printf("❌ Mock PAA5100: GPS introuvable\n");
        }
    }

    // CORRECTION 1 : Renommer init() en begin() pour matcher holonomic_basis.cpp
    bool begin() {
        return (gps_sensor != 0);
    }

    // CORRECTION 2 : Modifier la signature pour accepter les références
    void readMotion(int16_t& out_x, int16_t& out_y) {
        if (!gps_sensor) {
            out_x = 0;
            out_y = 0;
            return;
        }

        const double* pos = wb_gps_get_values(gps_sensor);
        double current_x_mm = pos[0] * 1000.0; 
        double current_y_mm = pos[1] * 1000.0;

        if (first_read) {
            last_x = current_x_mm;
            last_y = current_y_mm;
            delta_x_internal = 0;
            delta_y_internal = 0;
            first_read = false;
        } else {
            delta_x_internal = (int16_t)(current_x_mm - last_x);
            delta_y_internal = (int16_t)(current_y_mm - last_y);
            
            last_x = current_x_mm;
            last_y = current_y_mm;
        }
        
        // Remplir les variables passées en argument
        out_x = delta_x_internal;
        out_y = delta_y_internal;
    }
};