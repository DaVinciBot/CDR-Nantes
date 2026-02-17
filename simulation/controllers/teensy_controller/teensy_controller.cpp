#include "ArduinoFake.h"
#include <holonomic_basis.h> 
#include <config.h> 
// Com est déjà inclus via holonomic_basis.h

// Sensors mockés pour l'odométrie
#include "Mock_PAA5100.h"  // GPS
#include "Mock_BNO085.h"   // IMU

// Supervisor pour lire la vraie position du robot
#include <webots/supervisor.h>
// LIDAR Webots
#include <webots/lidar.h>

// PID et Base (Mêmes valeurs que votre vrai robot ou ajustées pour la simu)
PID x_pid(KP_X, KI_X, KD_X, -MAX_SPEED, MAX_SPEED, 5.0);
PID y_pid(KP_Y, KI_Y, KD_Y, -MAX_SPEED, MAX_SPEED, 5.0);
PID theta_pid(KP_THETA, KI_THETA, KD_THETA, -MAX_SPEED, MAX_SPEED, 0.005);

Holonomic_Basis* holonomic_basis_ptr = nullptr;
Com* com = nullptr;

// Sensors pour odométrie (noms compatibles avec les vraies librairies)
PAA5100* paa5100 = nullptr;           // Capteur optique
Adafruit_BNO085* bno085 = nullptr;    // IMU

// LIDAR Webots
WbDeviceTag lidar = 0;

// --- MODIFICATION ICI : Cible définie à 100, 0, 0 pour le test ---
// Au lieu de START_X, START_Y, START_THETA
Point target_position(START_X,START_Y, START_THETA);


// Callback simple pour recevoir la cible depuis Python
void set_target_position(byte* msg, byte size) {
    msg_set_target_position* t = (msg_set_target_position*)msg;
    // Python envoie déjà en mm, pas besoin de conversion
    target_position.x = t->target_position_x;
    target_position.y = t->target_position_y;
    target_position.theta = t->target_position_theta;
    
    printf("Webots: Nouvelle cible reçue -> X:%.2fmm Y:%.2fmm θ:%.2frad\n", 
           target_position.x, target_position.y, target_position.theta);
}

void (*callback_functions[256])(byte* msg, byte size);

// Fonction pour traiter et envoyer les données LIDAR par secteurs
void process_lidar_scan(WbDeviceTag lidar, Com* com) {
    if (lidar == 0) return;
    
    const float* range_image = wb_lidar_get_range_image(lidar);
    if (range_image == NULL) return;
    
    int num_points = wb_lidar_get_horizontal_resolution(lidar);
    float fov = wb_lidar_get_fov(lidar);  // Champ de vision en radians
    
    // Diviser en 8 secteurs de 45° (π/4 radians)
    const int NUM_SECTORS = 8;
    const float SECTOR_ANGLE = 2.0f * M_PI / NUM_SECTORS;  // 45° en radians
    
    msg_lidar_scan lidar_data;
    lidar_data.timestamp = (uint32_t)wb_robot_get_time() * 1000;  // Conversion en ms
    
    // Initialisation des secteurs
    for (int i = 0; i < NUM_SECTORS; i++) {
        lidar_data.sector_min[i] = 65535;  // Max uint16 (infini)
        lidar_data.sector_max[i] = 0;
    }
    
    // Traitement de chaque point LIDAR
    float angle_step = fov / (num_points - 1);
    float start_angle = -fov / 2.0f;  // Angle de départ (généralement -π)
    
    for (int i = 0; i < num_points; i++) {
        float distance = range_image[i];
        
        // Ignorer les points à l'infini ou invalides
        if (isinf(distance) || isnan(distance) || distance < 0.01f) {
            continue;
        }
        
        // Calculer l'angle du point (normalisé entre 0 et 2π)
        float point_angle = start_angle + (i * angle_step);
        while (point_angle < 0) point_angle += 2.0f * M_PI;
        while (point_angle >= 2.0f * M_PI) point_angle -= 2.0f * M_PI;
        
        // Déterminer le secteur (0-7)
        int sector = (int)(point_angle / SECTOR_ANGLE);
        if (sector >= NUM_SECTORS) sector = NUM_SECTORS - 1;
        
        // Conversion mètres → millimètres
        uint16_t distance_mm = (uint16_t)(distance * 1000.0f);
        if (distance_mm > 10000) distance_mm = 10000;  // Limiter à 10m
        
        // Mise à jour min/max du secteur
        if (distance_mm < lidar_data.sector_min[sector]) {
            lidar_data.sector_min[sector] = distance_mm;
        }
        if (distance_mm > lidar_data.sector_max[sector]) {
            lidar_data.sector_max[sector] = distance_mm;
        }
    }
    
    // Remplacer les secteurs vides (infini) par 10000mm (10m = hors portée)
    for (int i = 0; i < NUM_SECTORS; i++) {
        if (lidar_data.sector_min[i] == 65535) {
            lidar_data.sector_min[i] = 10000;
            lidar_data.sector_max[i] = 10000;
        }
    }
    
    // Envoi des données au Raspberry Pi
    com->send_msg((byte*)&lidar_data, sizeof(lidar_data));
}

int main(int argc, char **argv) {
    wb_robot_init(); // 1. Initialisation Webots obligatoire
    
    // Récupération du nœud robot pour lire sa vraie position (Ground Truth)
    WbNodeRef robot_node = wb_supervisor_node_get_self();
    WbFieldRef translation_field = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");
    
    // Serial.begin() sera appelé par le constructeur Com
    com = new Com(&Serial, 115200);  // 2. Création de Com (ouvre COM2 automatiquement)

    // 4. Création de votre base
    holonomic_basis_ptr = new Holonomic_Basis(
        ROBOT_RADIUS, WHEEL_DIAMETER, MAX_SPEED, MAX_ACCELERATION,
        STEPS_PER_REVOLUTION, MICROSTEPS, x_pid, y_pid, theta_pid
    );
    // 5. Définition des roues (IMPORTANT)
    // On passe 1, 2, 3 car fake_stepper.cpp va chercher "motor1", "motor2", "motor3"
    holonomic_basis_ptr->define_wheel1(1, 0, 0); 
    holonomic_basis_ptr->define_wheel2(2, 0, 0);
    holonomic_basis_ptr->define_wheel3(3, 0, 0);
    
    holonomic_basis_ptr->init_motors();
    // Le robot pense toujours qu'il est à START_X (ex: 0), mais la target est à 100
    holonomic_basis_ptr->init_holonomic_basis(START_X, START_Y, START_THETA);
    
    // Initialisation des sensors pour odométrie
    //paa5100 = new PAA5100();

    // Initialisation du LIDAR
    lidar = wb_robot_get_device("lidar");
    if (lidar != 0) {
        wb_lidar_enable(lidar, time_step);
        wb_lidar_enable_point_cloud(lidar);
        int h_res = wb_lidar_get_horizontal_resolution(lidar);
        printf("✅ LIDAR activé : %d points de résolution\n", h_res);
    } else {
        printf("⚠️  LIDAR non trouvé (device 'lidar' absent du .wbt)\n");
    }
    //bno085 = new Adafruit_BNO085();
    holonomic_basis_ptr->init_sensors();
    printf("✅ Sensors d'odométrie initialisés (GPS + IMU)\n");

    // 6. Enregistrement des callbacks
    for (int i = 0; i < 256; i++) callback_functions[i] = nullptr;
    callback_functions[SET_TARGET_POSITION] = &set_target_position;
    printf("✅ Webots: Contrôleur Teensy démarré (PID: %s)\n", 
           holonomic_basis_ptr->use_pid_control ? "ON" : "OFF");
    printf("📍 Position initiale: X=%.1f Y=%.1f θ=%.2f\n", 
           START_X, START_Y, START_THETA);
    printf("🎯 Cible initiale de test: X=%.1f Y=%.1f θ=%.2f\n", 
           target_position.x, target_position.y, target_position.theta);

    // BOUCLE PRINCIPALE (remplace loop())
    int time_step = (int)wb_robot_get_basic_time_step();
    uint32_t loop_counter = 0;

    while (wb_robot_step(time_step) != -1) {
        loop_counter++;
        
        // A. Communication (Réception des ordres Python)
        com->handle_callback(callback_functions);
        
        // B. Mise à jour odométrie avec fusion sensors (encodeurs + GPS + IMU)
        holonomic_basis_ptr->update_odometry();
        
        // C. Calcul Asservissement
        // (Note: handle() appelle getPosition() qui lit les encodeurs Webots)
        holonomic_basis_ptr->handle(target_position, com);
        
        // D. Action (Envoi des vitesses aux moteurs Webots)
        holonomic_basis_ptr->execute_movement();

        // E. Acquisition et envoi des données LIDAR (toutes les 100ms = 10Hz)
        if (lidar != 0 && loop_counter % 10 == 0) {
            process_lidar_scan(lidar, com);
        }

        if (loop_counter % (1000 / time_step) == 0) { // Log périodique toutes les 1 sec
            Point current_pos = holonomic_basis_ptr->get_current_position();
            
            // Lecture position RÉELLE du robot (Ground Truth)
            const double* real_pos = wb_supervisor_field_get_sf_vec3f(translation_field);
            const double* real_rot = wb_supervisor_field_get_sf_rotation(rotation_field);
            
            // Conversion en mm et angle
            double real_x_mm = real_pos[0] * 1000.0;
            double real_y_mm = real_pos[1] * 1000.0;
            double real_theta = real_rot[3];  // Angle de rotation autour de l'axe Z
            
            // Calcul des erreurs d'odométrie
            double error_x = current_pos.x - real_x_mm;
            double error_y = current_pos.y - real_y_mm;
            double error_theta = current_pos.theta - real_theta;
            double error_distance = sqrt(error_x*error_x + error_y*error_y);
            
            // Calcul distance à la cible
            double dist_to_target = sqrt(pow(target_position.x - real_x_mm, 2) + 
                                        pow(target_position.y - real_y_mm, 2));
            
            printf("\n═══════════════════════════════════════════════════════════════\n");
            printf("🎯 TARGET     : X=%6.1f Y=%6.1f θ=%+6.3f (dist=%.1fmm)\n",
                   target_position.x, target_position.y, target_position.theta, dist_to_target);
            printf("✅ RÉEL       : X=%6.1f Y=%6.1f θ=%+6.3f\n",
                   real_x_mm, real_y_mm, real_theta);
            printf("📊 CALCULÉ    : X=%6.1f Y=%6.1f θ=%+6.3f\n",
                   current_pos.x, current_pos.y, current_pos.theta);
            printf("❌ ERREUR ODO : ΔX=%+5.1f ΔY=%+5.1f Δθ=%+.3f (%.1fmm)\n",
                   error_x, error_y, error_theta, error_distance);
            printf("═══════════════════════════════════════════════════════════════\n");
        }
         if (loop_counter % 10 == 0) {  // Envoi télémétrie toutes les ~X ms
            Point current_pos = holonomic_basis_ptr->get_current_position();
            msg_update_rolling_basis msg_pos;
            msg_pos.x = current_pos.x;
            msg_pos.y = current_pos.y;
            msg_pos.theta = current_pos.theta;
            com->send_msg((byte*)&msg_pos, sizeof(msg_pos));
        }
    }
    
    wb_robot_cleanup();
    return 0;
}