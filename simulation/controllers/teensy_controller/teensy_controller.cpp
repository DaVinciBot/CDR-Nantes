#include "ArduinoFake.h"
#include <holonomic_basis.h> 
#include <config.h> 
// Com est déjà inclus via holonomic_basis.h

// Sensors mockés pour l'odométrie
#include "Mock_PAA5100.h"  // GPS
#include "Mock_BNO085.h"   // IMU

// PID et Base (Mêmes valeurs que votre vrai robot ou ajustées pour la simu)
PID x_pid(KP_X, KI_X, KD_X, -MAX_SPEED, MAX_SPEED, 5.0);
PID y_pid(KP_Y, KI_Y, KD_Y, -MAX_SPEED, MAX_SPEED, 5.0);
PID theta_pid(KP_THETA, KI_THETA, KD_THETA, -MAX_SPEED, MAX_SPEED, 0.005);

Holonomic_Basis* holonomic_basis_ptr = nullptr;
Com* com = nullptr;

// Sensors pour odométrie (noms compatibles avec les vraies librairies)
PAA5100* paa5100 = nullptr;           // Capteur optique
Adafruit_BNO085* bno085 = nullptr;    // IMU

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

int main(int argc, char **argv) {
    wb_robot_init(); // 1. Initialisation Webots obligatoire
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

        if (loop_counter % (1000 / time_step) == 0) { // Log périodique
            Point current_pos = holonomic_basis_ptr->get_current_position();
            printf("📊 [%d] Pos: X=%.1f Y=%.1f θ=%.2f | Target: X=%.1f Y=%.1f θ=%.2f\n",
                   loop_counter, 
                   current_pos.x, current_pos.y, current_pos.theta,
                   target_position.x, target_position.y, target_position.theta);
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