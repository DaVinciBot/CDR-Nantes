#include "ArduinoFake.h"
// Assurez-vous que ces chemins d'include sont corrects dans le Makefile
#include <config.h> 
#include <holonomic_basis.h> 
#include <com.h>

SerialMock Serial;
Com* com;

// PID et Base (Mêmes valeurs que votre vrai robot ou ajustées pour la simu)
// Astuce: En simu, on peut être plus agressif sur les PID
PID x_pid(KP_X, KI_X, KD_X, -MAX_SPEED, MAX_SPEED, 10.0);
PID y_pid(KP_Y, KI_Y, KD_Y, -MAX_SPEED, MAX_SPEED, 10.0);
PID theta_pid(KP_THETA, KI_THETA, KD_THETA, -MAX_SPEED, MAX_SPEED, 5.0);

Holonomic_Basis* holonomic_basis_ptr;
Point target_position(START_X, START_Y, START_THETA);

// Callback simple pour recevoir la cible depuis Python
void set_target_position(byte* msg, byte size) {
    msg_set_target_position* t = (msg_set_target_position*)msg;
    target_position.x = t->target_position_x;
    target_position.y = t->target_position_y;
    target_position.theta = t->target_position_theta;
    printf("Webots: Nouvelle cible reçue -> X:%.2f Y:%.2f T:%.2f\n", target_position.x, target_position.y, target_position.theta);
}

void (*callback_functions[256])(byte* msg, byte size);

int main(int argc, char **argv) {
    wb_robot_init(); // 1. Initialisation Webots obligatire
    
    Serial.begin(115200); // 2. Connexion au port COM virtuel
    com = new Com(&Serial, 115200);

    // 3. Création de votre base
    holonomic_basis_ptr = new Holonomic_Basis(
        ROBOT_RADIUS, WHEEL_DIAMETER, MAX_SPEED, MAX_ACCELERATION,
        STEPS_PER_REVOLUTION, MICROSTEPS, x_pid, y_pid, theta_pid
    );

    // 4. Définition des roues (IMPORTANT)
    // On passe 1, 2, 3 car fake_stepper.cpp va chercher "motor1", "motor2", "motor3"
    holonomic_basis_ptr->define_wheel1(1, 0, 0); 
    holonomic_basis_ptr->define_wheel2(2, 0, 0);
    holonomic_basis_ptr->define_wheel3(3, 0, 0);
    
    holonomic_basis_ptr->init_motors();
    holonomic_basis_ptr->init_holonomic_basis(START_X, START_Y, START_THETA);

    // Enregistrement des callbacks
    callback_functions[SET_TARGET_POSITION] = &set_target_position;
    
    printf("Webots: Contrôleur Teensy Démarré !\n");

    // BOUCLE PRINCIPALE (remplace loop())
    int time_step = (int)wb_robot_get_basic_time_step();
    while (wb_robot_step(time_step) != -1) {
        
        // A. Communication (Réception des ordres Python)
        com->handle_callback(callback_functions);
        
        // B. Calcul Asservissement
        // (Note: handle() appelle getPosition() qui lit les encodeurs Webots)
        holonomic_basis_ptr->handle(target_position, com);
        
        // C. Action (Envoi des vitesses aux moteurs Webots)
        holonomic_basis_ptr->execute_movement();
    }
    
    wb_robot_cleanup();
    return 0;
}