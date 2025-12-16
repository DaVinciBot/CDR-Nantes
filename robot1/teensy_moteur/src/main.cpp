// External libraries used: Arduino
#include <Arduino.h>      // Arduino framework

#include <com.h>              // Communication class (includes messages.h)
#include <holonomic_basis.h>  // Holonomic Basis with KaribouMotion
#include <config.h>           // Configuration file


PID x_pid(KP_X, KI_X, KD_X, -MAX_SPEED, MAX_SPEED, 5.0);
PID y_pid(KP_Y, KI_Y, KD_Y, -MAX_SPEED, MAX_SPEED, 5.0);
PID theta_pid(KP_THETA, KI_THETA, KD_THETA, -MAX_SPEED, MAX_SPEED, 2.0);

Holonomic_Basis* holonomic_basis_ptr = new Holonomic_Basis(
    ROBOT_RADIUS,
    WHEEL_DIAMETER,
    MAX_SPEED,
    MAX_ACCELERATION,
    STEPS_PER_REVOLUTION,
    MICROSTEPS,
    x_pid,
    y_pid,
    theta_pid
);

Com* com;
Point target_position(START_X, START_Y, START_THETA);

// Timers Hardware (Teensy 4.1 possède 4 IntervalTimers)
IntervalTimer timer_compute; // Pour l'asservissement (Lent)
IntervalTimer timer_step;    // Pour les moteurs (Rapide)

void set_target_position(byte* msg, byte size) {
    msg_set_target_position* target_msg = (msg_set_target_position*)msg;
    target_position.x = target_msg->target_position_x;
    target_position.y = target_msg->target_position_y;
    target_position.theta = target_msg->target_position_theta;
}

void set_pid(byte* msg, byte size) {
    msg_set_pid* pid_msg = (msg_set_pid*)msg;
    PID* pid = nullptr;
    
    switch (pid_msg->pid_type) {
        case X_PID_ID: pid = &holonomic_basis_ptr->x_pid; break;
        case Y_PID_ID: pid = &holonomic_basis_ptr->y_pid; break;
        case THETA_PID_ID: pid = &holonomic_basis_ptr->theta_pid; break;
    }
    
    if (pid) {
        pid->updateParameters(pid_msg->kp, pid_msg->ki, pid_msg->kd);
    }
}

void set_odometrie(byte* msg, byte size) {
    msg_set_odometrie* odo = (msg_set_odometrie*)msg;
    
    holonomic_basis_ptr->init_holonomic_basis(odo->x, odo->y, odo->theta);

    target_position.x = odo->x;
    target_position.y = odo->y;
    target_position.theta = odo->theta;
}

void reset_teensy(byte* msg, byte size) {
    SCB_AIRCR = 0x05FA0004; // Reset Hardware ARM
}

void (*callback_functions[256])(byte* msg, byte size);

void initialize_callback_functions() {
    callback_functions[SET_TARGET_POSITION] = &set_target_position;
    callback_functions[SET_PID] = &set_pid;
    callback_functions[SET_ODOMETRIE] = &set_odometrie;
    callback_functions[RESET_TEENSY] = &reset_teensy;
}


// [TIMER LENT] - 100 Hz (10ms)
// Gère l'intelligence : PID, Cinématique, Planification de trajectoire
void interruption_compute() {
    // 1. Calcul du PID et mise à jour de l'odométrie (Dead Reckoning)
    holonomic_basis_ptr->handle(target_position, com);
    
    // 2. Conversion des vitesses PID en commandes de pas (Relatif)
    holonomic_basis_ptr->execute_movement();
    
    // 3. Mise à jour du profil de vitesse trapézoïdal des steppers
    holonomic_basis_ptr->compute_steppers();
}

// [TIMER RAPIDE] - 25 kHz (40µs)
// Gère la physique : Génération des signaux STEP pour les drivers
// Plus ce timer est rapide, plus la vitesse max est élevée et le mouvement fluide.
// Teensy 4.1 peut encaisser 50kHz ou 100kHz sans problème si le code est optimisé.
void interruption_step() {
    holonomic_basis_ptr->step_steppers();
}

void setup() {
    
    com = new Com(&Serial, BAUDRATE);

    // Configuration des Pins Moteurs
    holonomic_basis_ptr->define_wheel1(W1_STEP_PIN, W1_DIR_PIN, W1_ENABLE_PIN);
    holonomic_basis_ptr->define_wheel2(W2_STEP_PIN, W2_DIR_PIN, W2_ENABLE_PIN);
    holonomic_basis_ptr->define_wheel3(W3_STEP_PIN, W3_DIR_PIN, W3_ENABLE_PIN);
    
    // Initialisation (Création du StepperGroup KaribouMotion)
    holonomic_basis_ptr->init_motors();
    holonomic_basis_ptr->init_holonomic_basis(START_X, START_Y, START_THETA);
    holonomic_basis_ptr->enable_motors();

    // Init Callbacks
    initialize_callback_functions();

    // Démarrage des Timers
    // timer_compute : 100Hz = 10000 µs
    timer_compute.begin(interruption_compute, ASSERVISSEMENT_FREQUENCY); 
    
    // timer_step : 25kHz = 40 µs
    // C'est ici qu'on remplace la magie de TeensyStep4 par notre contrôle direct
    timer_step.begin(interruption_step, 40);

    delay(100);
}


uint_fast32_t counter = 0;

void loop() {
    // Gestion des messages entrants (USB)
    com->handle_callback(callback_functions);

    // Envoi périodique de la télémétrie vers la Raspberry Pi
    // On utilise un compteur simple pour ne pas saturer le port série
    if (counter++ > 50000) { 
        msg_update_rolling_basis odo_msg;
        Point current = holonomic_basis_ptr->get_current_position();
        
        odo_msg.x = current.x;
        odo_msg.y = current.y;
        odo_msg.theta = current.theta;

        com->send_msg((byte*)&odo_msg, sizeof(msg_update_rolling_basis));
        counter = 0;
    }
}