// External libraries used: Arduino
#include <Arduino.h>      // Arduino framework
#include <holonomic_basis.h>  // Holonomic Basis with MKS RS485
#include <config.h>           // Configuration file

// PID Controllers
PID x_pid(KP_X, KI_X, KD_X, -MAX_SPEED_RPM, MAX_SPEED_RPM, 5.0);
PID y_pid(KP_Y, KI_Y, KD_Y, -MAX_SPEED_RPM, MAX_SPEED_RPM, 5.0);
PID theta_pid(KP_THETA, KI_THETA, KD_THETA, -MAX_SPEED_RPM, MAX_SPEED_RPM, 2.0);

Holonomic_Basis* holonomic_basis_ptr = new Holonomic_Basis(
    ROBOT_RADIUS,
    WHEEL_DIAMETER,
    MAX_SPEED_RPM,
    x_pid,
    y_pid,
    theta_pid
);

Com* com;
Point target_position(START_X, START_Y, START_THETA);

// Timers Hardware (Teensy 4.1 possède 4 IntervalTimers)
IntervalTimer timer_compute; // Pour l'asservissement (Lent - 100Hz)

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
    void(*reboot)(void)=0;
    reboot(); // Reset Hardware ARM
}

void (*callback_functions[256])(byte* msg, byte size);

void initialize_callback_functions() {
    callback_functions[SET_TARGET_POSITION] = &set_target_position;
    callback_functions[SET_PID] = &set_pid;
    callback_functions[SET_ODOMETRIE] = &set_odometrie;
    callback_functions[RESET_TEENSY] = &reset_teensy;
}


// [TIMER LENT] - 100 Hz (10ms)
// Gère l'intelligence : PID, Cinématique
void interruption_compute() {
    // 1. Fusion de capteurs (GPS/IMU + Dead Reckoning)
    holonomic_basis_ptr->update_odometry();
    
    // 2. Calcul du PID et mise à jour de l'odométrie
    holonomic_basis_ptr->handle(target_position, com);
    
    // 3. Signaler que loop() doit envoyer les commandes velocité
    holonomic_basis_ptr->need_send_movement = true;
    
    // 4. Signaler que loop() doit lire les encodeurs (rafale synchronisée)
    holonomic_basis_ptr->need_read_encoders = true;
    
    // 5. Signaler que loop() doit lire l'IMU
    holonomic_basis_ptr->need_read_imu = true;
}

void setup() {
    // Initialisation communication
    com = new Com(&Serial, BAUDRATE);

    // Configuration des moteurs RS485
    holonomic_basis_ptr->define_wheel1(W1_SERIAL, W1_ADDR);
    holonomic_basis_ptr->define_wheel2(W2_SERIAL, W2_ADDR);
    holonomic_basis_ptr->define_wheel3(W3_SERIAL, W3_ADDR);
    
    // Initialisation moteurs et base holonome
    holonomic_basis_ptr->init_motors();
    holonomic_basis_ptr->init_holonomic_basis(START_X, START_Y, START_THETA);
    
    // Initialisation des capteurs (PMW3901, BNO085)
    //holonomic_basis_ptr->init_sensors();

    // Initialisation des callbacks BEFORE starting timer
    initialize_callback_functions();

    // Démarrage des Timers (AFTER callbacks are set)
    timer_compute.begin(interruption_compute, ASSERVISSEMENT_FREQUENCY); 
    
    delay(100);
}


uint_fast32_t counter = 0;
static bool motors_enabled = false;

void loop() {
    // ===== ÉTAPE 1: GESTION DES MESSAGES USB (prioritaire) =====
    com->handle_callback(callback_functions);
    
    // ===== ÉTAPE 2: OPÉRATIONS RS485 NON-BLOQUANTES =====
    // Ces appels peuvent bloquer ~2ms (rafale sync), mais c'est acceptable après les callbacks USB
    
    // Lecture encodeurs (~2ms via readAllEncodersSynced rafale synchronisée)
    if (holonomic_basis_ptr->need_read_encoders) {
        if (holonomic_basis_ptr->read_encoders_nonblocking()) {
            holonomic_basis_ptr->need_read_encoders = false;
        }
        // Si timeout : retry au prochain loop()
    }

    // Lecture IMU (~0.5ms I2C non-bloquant)
    if (holonomic_basis_ptr->need_read_imu) {
        holonomic_basis_ptr->read_imu_nonblocking();
        holonomic_basis_ptr->need_read_imu = false;
    }
    
    // Envoi commandes vitesse aux MKS (~0.3ms fire & forget avec setSpeedsSynced)
    if (holonomic_basis_ptr->need_send_movement) {
        if (holonomic_basis_ptr->send_movement_commands_nonblocking()) {
            holonomic_basis_ptr->need_send_movement = false;
        }
        // Si timeout : retry au prochain loop()
    }
    
    // ===== ÉTAPE 3: MOTOR ENABLE (une seule fois) =====
    // Non-blocking motor enable on first pass (deferred from setup to avoid RS485 timeout freeze)
    if (!motors_enabled) {
        holonomic_basis_ptr->enable_motors();
        motors_enabled = true;
    }

    // ===== ÉTAPE 4: TÉLÉMÉTRIE VERS PI (lente) =====
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