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

IntervalTimer timer_compute;

// ===== ÉTAT MACHINE =====
enum RobotState { IDLE, MOVING };
RobotState robot_state = IDLE;

// ===== WATCHDOG & TIMEOUT =====
uint32_t last_message_timestamp = 0;
uint32_t movement_start_ms      = 0;
uint32_t movement_timeout_ms    = 0;

// ===== UTILITAIRE : angle normalisé =====
// (déjà défini dans holonomic_basis.cpp mais on en a besoin ici aussi)
double normalizeAngle_main(double theta) {
    theta = fmod(theta + M_PI, 2.0 * M_PI);
    if (theta < 0.0) theta += 2.0 * M_PI;
    return theta - M_PI;
}

// ===== CALCUL TIMEOUT DYNAMIQUE =====
uint32_t compute_movement_timeout(double from_x, double from_y, double from_theta,
                                  double to_x,   double to_y,   double to_theta) {
    double dist_mm   = sqrt(pow(to_x - from_x, 2) + pow(to_y - from_y, 2));
    double angle_rad = fabs(normalizeAngle_main(to_theta - from_theta));

    double t_trans_s = dist_mm   / ROBOT_MAX_SPEED_MM_S;
    double t_rot_s   = angle_rad / ROBOT_MAX_RAD_S;
    double t_total_s = (t_trans_s + t_rot_s) * MOVEMENT_TIMEOUT_MARGIN;

    uint32_t timeout_ms = (uint32_t)(t_total_s * 1000.0);
    if (timeout_ms < MOVEMENT_TIMEOUT_MIN_MS) timeout_ms = MOVEMENT_TIMEOUT_MIN_MS;

    return timeout_ms;
}

// ===== CALLBACKS =====

void set_target_position(byte* msg, byte size) {
    last_message_timestamp = millis();

    msg_set_target_position* target_msg = (msg_set_target_position*)msg;

    // Calculer timeout AVANT d'écraser target_position
    Point current = holonomic_basis_ptr->get_current_position();
    movement_timeout_ms = compute_movement_timeout(
        current.x, current.y, current.theta,
        target_msg->target_position_x,
        target_msg->target_position_y,
        target_msg->target_position_theta
    );

    target_position.x     = target_msg->target_position_x;
    target_position.y     = target_msg->target_position_y;
    target_position.theta = target_msg->target_position_theta;

    movement_start_ms = millis();
    robot_state       = MOVING;
}

void set_pid(byte* msg, byte size) {
    last_message_timestamp = millis();

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
    last_message_timestamp = millis();

    msg_set_odometrie* odo = (msg_set_odometrie*)msg;
    
    holonomic_basis_ptr->init_holonomic_basis(odo->x, odo->y, odo->theta);

    target_position.x = odo->x;
    target_position.y = odo->y;
    target_position.theta = odo->theta;

    // Arrêt propre + reset état machine
    robot_state = IDLE;
    holonomic_basis_ptr->emergency_stop();
}

void reset_teensy(byte* msg, byte size) {
    void(*reboot)(void) = 0;
    reboot();
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

    // 2. Calcul PID uniquement si on est en mouvement
    if (robot_state == MOVING) {
        holonomic_basis_ptr->handle(target_position, com);

        // 3. Vérifier si cible atteinte (zone morte)
        Point current = holonomic_basis_ptr->get_current_position();
        double xerr = target_position.x - current.x;
        double yerr = target_position.y - current.y;
        double terr = fabs(normalizeAngle_main(target_position.theta - current.theta));

        if (sqrt(xerr*xerr + yerr*yerr) < 1.0 && terr < 0.02) {
            robot_state = IDLE;
            holonomic_basis_ptr->emergency_stop();
            Serial.println("[DONE] Cible atteinte → IDLE");
        }
    }

    // 4. Signaux pour loop() — RS485 uniquement si nécessaire
    holonomic_basis_ptr->need_send_movement = (robot_state == MOVING);
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
    uint32_t now = millis();

    // ===== ÉTAPE 0: WATCHDOG & TIMEOUT =====
    if (robot_state == MOVING) {
        // Timeout dynamique : mouvement trop long (odo fausse, robot bloqué)
        if ((now - movement_start_ms) > movement_timeout_ms) {
            robot_state = IDLE;
            holonomic_basis_ptr->emergency_stop();
            Serial.printf("[TIMEOUT] Mouvement > %ums → IDLE\n", movement_timeout_ms);
        }
        // Watchdog : Python a crashé en plein mouvement
        else if ((now - last_message_timestamp) > WATCHDOG_TIMEOUT_MS) {
            robot_state = IDLE;
            holonomic_basis_ptr->emergency_stop();
            Serial.println("[WATCHDOG] Python injoignable → IDLE");
        }
    }

    // ===== ÉTAPE 1: MESSAGES USB =====
    com->handle_callback(callback_functions);

    // ===== ÉTAPE 2: RS485 NON-BLOQUANT =====
    if (holonomic_basis_ptr->need_read_encoders) {
        if (holonomic_basis_ptr->read_encoders_nonblocking())
            holonomic_basis_ptr->need_read_encoders = false;
    }

    if (holonomic_basis_ptr->need_read_imu) {
        holonomic_basis_ptr->read_imu_nonblocking();
        holonomic_basis_ptr->need_read_imu = false;
    }

    if (holonomic_basis_ptr->need_send_movement) {
        if (holonomic_basis_ptr->send_movement_commands_nonblocking())
            holonomic_basis_ptr->need_send_movement = false;
    }

    // ===== ÉTAPE 3: ENABLE MOTEURS (une seule fois) =====
    if (!motors_enabled) {
        holonomic_basis_ptr->enable_motors();
        motors_enabled = true;
    }

    // ===== ÉTAPE 4: TÉLÉMÉTRIE VERS PI =====
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