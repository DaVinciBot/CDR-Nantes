// External libraries used: Arduino
#include <Arduino.h>      // Arduino framework
#include <holonomic_basis.h>  // Holonomic Basis with MKS RS485
#include <config.h>           // Configuration file

// ============= SENSOR COMMUNICATION PROTOCOL =============
// Receive dx, dy, dtheta from teensy_capteur via Serial1
#define SYNC_START 0xAA
#define SYNC_END 0xBB

union FloatBytes {
    float value;
    uint8_t bytes[4];
};

struct SensorPacket {
    float dx;
    float dy;
    float dtheta;
};

// Sensor data reception state
SensorPacket sensor_data = {0.0f, 0.0f, 0.0f};
SensorPacket last_sensor_data = {0.0f, 0.0f, 0.0f};

// ============= PID CONTROLLERS =============
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

// ============= SENSOR DATA RECEPTION =============

/**
 * Non-blocking reception of sensor packet from teensy_capteur
 * Expected format: [0xAA] [dx:float] [dy:float] [dtheta:float] [0xBB]
 * Total: 14 bytes
 */
enum RxState {
    RX_SYNC_START,
    RX_DX,
    RX_DY,
    RX_DTHETA,
    RX_SYNC_END
};

static RxState rx_state = RX_SYNC_START;
static FloatBytes rx_buffer;
static uint8_t rx_byte_count = 0;
static SensorPacket rx_packet;

void receive_sensor_data() {
    while (SENSOR_BOARD_SERIAL.available()) {
        uint8_t byte = SENSOR_BOARD_SERIAL.read();
        
        switch (rx_state) {
            case RX_SYNC_START:
                if (byte == SYNC_START) {
                    rx_state = RX_DX;
                    rx_byte_count = 0;
                }
                break;
            
            case RX_DX:
                rx_buffer.bytes[rx_byte_count++] = byte;
                if (rx_byte_count == 4) {
                    rx_packet.dx = rx_buffer.value;
                    rx_state = RX_DY;
                    rx_byte_count = 0;
                }
                break;
            
            case RX_DY:
                rx_buffer.bytes[rx_byte_count++] = byte;
                if (rx_byte_count == 4) {
                    rx_packet.dy = rx_buffer.value;
                    rx_state = RX_DTHETA;
                    rx_byte_count = 0;
                }
                break;
            
            case RX_DTHETA:
                rx_buffer.bytes[rx_byte_count++] = byte;
                if (rx_byte_count == 4) {
                    rx_packet.dtheta = rx_buffer.value;
                    rx_state = RX_SYNC_END;
                    rx_byte_count = 0;
                }
                break;
            
            case RX_SYNC_END:
                if (byte == SYNC_END) {
                    // Valid packet received!
                    sensor_data = rx_packet;
                }
                rx_state = RX_SYNC_START;
                break;
        }
    }
}

/**
 * Apply sensor data (dx, dy, dtheta) to odometry
 * Forward sensor deltas to holonomic_basis for accumulation
 */
void update_odometry_from_sensors() {
    if (sensor_data.dx == 0 && sensor_data.dy == 0 && sensor_data.dtheta == 0) {
        return; // No new data
    }
    
    // Pass sensor deltas to holonomic_basis (accumulation happens there)
    holonomic_basis_ptr->update_from_sensor_deltas(
        (double)sensor_data.dx,
        (double)sensor_data.dy,
        (double)sensor_data.dtheta
    );
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
}

void setup() {
    // Initialisation communication
    com = new Com(&Serial, BAUDRATE);
    
    // Initialisation communication avec teensy_capteur (Serial4 = pins 16-17)
    // teensy_capteur TX (pin 16) → teensy_moteur Serial4 RX (pin 16)
    SENSOR_BOARD_SERIAL.begin(SENSOR_BOARD_BAUD);

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
    // ===== ÉTAPE 0: RÉCEPTION CAPTEURS (non-bloquant) =====
    // Lire les données du teensy_capteur (optical + IMU)
    receive_sensor_data();
    update_odometry_from_sensors();
    
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