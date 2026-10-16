/**
 * Implementation of Holonomic Basis for MKS SERVO57D over RS485.
 */

#include <Arduino.h>
#include <holonomic_basis.h>
#include <cstdio>

// Fonction utilitaire pour normaliser l'angle entre -PI et PI
double normalizeAngle(double theta) {
    // shift by +PI, take modulo 2*PI, remap to [0,2*PI)
    theta = fmod(theta + M_PI, 2.0 * M_PI);
    if (theta < 0.0) {
        theta += 2.0 * M_PI;
    }
    // shift back to [-PI, +PI)
    return theta - M_PI;
}

// Constructor
Holonomic_Basis::Holonomic_Basis(double robot_radius,
                                 double wheel_diameter,
                                 double max_speed_rpm,
                                 const PID& x_pid,
                                 const PID& y_pid,
                                 const PID& theta_pid)
    : x_pid(x_pid),
      y_pid(y_pid),
      theta_pid(theta_pid),
      robot_radius(robot_radius),
      wheel_diameter(wheel_diameter),
    max_speed_rpm(max_speed_rpm) {
    
    // Initialisation du filtre de lissage
    #ifdef SPEED_FILTER_ALPHA
        speed_filter_alpha = SPEED_FILTER_ALPHA;
    #else
        speed_filter_alpha = 0.3;  // Valeur par défaut
    #endif
    
    // Initialisation des pointeurs à null
    wheel1 = nullptr;
    wheel2 = nullptr;
    wheel3 = nullptr;
    mksGroup = nullptr;

    odo_data.last_enc1 = 0;
    odo_data.last_enc2 = 0;
    odo_data.last_enc3 = 0;
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    // Nettoyage de la mémoire si nécessaire
    delete wheel1;
    delete wheel2;
    delete wheel3;
    delete mksGroup;
    // The sensors are owned by the caller (attach_sensors).
}

// === DÉFINITION DES MOTEURS ===

void Holonomic_Basis::define_wheel1(HardwareSerial& serial, uint8_t addr) {
    wheel1 = new MKSServo(serial, addr, MKS_MSTEP);
    wheel1->begin(MKS_BAUDRATE);
}
void Holonomic_Basis::define_wheel2(HardwareSerial& serial, uint8_t addr) {
    wheel2 = new MKSServo(serial, addr, MKS_MSTEP);
    wheel2->begin(MKS_BAUDRATE);
}
void Holonomic_Basis::define_wheel3(HardwareSerial& serial, uint8_t addr) {
    wheel3 = new MKSServo(serial, addr, MKS_MSTEP);
    wheel3->begin(MKS_BAUDRATE);
}

// === INITIALISATION ===
void Holonomic_Basis::init_motors() {
    mksGroup = new MKSGroup(wheel1, wheel2, wheel3);
}
void Holonomic_Basis::init_holonomic_basis(double x, double y, double theta) {
    noInterrupts();
    this->X = x;
    this->Y = y;
    this->THETA = theta;
    if (estimator) estimator->reset(x, y, theta);
    optical_integrator.reset(x, y);
    interrupts();

    if (optical) optical->reset_tracking();
}

void Holonomic_Basis::calibrate_imu_origin() {
    if (use_imu && imu) imu->zero_heading();
    noInterrupts();
    this->THETA = 0.0;
    if (estimator) estimator->reset(this->X, this->Y, 0.0);
    interrupts();
}

// === ESTIMATEUR DE POSE ===

void Holonomic_Basis::set_estimator(pose::PoseEstimator* next) {
    if (!next) return;
    noInterrupts();
    next->reset(this->X, this->Y, this->THETA);
    estimator = next;
    interrupts();
}

bool Holonomic_Basis::select_estimator(pose::PoseEstimator* next) {
    if (!next) return false;
    // Only at rest: a switch while moving would hand over a pose mid-tick, and
    // the new estimator would start with a wrong velocity.
    if (filtered_wheel1_rpm != 0.0 || filtered_wheel2_rpm != 0.0 ||
        filtered_wheel3_rpm != 0.0) {
        return false;
    }
    set_estimator(next);
    return true;
}

// === GESTION ÉTAT MOTEURS ===

void Holonomic_Basis::enable_motors() {
    if (wheel1) wheel1->enable();
    if (wheel2) wheel2->enable();
    if (wheel3) wheel3->enable();
}

void Holonomic_Basis::disable_motors() {
    if (wheel1) wheel1->disable();
    if (wheel2) wheel2->disable();
    if (wheel3) wheel3->disable();
}

// === ODOMÉTRIE & PID ===
void Holonomic_Basis::attach_sensors(sensors::Bno085Sensor* imu_sensor,
                                     sensors::OtosSensor* optical_sensor) {
    noInterrupts();
    imu = imu_sensor;
    optical = optical_sensor;
    interrupts();
}

Point Holonomic_Basis::get_current_position() {
    Point position;
    noInterrupts(); // Section critique pour lire les doubles de manière atomique
    position.x = this->X;
    position.y = this->Y;
    position.theta = this->THETA;
    interrupts();
    return position;
}

// Builds the estimator input of the tick from the buffers filled in loop().
// Called from the ISR: no I/O here, only copies under noInterrupts().
void Holonomic_Basis::update_odometry() {
    // Ask loop() for the next sensor samples
    if (use_imu && imu) need_read_imu = true;
    if (use_optical_flow && optical) need_read_optical = true;

    int64_t enc1, enc2, enc3;
    uint32_t seq, enc_us;
    {
        noInterrupts();
        enc1 = odo_data.buffered_enc1;
        enc2 = odo_data.buffered_enc2;
        enc3 = odo_data.buffered_enc3;
        seq = odo_data.buffer_seq;
        enc_us = odo_data.buffer_timestamp_us;
        interrupts();
    }
    const uint32_t now_us = micros();

    pose::EstimatorInput in = pose::empty_input();

    // First tick: no motion computed at startup, only the references are taken.
    if (!odo_data.started) {
        in.first_frame = true;
        odo_data.started = true;
    } else {
        // Encoder counts increase in the opposite direction to commanded RPM on
        // all 3 motors. Negate so that positive d = forward wheel rotation, the
        // convention of the estimators (lib/kalman/include/kinematics.h).
        in.d1 = -static_cast<int32_t>(enc1 - odo_data.last_enc1);
        in.d2 = -static_cast<int32_t>(enc2 - odo_data.last_enc2);
        in.d3 = -static_cast<int32_t>(enc3 - odo_data.last_enc3);
        in.dt_us = now_us - odo_data.last_tick_us;
    }

    // The deltas span exactly one tick only if the buffer was refreshed this tick
    // AND the previous one. After a missed read they are zero (stale buffer),
    // then cover two ticks at once: a speed computed over one dt would be wrong.
    const bool fresh_read = (seq != odo_data.last_seq);
    in.encoders_fresh = !in.first_frame && fresh_read && odo_data.prev_read_fresh;
    in.encoder_age_us = now_us - enc_us;

    // IMU: heading in the table frame, origin set by calibrate_imu_origin()
    if (use_imu && imu) {
        const sensors::HeadingSample h = imu->latest();
        in.imu_valid = h.valid;
        in.heading_rad = h.heading_rad;
        in.imu_timestamp_us = h.timestamp_us;
    }

    // Optical: robot-frame velocity for the EKF, and a table-frame pose for the
    // complementary filter, rebuilt by rotating each new robot-frame step by the
    // heading of the previous tick. A sample is integrated once only.
    if (use_optical_flow && optical) {
        const sensors::OpticalSample o = optical->latest();
        if (o.valid) {
            if (o.seq != odo_data.last_optical_seq) {
                odo_data.last_optical_seq = o.seq;
                optical_integrator.add(o.dx_mm, o.dy_mm, this->THETA);
                in.optical_velocity_valid = true;
                in.optical_vx_mm_s = o.vx_mm_s;
                in.optical_vy_mm_s = o.vy_mm_s;
            }
            in.optical_pose_valid = true;
            in.optical_x_mm = static_cast<float>(optical_integrator.x_mm());
            in.optical_y_mm = static_cast<float>(optical_integrator.y_mm());
            in.optical_timestamp_us = o.timestamp_us;
        }
    }

    odo_data.last_enc1 = enc1;
    odo_data.last_enc2 = enc2;
    odo_data.last_enc3 = enc3;
    odo_data.last_seq = seq;
    odo_data.prev_read_fresh = fresh_read;
    odo_data.last_tick_us = now_us;

    if (!estimator) return;
    estimator->step(in);
    const pose::PoseEstimate p = estimator->pose();
    this->X = p.x_mm;
    this->Y = p.y_mm;
    this->THETA = p.theta_rad;
    #ifdef MODE_ODO_FIXE
        // Test mode: position frozen at (0, 0), heading still estimated
        this->X = 0.0;
        this->Y = 0.0;
    #endif
}

// Calcul de la boucle d'asservissement (PID + Cinématique Inverse)
void Holonomic_Basis::handle(Point target_position, Com* com) {
    // 1. Calcul des erreurs dans le référentiel Monde
    #ifdef MODE_ODO_FIXE
        // Mode test : ignore la position réelle, erreur = cible directement
        double xerr        = target_position.x;
        double yerr        = target_position.y;
        double theta_error = normalizeAngle(target_position.theta);
    #else
        // Mode normal : erreur = cible - position réelle
        double xerr        = target_position.x - this->X;
        double yerr        = target_position.y - this->Y;
        double theta_error = normalizeAngle(target_position.theta - this->THETA);
    #endif

    // 2. Calcul des vitesses cibles via PID (référentiel Monde)
    double vx_world, vy_world, omega;
    if (!this->use_pid_control) {
        // Mode simple proportionnel sans PID
        double gain_translation = 1.0; // Gain pour la translation (ajustable)
        double gain_rotation = 1.0;    // Gain pour la rotation (ajustable)
        
        vx_world = gain_translation * xerr;
        vy_world = gain_translation * yerr;
        omega = gain_rotation * theta_error;

    double max_linear_speed = 100.0; // Limite la vitesse linéaire
    double max_angular_speed = 1.0;   // Limite la vitesse angulaire


    vx_world = constrain(vx_world, -max_linear_speed, max_linear_speed);
    vy_world = constrain(vy_world, -max_linear_speed, max_linear_speed);
    omega = constrain(omega, -max_angular_speed, max_angular_speed);
    } else {
        // Mode PID complet
        vx_world = this->x_pid.compute(xerr);
        vy_world = this->y_pid.compute(yerr);
        omega = this->theta_pid.compute(theta_error);
    }
    
    double distance_error = sqrt(xerr*xerr + yerr*yerr);
    double angle_error = fabs(theta_error);
    
    // Zone morte
    if (distance_error < 10.0 && angle_error < 0.15) {  // 1mm et ~1.15°
        vx_world = 0.0;
        vy_world = 0.0;
        omega = 0.0;

        last_wheel1_rpm = 0.0;
        last_wheel2_rpm = 0.0;
        last_wheel3_rpm = 0.0;
        
        // Réinitialiser aussi les vitesses filtrées pour un arrêt propre
        filtered_wheel1_rpm = 0.0;
        filtered_wheel2_rpm = 0.0;
        filtered_wheel3_rpm = 0.0;
        return;  // Sortie anticipée
    }
    


    // 4. Changement de repère : Monde -> Robot
    double cos_theta = cosf(this->THETA);
    double sin_theta = sinf(this->THETA);
    
    double vx_robot = cos_theta * vx_world + sin_theta * vy_world;
    double vy_robot = -sin_theta * vx_world + cos_theta * vy_world;
    
    // 5. Cinématique Inverse : Vitesse Robot -> Vitesse Roues (RPM)
    double speed_factor = 60.0 / wheel_circumference();

    double vx_rpm = vx_robot * speed_factor;
    double vy_rpm = vy_robot * speed_factor;
    double omega_rpm = omega * robot_radius * speed_factor;
    
    //Equation de mouvements
    // Roue 1 avec axe à 120° : cos(120°) = -0.5, sin(120°) = +0.866
    double w1 = -0.5*vx_rpm + 0.866*vy_rpm - omega_rpm;
    double w2 = -0.5*vx_rpm - 0.866*vy_rpm - omega_rpm;
    double w3 = +1.0*vx_rpm + 0.0*vy_rpm   - omega_rpm;
    // DEBUG: Affichage des vitesses calculées
    
    // Normalisation proportionnelle pour préserver la direction du mouvement
    // Si une roue dépasse max_speed_rpm, on réduit toutes les roues du même ratio
    double max_wheel_speed = fmax(fmax(fabs(w1), fabs(w2)), fabs(w3));
    if (max_wheel_speed > max_speed_rpm) {
        double scale = max_speed_rpm / max_wheel_speed;
        last_wheel1_rpm = w1 * scale;
        last_wheel2_rpm = w2 * scale;
        last_wheel3_rpm = w3 * scale;
    } else {
        last_wheel1_rpm = w1;
        last_wheel2_rpm = w2;
        last_wheel3_rpm = w3;
    }

    // Application du filtre passe-bas pour lisser les changements de vitesse
    // Formule : filtered = alpha * new_value + (1 - alpha) * old_value
    filtered_wheel1_rpm = speed_filter_alpha * last_wheel1_rpm + (1.0 - speed_filter_alpha) * filtered_wheel1_rpm;
    filtered_wheel2_rpm = speed_filter_alpha * last_wheel2_rpm + (1.0 - speed_filter_alpha) * filtered_wheel2_rpm;
    filtered_wheel3_rpm = speed_filter_alpha * last_wheel3_rpm + (1.0 - speed_filter_alpha) * filtered_wheel3_rpm;
    
    // Normalisation proportionnelle APRÈS filtrage pour garantir le respect des limites
    // Cela préserve la direction du mouvement même après le lissage
    double max_filtered_speed = fmax(fmax(fabs(filtered_wheel1_rpm), fabs(filtered_wheel2_rpm)), fabs(filtered_wheel3_rpm));
    if (max_filtered_speed > max_speed_rpm) {
        double scale_filtered = max_speed_rpm / max_filtered_speed;
        filtered_wheel1_rpm *= scale_filtered;
        filtered_wheel2_rpm *= scale_filtered;
        filtered_wheel3_rpm *= scale_filtered;
    }

    static int i = 0;
    if (i++ > 50) {  // ~0.5 seconde à 100Hz
        #ifdef MODE_ODO_FIXE
            Serial.printf("[HANDLE/FIXE] Target[%+6.0f,%+6.0f,%+.2f] Err[%+6.0f,%+6.0f,%+.2f] CMD[W1=%+6.0f W2=%+6.0f W3=%+6.0f]\n",
        #else
            Serial.printf("[HANDLE/ENC]  Target[%+6.0f,%+6.0f,%+.2f] Err[%+6.0f,%+6.0f,%+.2f] CMD[W1=%+6.0f W2=%+6.0f W3=%+6.0f]\n",
        #endif
                          target_position.x, target_position.y, target_position.theta,
                          xerr, yerr, theta_error,
                          filtered_wheel1_rpm, filtered_wheel2_rpm, filtered_wheel3_rpm);
        i = 0;
    }

}

// === EXÉCUTION DU MOUVEMENT (KARIBOU MOTION) ===

void Holonomic_Basis::run_motors() {
    // Obsolète avec KaribouMotion géré par interruption, mais gardé pour compatibilité API
}

// Envoie les vitesses roues directement aux MKS en RPM
void Holonomic_Basis::execute_movement() {
    if (mksGroup) {
        mksGroup->setSpeedsSynced(filtered_wheel1_rpm,
                                  filtered_wheel2_rpm,
                                  filtered_wheel3_rpm,
                                  MKS_ACC);
    }
}

// Arrêt d'urgence
void Holonomic_Basis::emergency_stop() {
    if (mksGroup) {
        mksGroup->emergencyStopAll();
    }
}

// ===== ARCHITECTURE NON-BLOQUANTE: RS485 SORTIS DE L'ISR =====

// Lire les encodeurs DEPUIS loop() - rafale synchronisée, ~2ms total
bool Holonomic_Basis::read_encoders_nonblocking() {
    if (!mksGroup) return false;
    
    int64_t enc1 = 0, enc2 = 0, enc3 = 0;
    
    // Appel RS485 bloquant optimisé : envoie 3 requêtes en rafale, lit 3 réponses en parallèle
    // ~2ms au lieu de ~150ms avec readAllEncoders() séquentiel
    bool success = mksGroup->readAllEncodersSynced(enc1, enc2, enc3);
    
    if (success) {
        // Stocker dans buffers pour que update_odometry() (ISR) puisse les lire.
        // 64-bit values: written with interrupts off so the ISR never sees half.
        noInterrupts();
        odo_data.buffered_enc1 = enc1;
        odo_data.buffered_enc2 = enc2;
        odo_data.buffered_enc3 = enc3;
        odo_data.buffer_timestamp_us = micros();
        odo_data.buffer_seq++;
        interrupts();
        
        #ifdef DEBUG_RS485
        static uint32_t enc_read_counter = 0;
        if (++enc_read_counter >= 10) {  // Log toutes les 10 lectures
            enc_read_counter = 0;
            Serial.printf("[RS485 OK] Enc: %lld, %lld, %lld\n", enc1, enc2, enc3);
        }
        #endif
    } else {
        // Timeout RS485 - log mais pas de crash
        static uint32_t enc_fail_counter = 0;
        if (++enc_fail_counter >= 5) {  // Log une fois sur 5
            enc_fail_counter = 0;
            Serial.printf("[RS485 TIMEOUT] readAllEncoders() failed, retry next loop\n");
        }
    }
    
    return success;
}

// Envoyer les commandes vitesse DEPUIS loop() - 30ms max (3x 10ms), pas d'impact sur ISR
bool Holonomic_Basis::send_movement_commands_nonblocking() {
    if (!mksGroup) return false;
    
    // Appel RS485 bloquant - c'est OK ici, on est dans loop() pas dans ISR
    // setSpeedsSynced envoie 3 commandes 0xF6 (10ms timeout chacune)
    mksGroup->setSpeedsSynced(filtered_wheel1_rpm,
                              filtered_wheel2_rpm,
                              filtered_wheel3_rpm,
                              MKS_ACC);
    
    #ifdef DEBUG_RS485
    static uint32_t movement_counter = 0;
    if (++movement_counter >= 10) {  // Log toutes les 10 fois
        movement_counter = 0;
        Serial.printf("[RS485 MOVE] W1: %.0f, W2: %.0f, W3: %.0f RPM\n",
                      filtered_wheel1_rpm, filtered_wheel2_rpm, filtered_wheel3_rpm);
    }
    #endif
    
    return true;  // On considère OK même si timeout silencieux (moteur peut fonctionner)
}
