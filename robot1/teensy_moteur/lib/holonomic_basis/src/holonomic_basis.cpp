/**
 * Implementation of Holonomic Basis for MKS SERVO57D over RS485.
 */

#include <Arduino.h>
#include <holonomic_basis.h>
#include <cstdio>

// Fonction utilitaire pour normaliser l'angle entre -PI et PI
double normalizeAngle(double theta) {
    // shift by +PI, take modulo 2*PI, remap to [0,2*PI)
    theta = fmodf(theta + PI, 2.0f * PI);
    if (theta < 0.0f) {
        theta += 2.0f * PI;
    }
    // shift back to [-PI, +PI)
    return theta - PI;
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
    this->X = x;
    this->Y = y;
    this->THETA = theta;
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


Point Holonomic_Basis::get_current_position() {
    Point position;
    noInterrupts(); // Section critique pour lire les doubles de manière atomique
    position.x = this->X;
    position.y = this->Y;
    position.theta = this->THETA;
    interrupts();
    return position;
}

/**
 * Update robot position (for sensor fusion from teensy_capteur)
 * Thread-safe: disables interrupts during critical section
 */
void Holonomic_Basis::set_position(double x, double y, double theta) {
    noInterrupts(); // Critical section for atomic update
    this->X = x;
    this->Y = y;
    this->THETA = theta;
    interrupts();
}

/**
 * Queue sensor deltas (dx, dy, dtheta) from teensy_capteur.
 * Fusion and final odometry integration are handled in update_odometry().
 */
void Holonomic_Basis::update_from_sensor_deltas(double dx_mm, double dy_mm, double dtheta) {
    uint32_t now_ms = millis();
    noInterrupts(); // Critical section for shared buffer update
    odo_data.pending_sensor_dx_mm += dx_mm;
    odo_data.pending_sensor_dy_mm += dy_mm;
    odo_data.pending_sensor_dtheta += dtheta;
    odo_data.pending_sensor_packets++;
    odo_data.last_sensor_packet_ms = now_ms;
    interrupts();
}

void Holonomic_Basis::update_odometry() {
    // ===== NON-BLOQUANT: Lire encodeurs depuis les buffers (mis à jour par loop()) =====
    // Au lieu d'appeler RS485 ici (qui bloquerait l'ISR 50ms), on lit les buffers pré-remplis
    // par read_encoders_nonblocking() exécutée dans loop()

    int64_t enc1 = odo_data.buffered_enc1;
    int64_t enc2 = odo_data.buffered_enc2;
    int64_t enc3 = odo_data.buffered_enc3;

    // Si les buffers n'ont jamais été remplis, garder dernière valeur
    // (update_odometry() gère le fallback gracieux)

    static bool is_first_run = true;
    if (is_first_run) {
        odo_data.last_enc1 = enc1;
        odo_data.last_enc2 = enc2;
        odo_data.last_enc3 = enc3;

        is_first_run = false;
        return; // On sort ! Pas de calcul de mouvement au démarrage.
    }

    double d1 = double(enc1 - odo_data.last_enc1);
    double d2 = double(enc2 - odo_data.last_enc2);
    double d3 = double(enc3 - odo_data.last_enc3);
    const double ENCODER_NOISE_THRESHOLD = 5.0; // counts
    if (abs(d1) < ENCODER_NOISE_THRESHOLD) d1 = 0.0;
    if (abs(d2) < ENCODER_NOISE_THRESHOLD) d2 = 0.0;
    if (abs(d3) < ENCODER_NOISE_THRESHOLD) d3 = 0.0;

    const double CTM = COUNTS_TO_MM;
    double w1_mm = d1 * CTM;
    double w2_mm = d2 * CTM;
    double w3_mm = d3 * CTM;
    
    // ============= CINÉMATIQUE DIRECTE (INVERSE MATHÉMATIQUE DE L'INVERSE) =============
    // Configuration physique confirmée : Roue1=Haut-Droite, Roue2=Haut-Gauche, Roue3=Arrière
    // Angles utilisés dans cinématique inverse : Roue1=120°, Roue2=240°, Roue3=0°
    //
    // Système à résoudre (de la cinématique inverse) :
    //   w1 = -0.5*vx + 0.866*vy + ω*R
    //   w2 = -0.5*vx - 0.866*vy + ω*R
    //   w3 = 1.0*vx + ω*R
    //
    // Solutions (inverse matricielle) :
    //   vx = (2*w3 - w1 - w2) / 3
    //   vy = (w1 - w2) / √3
    //   ω*R = (w1 + w2 + w3) / 3
    // ==================================================================================

    // Vitesse X (longitudinale - vers l'avant robot)
    double dx_enc_robot = (2.0 * w3_mm - w1_mm - w2_mm) / 3.0;

    // Vitesse Y (latérale - vers la gauche robot)
    double dy_enc_robot = (w1_mm - w2_mm) / sqrt(3.0);  // 1.73205... = sqrt(3)

    // Rotation angulaire (en radians) : ω*R = (w1+w2+w3)/3, donc ω = (w1+w2+w3)/(3*R)
    // Note: les w_mm contiennent déjà la contribution (ω*R) de la cinématique inverse
    double omega_enc = (w1_mm + w2_mm + w3_mm) / (3.0 * robot_radius);

    // Transformation encodeurs dans le repère monde
    double cos_theta = cos(this->THETA);
    double sin_theta = sin(this->THETA);
    double dx_enc_world = dx_enc_robot * cos_theta - dy_enc_robot * sin_theta;
    double dy_enc_world = dx_enc_robot * sin_theta + dy_enc_robot * cos_theta;

    // Accumulateurs pour statistiques (debug possible)
    odo_data.encoder_x_acc += dx_enc_world;
    odo_data.encoder_y_acc += dy_enc_world;

    // External sensor freshness: if no packet for too long, fallback to encoders only.
    const uint32_t SENSOR_TIMEOUT_MS = 100;
    uint32_t now_ms = millis();
    bool has_new_sensor_data = false;
    double sensor_dx_robot = 0.0;
    double sensor_dy_robot = 0.0;
    double sensor_dtheta = 0.0;

    if (odo_data.last_sensor_packet_ms != 0 &&
        (now_ms - odo_data.last_sensor_packet_ms) <= SENSOR_TIMEOUT_MS) {
        if (odo_data.pending_sensor_packets > 0) {
            sensor_dx_robot = odo_data.pending_sensor_dx_mm;
            sensor_dy_robot = odo_data.pending_sensor_dy_mm;
            sensor_dtheta = odo_data.pending_sensor_dtheta;
            has_new_sensor_data = true;
        }

        // Consume queued deltas once per ISR cycle.
        odo_data.pending_sensor_dx_mm = 0.0;
        odo_data.pending_sensor_dy_mm = 0.0;
        odo_data.pending_sensor_dtheta = 0.0;
        odo_data.pending_sensor_packets = 0;
    } else if (odo_data.last_sensor_packet_ms != 0 &&
               (now_ms - odo_data.last_sensor_packet_ms) > SENSOR_TIMEOUT_MS) {
        // Stale stream: discard pending deltas and fallback on encoders.
        odo_data.pending_sensor_dx_mm = 0.0;
        odo_data.pending_sensor_dy_mm = 0.0;
        odo_data.pending_sensor_dtheta = 0.0;
        odo_data.pending_sensor_packets = 0;
    }

    double dx_final_world = dx_enc_world;
    double dy_final_world = dy_enc_world;

    if (has_new_sensor_data) {
        // teensy_capteur sends deltas in robot frame: convert to world frame before fusion.
        double sensor_dx_world = sensor_dx_robot * cos_theta - sensor_dy_robot * sin_theta;
        double sensor_dy_world = sensor_dx_robot * sin_theta + sensor_dy_robot * cos_theta;

        const double ALPHA = 0.20;  // 20% encoders + 80% optical/IMU board
        dx_final_world = ALPHA * dx_enc_world + (1.0 - ALPHA) * sensor_dx_world;
        dy_final_world = ALPHA * dy_enc_world + (1.0 - ALPHA) * sensor_dy_world;

        this->THETA = normalizeAngle(this->THETA + sensor_dtheta);
    } else {
        this->THETA = normalizeAngle(this->THETA + omega_enc);
    }

    // Normalisation θ ∈ [-π, +π]
    while (this->THETA >  M_PI) this->THETA -= 2.0 * M_PI;
    while (this->THETA < -M_PI) this->THETA += 2.0 * M_PI;

    // Intégration position X,Y
    if (has_new_sensor_data || use_encoders) {
        this->X += dx_final_world;
        this->Y += dy_final_world;
    }

    // Sauvegarder pour prochaine itération
    odo_data.last_enc1 = enc1;
    odo_data.last_enc2 = enc2;
    odo_data.last_enc3 = enc3;
}

// Calcul de la boucle d'asservissement (PID + Cinématique Inverse)
void Holonomic_Basis::handle(Point target_position, Com* com) {
    // 1. Calcul des erreurs dans le référentiel Monde
    double xerr = target_position.x - this->X;
    double yerr = target_position.y - this->Y;
    double theta_error = normalizeAngle(target_position.theta - this->THETA);

    #ifdef WEBOTS_SIMULATION
    static uint32_t debug_err = 0;
    if (++debug_err > 100) {  // ~1 seconde à 100Hz
        Serial.printf("DEBUG: Erreurs: ΔX=%.1f ΔY=%.1f Δθ=%.2f\n", xerr, yerr, theta_error);
        debug_err = 0;
    }
    #endif
    // 2. Calcul des vitesses cibles via PID (référentiel Monde)
    double vx_world, vy_world, omega;
    if (!this->use_pid_control) {
        // Mode simple proportionnel sans PID
    double gain_translation = 2.0; // Gain pour la translation (ajustable)
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
    if (distance_error < 1.0 && angle_error < 0.02) {  // 1mm et ~1.15°
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
    double w1 = -(0.5 * vx_rpm - sqrt(3.0)/2.0 * vy_rpm - omega_rpm);
    // Roue 2 avec axe à 240° : cos(240°) = -0.5, sin(240°) = -0.866
    double w2 = -(0.5 * vx_rpm +sqrt(3.0)/2.0 * vy_rpm -omega_rpm);
    // Roue 3 avec axe à 0° : cos(0°) = +1.0, sin(0°) = 0
    double w3 = 1.0*vx_rpm + omega_rpm;

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
    if (i++ > 100) {  // ~1 seconde
        //printf(" PID: Target[%.1f,%.1f,%.2f] Actual[%.1f,%.1f,%.2f] Err[%.1f,%.1f,%.2f]\n",
        //       target_position.x, target_position.y, target_position.theta,
        //       this->X, this->Y, this->THETA,
        //       xerr, yerr, theta_error);
        //printf(" Cmds: Vx=%.1f Vy=%.1f ω=%.2f -> W1=%.0f W2=%.0f W3=%.0f RPM (filtered)\n",
         //      vx_world, vy_world, omega, filtered_wheel1_rpm, filtered_wheel2_rpm, filtered_wheel3_rpm);
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
        // Stocker dans buffers pour que update_odometry() (ISR) puisse les lire
        uint32_t now_ms = millis();
        noInterrupts();
        odo_data.buffered_enc1 = enc1;
        odo_data.buffered_enc2 = enc2;
        odo_data.buffered_enc3 = enc3;
        odo_data.buffer_timestamp = now_ms;
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