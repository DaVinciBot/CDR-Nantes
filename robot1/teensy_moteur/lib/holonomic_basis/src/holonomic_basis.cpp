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
    pmw3901 = nullptr;
    bno085 = nullptr;  

    odo_data.last_enc1 = 0;
    odo_data.last_enc2 = 0;
    odo_data.last_enc3 = 0;

    odo_data.optical_x_acc = 0.0;
    odo_data.optical_y_acc = 0.0;
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    // Nettoyage de la mémoire si nécessaire
    delete wheel1;
    delete wheel2;
    delete wheel3;
    delete mksGroup;
    delete pmw3901;
    // Adafruit_BNO08x expose une classe polymorphique sans destructeur virtuel.
    // Eviter delete ici pour ne pas déclencher -Wdelete-non-virtual-dtor.
    bno085 = nullptr;
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

    if (pmw3901) {
        #ifdef WEBOTS_SIMULATION
            ((PAA5100*)pmw3901)->reset(); 
        #endif
        
    }
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
void Holonomic_Basis::init_sensors() {
    // === CAPTEUR OPTIQUE PAA5100JE ===
    #ifdef WEBOTS_SIMULATION
        pmw3901 = new PAA5100();
        if (pmw3901) {
            pmw3901->begin();
        }
    #else
        // ROBOT RÉEL - Bitcraze PMW3901
        pmw3901 = new Bitcraze_PMW3901(PAA5100_CS_PIN);
        if (pmw3901) {
            // Timeout short: if sensor doesn't respond in 100ms, move on
            uint32_t sensorStart = millis();
            while (!pmw3901->begin() && (millis() - sensorStart) < 100) {
                delay(10);
            }
        }
    #endif

    // === IMU BNO085 ===
    #ifdef WEBOTS_SIMULATION
        bno085 = new Adafruit_BNO08x();
        if (bno085 && bno085->begin_I2C()) {
            bno085->enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 100Hz
            odo_data.imu_calibrated = true;
            odo_data.imu_yaw_offset = 0.0;
        }
    #else
        // ROBOT RÉEL
        bno085 = new Adafruit_BNO08x(BNO085_RESET_PIN);
        
        if (bno085 && bno085->begin_I2C()) {
            Wire.setClock(400000); // I2C Fast Mode
            bno085->enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 100Hz
            
            // Brief calibration attempt with timeout (don't block forever)
            uint32_t calibStart = millis();
            sh2_SensorValue_t sv;
            
            while ((millis() - calibStart) < 500) {  // 500ms max timeout
                if (bno085->getSensorEvent(&sv)) {
                    if (sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
                        float r = sv.un.gameRotationVector.real;
                        float i = sv.un.gameRotationVector.i;
                        float j = sv.un.gameRotationVector.j;
                        float k = sv.un.gameRotationVector.k;
                        
                        odo_data.imu_yaw_offset = atan2(2.0f*(r*k + i*j), 1.0f-2.0f*(j*j + k*k));
                        break;
                    }
                }
                delay(10);
            }
            
            odo_data.imu_calibrated = true;  // Mark calibrated regardless (use 0 offset as default)
        } else {
            // I2C begin failed - IMU not available
            odo_data.imu_calibrated = false;
        }
    #endif
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

// FONCTION PRINCIPALE - ODOMÉTRIE OPTIQUE
void Holonomic_Basis::update_optical_odometry(double dtheta_robot) {
    if (!pmw3901) return;
    
    int16_t deltaX = 0, deltaY = 0;
    double dx_mm = 0.0, dy_mm = 0.0;
    
    // LECTURE CAPTEUR (API différente selon le mode)
    #ifdef WEBOTS_SIMULATION
        // Mock Webots : retourne directement en mm
        pmw3901->readMotion(deltaX, deltaY);
        dx_mm = (double)deltaX;
        dy_mm = (double)deltaY;
    #else
        // Vrai capteur : retourne des counts
        pmw3901->readMotionCount(&deltaX, &deltaY);
        dx_mm = deltaX * OPTICAL_SCALE;  // Conversion counts → mm
        dy_mm = deltaY * OPTICAL_SCALE;
    #endif
    
    // Ignore la première lecture (souvent aberrante)
    static bool is_first_run_opt = true;
    if (is_first_run_opt) {
        is_first_run_opt = false;
        return; 
    }

    // Debug périodique AVANT filtre (pour voir fréquence réelle d'appel)
    static uint32_t debug_cnt = 0;
    bool should_print = (++debug_cnt >= 200);  // Toutes les 2 secondes à 100Hz
    if (should_print) {
        debug_cnt = 0;
    }
    
    // 1. Transformation Capteur → Robot
    double c_mnt = cos(OPTICAL_MOUNT_ANGLE);
    double s_mnt = sin(OPTICAL_MOUNT_ANGLE);
    double dx_robot = dx_mm * c_mnt - dy_mm * s_mnt;
    double dy_robot = dx_mm * s_mnt + dy_mm * c_mnt;
    
    // 2. Compensation effet centrifuge (rotation robot créant faux mouvement)
    dx_robot -= -OPTICAL_OFFSET_Y * dtheta_robot;
    dy_robot -=  OPTICAL_OFFSET_X * dtheta_robot;

    // 3. Transformation Robot → Monde
    double cos_theta = cos(this->THETA);
    double sin_theta = sin(this->THETA);
    double dx_world = dx_robot * cos_theta - dy_robot * sin_theta;
    double dy_world = dx_robot * sin_theta + dy_robot * cos_theta;

    // 3.5. Filtrage outliers basé sur magnitude (protection robot réel)
    double magnitude = sqrt(dx_world*dx_world + dy_world*dy_world);
    bool is_outlier = false;
    
    // Rejet outliers : magnitude > 15mm en une lecture = physiquement impossible
    // (robot max ~300mm/s @ 100Hz = 3mm/lecture max attendu)
    if (magnitude > 15.0) {
        is_outlier = true;
        odo_data.optical_outlier_count++;
        if (should_print) {
            //printf(" OPTIQUE: Outlier rejeté! Mag=%.1fmm > 15mm (X:%d Y:%d)\n", 
            //       magnitude, deltaX, deltaY);
        }
    }
    
    // Filtre bruit au repos : magnitude < 2mm = oscillation capteur ±1mm
    if (magnitude < 2.0 && !is_outlier) {
        // Considérer comme bruit, ne pas accumuler
        dx_world = 0.0;
        dy_world = 0.0;
    }

    // 4. Filtrage anti-bruit UNIQUEMENT au repos (simulation Webots)
    #ifdef WEBOTS_SIMULATION
        // Détection repos : vitesses des roues nulles ET encodeurs immobiles
        bool robot_at_rest = (abs(this->last_wheel1_rpm) < 1.0 && 
                             abs(this->last_wheel2_rpm) < 1.0 && 
                             abs(this->last_wheel3_rpm) < 1.0);
        
        if (robot_at_rest) {
            // Robot immobile : IGNORER le GPS (bruit 2-3mm par cycle)
            // On fait confiance aux encodeurs qui montrent d[0,0,0]
            static uint32_t noise_filter_debug = 0;
            double movement_magnitude = sqrt(dx_world*dx_world + dy_world*dy_world);
            if (should_print && movement_magnitude > 0.5 && ++noise_filter_debug >= 10) {
                noise_filter_debug = 0;
                //printf(" GPS au repos: Bruit ignoré (%.2fmm) - encodeurs prioritaires\n", 
                //       movement_magnitude);
            }
            // Pas d'accumulation optique au repos
        } else {
            // Robot en mouvement : accumulation GPS directe (si non rejeté)
            if (!is_outlier) {
                odo_data.optical_x_acc += dx_world;
                odo_data.optical_y_acc += dy_world;
                odo_data.optical_valid_count++;
            }
        }
    #else
        // Robot réel : accumulation avec filtrage outliers
        if (!is_outlier) {
            odo_data.optical_x_acc += dx_world;
            odo_data.optical_y_acc += dy_world;
            odo_data.optical_valid_count++;
        }
    #endif
    
    // 5. Debug périodique avec filtre d'affichage uniquement
    if (should_print) {
        // Filtre de bruit pour logs uniquement (pas d'impact sur accumulation)
        bool is_display_noise = (abs(dx_mm) < 0.1 && abs(dy_mm) < 0.1);
        if (is_display_noise) {
            //printf("📷 OPTIQUE: [FILTRÉ AFFICHAGE] raw=[%4d,%4d]mm → %.3f,%.3f < 0.1mm | pos=[%7.1f,%7.1f]mm\n", 
            //       deltaX, deltaY, dx_mm, dy_mm,
            //       odo_data.optical_x_acc, odo_data.optical_y_acc);
        } else {
            //double movement_magnitude = sqrt(dx_world*dx_world + dy_world*dy_world);
            //printf("📷 OPTIQUE: raw=[%4d,%4d]mm | robot=[%6.2f,%6.2f]mm | world=[%6.2f,%6.2f]mm (%.2fmm) | pos=[%7.1f,%7.1f]mm\n",
            //       deltaX, deltaY, dx_robot, dy_robot, dx_world, dy_world, movement_magnitude,
            //       odo_data.optical_x_acc, odo_data.optical_y_acc);
        }
    }
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
    static double last_theta_enc = 0.0;
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

    // External sensor freshness: if no packet for too long, fallback to encoders only.
    const uint32_t SENSOR_TIMEOUT_MS = 100;
    uint32_t now_ms = millis();
    bool sensor_packet_recent = false;
    uint16_t sensor_packet_count = 0;
    double sensor_dx_world = 0.0;
    double sensor_dy_world = 0.0;
    double sensor_dtheta = 0.0;

    if (odo_data.last_sensor_packet_ms != 0 &&
        (now_ms - odo_data.last_sensor_packet_ms) <= SENSOR_TIMEOUT_MS) {
        sensor_packet_recent = true;
        sensor_packet_count = odo_data.pending_sensor_packets;
        sensor_dx_world = odo_data.pending_sensor_dx_mm;
        sensor_dy_world = odo_data.pending_sensor_dy_mm;
        sensor_dtheta = odo_data.pending_sensor_dtheta;

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

    bool has_new_sensor_data = sensor_packet_recent && (sensor_packet_count > 0);

    //  Appel du capteur optique AVANT check encodeurs (pour avoir logs même au repos)
    bool optical_active = false;
    double dx_optical_world = 0.0;
    double dy_optical_world = 0.0;
    if (has_new_sensor_data) {
        // teensy_capteur already provides deltas in the robot odometry frame.
        dx_optical_world = sensor_dx_world;
        dy_optical_world = sensor_dy_world;
        optical_active = true;
    }

    if (!optical_active && use_optical_flow && pmw3901) {
        double prev_acc_x = odo_data.optical_x_acc;
        double prev_acc_y = odo_data.optical_y_acc;
        
        // omega_enc provisoire = 0 si encodeurs immobiles
        double omega_temp = 0.0;
        if (d1 != 0.0 || d2 != 0.0 || d3 != 0.0) {
            double w1_mm = d1 * COUNTS_TO_MM;
            double w2_mm = d2 * COUNTS_TO_MM;
            double w3_mm = d3 * COUNTS_TO_MM;
            omega_temp = (w1_mm + w2_mm + w3_mm) / (3.0 * robot_radius);
        }
        
        update_optical_odometry(omega_temp);
        
        double diff_opt_x = odo_data.optical_x_acc - prev_acc_x;
        double diff_opt_y = odo_data.optical_y_acc - prev_acc_y;
        
        // Détection rotation pure : pattern encodeurs (3 roues même sens/vitesse)
        // Rotation pure si d1 ≈ d2 ≈ d3 (toutes tournent ensemble)
        #ifdef WEBOTS_SIMULATION
        bool is_pure_rotation = false;
        #endif
        if (abs(omega_temp) > 0.005) { // Rotation détectée (> 0.005 rad = 0.3°)
            // Vérifier si les 3 roues tournent dans le même sens
            double d1_abs = abs(d1);
            double d2_abs = abs(d2);
            double d3_abs = abs(d3);
            
            // Vérifier même signe (toutes positives ou toutes négatives)
            bool same_sign = (d1 * d2 > 0) && (d2 * d3 > 0);
            
            // Vérifier vitesses similaires (tolérance 20% pour imprécisions)
            double d_avg = (d1_abs + d2_abs + d3_abs) / 3.0;
            if (d_avg > 50.0) { // Mouvement significatif (>50 counts)
                double d1_diff = abs(d1_abs - d_avg) / d_avg;
                double d2_diff = abs(d2_abs - d_avg) / d_avg;
                double d3_diff = abs(d3_abs - d_avg) / d_avg;
                
                // Rotation pure = même signe ET vitesses similaires (<20% écart)
                if (same_sign && d1_diff < 0.2 && d2_diff < 0.2 && d3_diff < 0.2) {
                    #ifdef WEBOTS_SIMULATION
                    is_pure_rotation = true;
                    #endif
                }
            }
        }
        
        // En simulation: GPS ground truth TOUJOURS prioritaire (même si delta=0)
        // SAUF pendant rotations pures (filtrage mouvements parasites)
        // En réel: seuil à 0.01mm pour éviter bruit capteur
        #ifdef WEBOTS_SIMULATION
            if (is_pure_rotation) {
                // Rotation pure détectée : IGNORER mouvements X/Y optiques (glissement)
                // On garde uniquement theta de l'IMU
                dx_optical_world = 0.0;
                dy_optical_world = 0.0;
                optical_active = false;  // Fallback encodeurs (qui devraient aussi être ≈0)
                
                static uint32_t pure_rot_debug = 0;
                if (++pure_rot_debug >= 50) { // Log toutes les 0.5s
                    pure_rot_debug = 0;
                    //printf(" ROTATION PURE: Filtrage X/Y optique (ω=%.3f rad, GPS filtré=[%.1f,%.1f]mm)\n",
                    //       omega_temp, diff_opt_x, diff_opt_y);
                }
            } else {
                // Mouvement normal : GPS Mock prioritaire
                dx_optical_world = diff_opt_x;
                dy_optical_world = diff_opt_y;
                optical_active = true;  // Jamais de fallback encodeurs !
            }
        #else
            // Robot réel: filtrer le bruit du capteur optique
            const double OPTICAL_THRESHOLD = 0.01; // 10 microns
            if (abs(diff_opt_x) > OPTICAL_THRESHOLD || abs(diff_opt_y) > OPTICAL_THRESHOLD) {
                dx_optical_world = diff_opt_x;
                dy_optical_world = diff_opt_y;
                optical_active = true;
            }
        #endif
    }

    // Ne pas skipper même au repos : permet le logging optique continu
    // (Au repos: encodeurs=0, optique filtré, mais logs debug utiles)

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
    double dx_enc = (2.0 * w3_mm - w1_mm - w2_mm) / 3.0;

    // Vitesse Y (latérale - vers la gauche robot)
    double dy_enc = (w1_mm - w2_mm) / sqrt(3.0);  // 1.73205... = sqrt(3)

    // Rotation angulaire (en radians) : ω*R = (w1+w2+w3)/3, donc ω = (w1+w2+w3)/(3*R)
    // Note: les w_mm contiennent déjà la contribution (ω*R) de la cinématique inverse
    double omega_enc = (w1_mm + w2_mm + w3_mm) / (3.0 * robot_radius);
    
    
    static uint32_t enc_debug_counter = 0;
        if (++enc_debug_counter >= 20) {  // Affichage toutes les 0.2s au lieu de 2s
            enc_debug_counter = 0;
            //printf(" ENC: d[%+4.0f,%+4.0f,%+4.0f] v[%+5.1f,%+5.1f] dθ=%+.3f | pos[%+6ld,%+6ld,%+6ld]\n", 
            //    (double)d1, (double)d2, (double)d3, dx_enc, dy_enc, omega_enc,
            //    (long)pos1, (long)pos2, (long)pos3);
        }

    if (++odo_data.debug_counter >= 20) { 
        odo_data.debug_counter = 0;
       
    }
    
    // Transformation encodeurs dans le repère monde
    double dx_enc_world = dx_enc * cos(this->THETA) - dy_enc * sin(this->THETA);
    double dy_enc_world = dx_enc * sin(this->THETA) + dy_enc * cos(this->THETA);
    
    // Accumulateurs pour statistiques (debugging uniquement)
    odo_data.encoder_x_acc += dx_enc_world;
    odo_data.encoder_y_acc += dy_enc_world;
    
    double dx_final_world, dy_final_world;
    
    if (optical_active && odo_data.optical_valid_count > 0) {
        // FILTRE COMPLÉMENTAIRE CONSTANT - OPTIQUE MAJORITAIRE
        
        const float ALPHA = 0.20f;  // 20% encodeurs + 80% optique (OPTIQUE ULTRA-DOMINANT)
        
        dx_final_world = ALPHA * dx_enc_world + (1.0f - ALPHA) * dx_optical_world;
        dy_final_world = ALPHA * dy_enc_world + (1.0f - ALPHA) * dy_optical_world;
        
        // Debug périodique
        static uint32_t fusion_debug = 0;
        if (++fusion_debug >= 50) {
            fusion_debug = 0;
            //printf(" FUSION CONSTANTE: α=%.2f (%.0f%% enc + %.0f%% opt) | dx=%.2fmm dy=%.2fmm\n",
            //       ALPHA, ALPHA*100, (1.0f-ALPHA)*100, dx_final_world, dy_final_world);
        }
    } else {
        // Fallback encodeurs si capteur optique inactif
        dx_final_world = dx_enc_world;
        dy_final_world = dy_enc_world;
    }
    
    // Méthode 3 : IMU BNO085  
    bool theta_updated = false;
    if (has_new_sensor_data) {
        // dtheta from teensy_capteur is already a delta in radians.
        this->THETA = normalizeAngle(this->THETA + sensor_dtheta);
        theta_updated = true;
    }

    if (!theta_updated && use_imu && bno085 && odo_data.imu_calibrated) {
        #ifdef WEBOTS_SIMULATION
            // SIMULATION : Utiliser getSensorEvent
            sh2_SensorValue_t sv;
            if (bno085->getSensorEvent(&sv) && sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
                theta_updated = true;
                float r = sv.un.gameRotationVector.real;
                float i = sv.un.gameRotationVector.i;
                float j = sv.un.gameRotationVector.j;
                float k = sv.un.gameRotationVector.k;
                
                // Conversion quaternion -> yaw
                double yaw = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
                
                static bool is_imu_first_run = true;      // Marqueur pour la 1ère fois
                static double loop_yaw_offset = 0.0;      // Pour mémoriser l'angle "tordu" de départ

                if (is_imu_first_run) {
                    loop_yaw_offset = yaw; // On capture l'angle actuel (ex: -1.26 rad) comme référence
                    is_imu_first_run = false;
                    //printf(" IMU: Re-Tare au début de boucle (Offset dynamique = %.3f rad)\n", loop_yaw_offset);
                }
                
                this->THETA = normalizeAngle(yaw - loop_yaw_offset);

                
                static uint32_t imu_debug = 0;
                if (++imu_debug >= 50) {
                    imu_debug = 0;
                    //printf(" IMU: quat[%.3f,%.3f,%.3f,%.3f] → yaw=%.3frad (%.1f°)\n",
                    //       r, i, j, k, yaw, yaw * 180.0 / M_PI);
                }
            }
        #else
            // ROBOT RÉEL : Utiliser getSensorEvent avec GAME_ROTATION_VECTOR
            sh2_SensorValue_t sv;
            if (bno085->getSensorEvent(&sv) && sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
                theta_updated = true;
                float r = sv.un.gameRotationVector.real;
                float i = sv.un.gameRotationVector.i;
                float j = sv.un.gameRotationVector.j;
                float k = sv.un.gameRotationVector.k;
                
                // Conversion quaternion -> yaw
                double yaw = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
                
                // Appliquer offset si défini
                this->THETA = yaw - odo_data.imu_yaw_offset;

            }
        #endif
    }
    
    // Fallback encodeurs si IMU non disponible
    if (!theta_updated) {
        this->THETA = normalizeAngle(this->THETA + omega_enc);
    }
    
    // Normalisation θ ∈ [-π, +π]
    while (this->THETA >  M_PI) this->THETA -= 2.0 * M_PI;
    while (this->THETA < -M_PI) this->THETA += 2.0 * M_PI;
    double theta_moyen = (last_theta_enc + this->THETA) / 2.0;
    //Intégration position X,Y
    if (optical_active) {
        this->X += dx_final_world;
        this->Y += dy_final_world;
    } else if (use_encoders) {
        //  Encodeurs en fallback uniquement
        double cos_theta = cos(theta_moyen);     
        double sin_theta = sin(theta_moyen);     
        
        this->X += dx_enc * cos_theta - dy_enc * sin_theta;
        this->Y += dx_enc * sin_theta + dy_enc * cos_theta;
        
    }
    last_theta_enc = this->THETA;
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