/**
 * Implementation of Holonomic Basis for STEPPER MOTORS (KARIBOU MOTION VERSION)
 * Remplaçant TeensyStep4 pour un contrôle total via interruptions
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
                                 double max_speed,
                                 double max_acceleration,
                                 unsigned short steps_per_revolution,
                                 unsigned short microsteps,
                                 const PID& x_pid,
                                 const PID& y_pid,
                                 const PID& theta_pid)
    : x_pid(x_pid),
      y_pid(y_pid),
      theta_pid(theta_pid),
      robot_radius(robot_radius),
      wheel_diameter(wheel_diameter),
      max_speed(max_speed),
      max_acceleration(max_acceleration),
      steps_per_revolution(steps_per_revolution),
      microsteps(microsteps) {
    
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
    stepperGroup = nullptr;
    pmw3901 = nullptr;
    bno085 = nullptr;  


    if (wheel1) odo_data.last_pos1 = wheel1->getPosition();
    if (wheel2) odo_data.last_pos2 = wheel2->getPosition();
    if (wheel3) odo_data.last_pos3 = wheel3->getPosition();

    odo_data.optical_x_acc = 0.0;
    odo_data.optical_y_acc = 0.0;
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    // Nettoyage de la mémoire si nécessaire
    delete wheel1;
    delete wheel2;
    delete wheel3;
    delete stepperGroup;
    delete pmw3901;
    delete bno085;
}

// === DÉFINITION DES MOTEURS ===

void Holonomic_Basis::define_wheel1(byte step_pin, byte dir_pin, byte enable_pin) {
    // Création d'un Stepper KaribouMotion
    wheel1 = new Stepper(step_pin, dir_pin);
    wheel1_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}
void Holonomic_Basis::define_wheel2(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel2 = new Stepper(step_pin, dir_pin);
    wheel2_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}
void Holonomic_Basis::define_wheel3(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel3 = new Stepper(step_pin, dir_pin);
    wheel3_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

// === INITIALISATION ===
void Holonomic_Basis::init_motors() {
    // Configuration des paramètres physiques des moteurs
    if (wheel1) {
        wheel1->setMaxSpeed(max_speed);
        wheel1->setAcceleration(max_acceleration);
    }
    if (wheel2) {
        wheel2->setMaxSpeed(max_speed);
        wheel2->setAcceleration(max_acceleration);
    }
    if (wheel3) {
        wheel3->setMaxSpeed(max_speed);
        wheel3->setAcceleration(max_acceleration);
    }

    // IMPORTANT : Création du groupe de synchronisation KaribouMotion
    // C'est ici qu'on lie les 3 moteurs pour qu'ils bougent ensemble
    stepperGroup = new StepperGroup(wheel1, wheel2, wheel3);
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
    // Active LOW (classique pour les drivers) 
    digitalWrite(wheel1_enable_pin, LOW);
    digitalWrite(wheel2_enable_pin, LOW);
    digitalWrite(wheel3_enable_pin, LOW);
    
    if (wheel1) wheel1->enable();
    if (wheel2) wheel2->enable();
    if (wheel3) wheel3->enable();
}

void Holonomic_Basis::disable_motors() {
    
    digitalWrite(wheel1_enable_pin, HIGH);
    digitalWrite(wheel2_enable_pin, HIGH);
    digitalWrite(wheel3_enable_pin, HIGH);
    
    if (wheel1) wheel1->disable();
    if (wheel2) wheel2->disable();
    if (wheel3) wheel3->disable();
}

// === ODOMÉTRIE & PID ===
void Holonomic_Basis::init_sensors() {
    //printf(" Initialisation des capteurs d'odométrie...\n"); 
    // === CAPTEUR OPTIQUE PAA5100JE ===
    #ifdef WEBOTS_SIMULATION
        pmw3901 = new PAA5100();
    #else
        pmw3901 = new Bitcraze_PMW3901(PAA5100_CS_PIN);

        if (pmw3901->begin()){
            //printf(" PMW3901 : Capteur optique initialisé\n");
        } else {
            //printf(" PMW3901 : Échec initialisation\n"); //Si ca marche a supprimer
        }
    #endif
    
        if (pmw3901 && pmw3901->begin()) {
            //printf(" PAA5100 : Capteur optique initialisé\n"); //Si ca marche a supprimer
        } else {
            //printf(" PAA5100 : Échec initialisation\n"); //Si ca marche a supprimer
        }

      // === IMU BNO085 ===
    #ifdef WEBOTS_SIMULATION
        bno085 = new Adafruit_BNO08x();
        if (bno085 && bno085->begin_I2C()) {
            //printf("✅ BNO08x : IMU Mock initialisée\n");
            
            bno085->enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 100Hz
            
            odo_data.imu_calibrated = true;
            odo_data.imu_yaw_offset = 0.0; // Géré par le Mock
        } else {
            //printf("BNO08x Mock : Échec\n");
        }
    #else
        // ROBOT RÉEL
        bno085 = new Adafruit_BNO08x(BNO085_RESET_PIN);
        
        if (bno085 && bno085->begin_I2C()) {
            Wire.setClock(400000); // I2C Fast Mode
            
            bno085->enableReport(SH2_GAME_ROTATION_VECTOR, 10000); // 100Hz
            
            //printf("BNO08x : IMU réelle initialisée\n");
            
            delay(100); // Attendre stabilisation
            
            sh2_SensorValue_t sv;
            if (bno085->getSensorEvent(&sv) && sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
                float r = sv.un.gameRotationVector.real;
                float i = sv.un.gameRotationVector.i;
                float j = sv.un.gameRotationVector.j;
                float k = sv.un.gameRotationVector.k;
                
                // Calculer yaw initial (normalement proche de 0 avec Game RV)
                odo_data.imu_yaw_offset = atan2(2.0f*(r*k + i*j), 1.0f-2.0f*(j*j + k*k));
                odo_data.imu_calibrated = true;
                
                //printf(" IMU calibrée : yaw_offset = %.3f rad\n", odo_data.imu_yaw_offset);
            } else {
                //printf("  IMU : Calibration impossible, utilisation directe\n");
                odo_data.imu_yaw_offset = 0.0;
                odo_data.imu_calibrated = true;
            }
        } else {
            printf(" BNO08x : Échec initialisation I2C\n");
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
        pmw3901->readMotionCounts(&deltaX, &deltaY);
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
        bool robot_at_rest = (abs(this->last_wheel1_speed) < 1.0 && 
                             abs(this->last_wheel2_speed) < 1.0 && 
                             abs(this->last_wheel3_speed) < 1.0);
        
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
    // Récupérer positions actuelles
    int32_t pos1 = wheel1 ? wheel1->getPosition() : 0;
    int32_t pos2 = wheel2 ? wheel2->getPosition() : 0;
    int32_t pos3 = wheel3 ? wheel3->getPosition() : 0;
    
    static bool is_first_run = true;
    static double last_theta_enc =0.0;
    if (is_first_run) {
        // On synchronise juste les "last_pos" avec la réalité
        odo_data.last_pos1 = pos1;
        odo_data.last_pos2 = pos2;
        odo_data.last_pos3 = pos3;

        is_first_run = false;
        return; // On sort ! Pas de calcul de mouvement au démarrage.
    }

    // Calculer deltas
    double d1 = double(pos1 - odo_data.last_pos1);
    double d2 = double(pos2 - odo_data.last_pos2);
    double d3 = double(pos3 - odo_data.last_pos3);
    const double ENCODER_NOISE_THRESHOLD = 5.0; // steps (ajuster selon tests)
    if (abs(d1) < ENCODER_NOISE_THRESHOLD) d1 = 0.0;
    if (abs(d2) < ENCODER_NOISE_THRESHOLD) d2 = 0.0;
    if (abs(d3) < ENCODER_NOISE_THRESHOLD) d3 = 0.0;

    //  Appel du capteur optique AVANT check encodeurs (pour avoir logs même au repos)
    bool optical_active = false;
    double dx_optical_world = 0.0;
    double dy_optical_world = 0.0;
    if (use_optical_flow && pmw3901) {
        double prev_acc_x = odo_data.optical_x_acc;
        double prev_acc_y = odo_data.optical_y_acc;
        
        // omega_enc provisoire = 0 si encodeurs immobiles
        double omega_temp = 0.0;
        if (d1 != 0.0 || d2 != 0.0 || d3 != 0.0) {
            // Conversion steps -> mm temporaire pour calcul omega
            const double STEPS_TO_MM = (wheel_diameter * M_PI) / (steps_per_revolution * microsteps);
            double w1_mm = d1 * STEPS_TO_MM;
            double w2_mm = d2 * STEPS_TO_MM;
            double w3_mm = d3 * STEPS_TO_MM;
            omega_temp = (w1_mm + w2_mm + w3_mm) / (3.0 * robot_radius);
        }
        
        update_optical_odometry(omega_temp);
        
        double diff_opt_x = odo_data.optical_x_acc - prev_acc_x;
        double diff_opt_y = odo_data.optical_y_acc - prev_acc_y;
        
        // Détection rotation pure : pattern encodeurs (3 roues même sens/vitesse)
        // Rotation pure si d1 ≈ d2 ≈ d3 (toutes tournent ensemble)
        bool is_pure_rotation = false;
        if (abs(omega_temp) > 0.005) { // Rotation détectée (> 0.005 rad = 0.3°)
            // Vérifier si les 3 roues tournent dans le même sens
            double d1_abs = abs(d1);
            double d2_abs = abs(d2);
            double d3_abs = abs(d3);
            
            // Vérifier même signe (toutes positives ou toutes négatives)
            bool same_sign = (d1 * d2 > 0) && (d2 * d3 > 0);
            
            // Vérifier vitesses similaires (tolérance 20% pour imprécisions)
            double d_avg = (d1_abs + d2_abs + d3_abs) / 3.0;
            if (d_avg > 50.0) { // Mouvement significatif (>50 steps)
                double d1_diff = abs(d1_abs - d_avg) / d_avg;
                double d2_diff = abs(d2_abs - d_avg) / d_avg;
                double d3_diff = abs(d3_abs - d_avg) / d_avg;
                
                // Rotation pure = même signe ET vitesses similaires (<20% écart)
                if (same_sign && d1_diff < 0.2 && d2_diff < 0.2 && d3_diff < 0.2) {
                    is_pure_rotation = true;
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

    // Conversion steps -> mm
    const double STEPS_TO_MM = (wheel_diameter * M_PI) / 
                               (steps_per_revolution * microsteps);
    
    double w1_mm = d1 * STEPS_TO_MM;
    double w2_mm = d2 * STEPS_TO_MM;
    double w3_mm = d3 * STEPS_TO_MM;
    
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
    double dy_enc = (w1_mm - w2_mm) / 1.73205080757;  // 1.73205... = sqrt(3)

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
    if (use_imu && bno085 && odo_data.imu_calibrated) {
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
    static uint32_t fusion_debug = 0;
    if (optical_active) {
        // 🔀 FUSION ADAPTATIVE (blending encodeurs + optique pondéré)
        this->X += dx_final_world;
        this->Y += dy_final_world;
        
        if (++fusion_debug >= 200) {
            fusion_debug = 0;
            //printf(" FUSION ACTIVE: dx_final=%.3fmm dy_final=%.3fmm\n", 
            //       dx_final_world, dy_final_world);
        }
    } else if (use_encoders) {
        // 🥈 Encodeurs en fallback uniquement
        double cos_theta = cos(theta_moyen);     
        double sin_theta = sin(theta_moyen);     
        
        this->X += dx_enc * cos_theta - dy_enc * sin_theta;
        this->Y += dx_enc * sin_theta + dy_enc * cos_theta;
        
        if (++fusion_debug >= 200) {
            fusion_debug = 0;
            //printf(" FUSION: Fallback encodeurs (dx=%.3f dy=%.3f) | Optique inactif\n", 
            //       dx_enc, dy_enc);
        }
    }
    last_theta_enc = this->THETA;
    // Sauvegarder pour prochaine itération
    odo_data.last_pos1 = pos1;
    odo_data.last_pos2 = pos2;
    odo_data.last_pos3 = pos3;
    
    // =========================================
    // DEBUG PÉRIODIQUE
    // =========================================
    double dx_optical = odo_data.optical_x_acc;
    double dy_optical = odo_data.optical_y_acc;
    
    if (++odo_data.debug_counter >= 200) {  // 200 * 10ms = 2s
        odo_data.debug_counter = 0;
        
        #ifdef WEBOTS_SIMULATION
        //printf(" [WEBOTS] Odo: X=%.1f Y=%.1f θ=%.3f | ENC:[%.1f,%.1f,%.1f] GPS:[%.2f,%.2f]\n",
        //       this->X, this->Y, this->THETA,w1_mm, w2_mm, w3_mm,dx_optical, dy_optical);
        #else
        //printf(" Odo: X=%.1f Y=%.1f θ=%.3f | ENC:[%.1f,%.1f,%.1f] PAA:[%.2f,%.2f]\n",
        //       this->X, this->Y, this->THETA,w1_mm, w2_mm, w3_mm,dx_optical, dy_optical);
        #endif
    }
}

// Calcul de la boucle d'asservissement (PID + Cinématique Inverse)
void Holonomic_Basis::handle(Point target_position, Com* com) {
    // 1. Calcul des erreurs dans le référentiel Monde
    double xerr = target_position.x - this->X;
    double yerr = target_position.y - this->Y;
    double theta_error = normalizeAngle(target_position.theta - this->THETA);

    static uint32_t debug_err = 0;
    if (++debug_err > 100) {  // ~1 seconde à 100Hz
        //printf(" Erreurs: ΔX=%.1f ΔY=%.1f Δθ=%.2f\n", xerr, yerr, theta_error);
        debug_err = 0;
    }
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
    if (distance_error < 0.75 && angle_error < 0.02) {  // 1mm et ~1.15°
        vx_world = 0.0;
        vy_world = 0.0;
        omega = 0.0;

        last_wheel1_speed = 0.0;
        last_wheel2_speed = 0.0;
        last_wheel3_speed = 0.0;
        
        // Réinitialiser aussi les vitesses filtrées pour un arrêt propre
        filtered_wheel1_speed = 0.0;
        filtered_wheel2_speed = 0.0;
        filtered_wheel3_speed = 0.0;
    }
    


    // 4. Changement de repère : Monde -> Robot
    double cos_theta = cosf(this->THETA);
    double sin_theta = sinf(this->THETA);
    
    double vx_robot = cos_theta * vx_world + sin_theta * vy_world;
    double vy_robot = -sin_theta * vx_world + cos_theta * vy_world;
    
    // 5. Cinématique Inverse : Vitesse Robot -> Vitesse Roues (steps/s)
    // steps_per_m = (steps_per_rev * microsteps) / (iameter * PI)d
    double speed_factor = (steps_per_revolution * microsteps) / wheel_circumference();
    
    double vx_steps = vx_robot * speed_factor;
    double vy_steps = vy_robot * speed_factor;
    double omega_steps = omega * robot_radius * speed_factor;
    
    //Equation de mouvements
    // Roue 1 avec axe à 120° : cos(120°) = -0.5, sin(120°) = +0.866
    double w1 = -(0.5 * vx_steps - 0.866025 * vy_steps - omega_steps);
    // Roue 2 avec axe à 240° : cos(240°) = -0.5, sin(240°) = -0.866
    double w2 = -(0.5 * vx_steps +0.866025 * vy_steps -omega_steps);
    // Roue 3 avec axe à 0° : cos(0°) = +1.0, sin(0°) = 0
    double w3 = 1.0*vx_steps + omega_steps;

    // DEBUG: Affichage des vitesses calculées
    
    // Normalisation proportionnelle pour préserver la direction du mouvement
    // Si une roue dépasse MAX_SPEED, on réduit toutes les roues du même ratio
    double max_wheel_speed = fmax(fmax(fabs(w1), fabs(w2)), fabs(w3));
    if (max_wheel_speed > max_speed) {
        double scale = max_speed / max_wheel_speed;
        last_wheel1_speed = w1 * scale;
        last_wheel2_speed = w2 * scale;
        last_wheel3_speed = w3 * scale;
    } else {
        last_wheel1_speed = w1;
        last_wheel2_speed = w2;
        last_wheel3_speed = w3;
    }

    // Application du filtre passe-bas pour lisser les changements de vitesse
    // Formule : filtered = alpha * new_value + (1 - alpha) * old_value
    filtered_wheel1_speed = speed_filter_alpha * last_wheel1_speed + (1.0 - speed_filter_alpha) * filtered_wheel1_speed;
    filtered_wheel2_speed = speed_filter_alpha * last_wheel2_speed + (1.0 - speed_filter_alpha) * filtered_wheel2_speed;
    filtered_wheel3_speed = speed_filter_alpha * last_wheel3_speed + (1.0 - speed_filter_alpha) * filtered_wheel3_speed;
    
    // Normalisation proportionnelle APRÈS filtrage pour garantir le respect des limites
    // Cela préserve la direction du mouvement même après le lissage
    double max_filtered_speed = fmax(fmax(fabs(filtered_wheel1_speed), fabs(filtered_wheel2_speed)), fabs(filtered_wheel3_speed));
    if (max_filtered_speed > max_speed) {
        double scale_filtered = max_speed / max_filtered_speed;
        filtered_wheel1_speed *= scale_filtered;
        filtered_wheel2_speed *= scale_filtered;
        filtered_wheel3_speed *= scale_filtered;
    }

    static int i = 0;
    if (i++ > 100) {  // ~1 seconde
        //printf(" PID: Target[%.1f,%.1f,%.2f] Actual[%.1f,%.1f,%.2f] Err[%.1f,%.1f,%.2f]\n",
        //       target_position.x, target_position.y, target_position.theta,
        //       this->X, this->Y, this->THETA,
        //       xerr, yerr, theta_error);
        //printf(" Cmds: Vx=%.1f Vy=%.1f ω=%.2f -> W1=%.0f W2=%.0f W3=%.0f steps/s (filtered)\n",
         //      vx_world, vy_world, omega, filtered_wheel1_speed, filtered_wheel2_speed, filtered_wheel3_speed);
        i = 0;
    }

}

// === EXÉCUTION DU MOUVEMENT (KARIBOU MOTION) ===

void Holonomic_Basis::run_motors() {
    // Obsolète avec KaribouMotion géré par interruption, mais gardé pour compatibilité API
}

// Convertit les vitesses calculées (PID) en commandes de pas pour le StepperGroup
void Holonomic_Basis::execute_movement() {

    #ifdef WEBOTS_SIMULATION
        // Utilisation des vitesses filtrées pour un mouvement plus fluide
        if (wheel1) wheel1->setMaxSpeed(filtered_wheel1_speed);
        if (wheel2) wheel2->setMaxSpeed(filtered_wheel2_speed);
        if (wheel3) wheel3->setMaxSpeed(filtered_wheel3_speed);
    #else
        // HARDWARE : Mouvement relatif par delta T
        float dt = 0.01f; // 100Hz = 10ms
        
        // Utilisation des vitesses filtrées pour un mouvement plus fluide
        int32_t steps1 = (int32_t)(filtered_wheel1_speed * dt);
        int32_t steps2 = (int32_t)(filtered_wheel2_speed * dt);
        int32_t steps3 = (int32_t)(filtered_wheel3_speed * dt);
        
        if (stepperGroup) {
            stepperGroup->setTargetsRel(steps1, steps2, steps3);
            stepperGroup->startMove();
        }
    #endif
}

// === INTERFACE TIMERS (INTERRUPTIONS) ===

// À appeler par le Timer Lent (ex: 100Hz)
// Calcule les profils de vitesse trapézoïdaux
void Holonomic_Basis::compute_steppers() {
    if (stepperGroup) {
        stepperGroup->compute();
    }
}

// À appeler par le Timer Rapide (ex: 20kHz - 50µs)
// Génère les impulsions STEP physiques
void Holonomic_Basis::step_steppers() {
    if (stepperGroup) {
        stepperGroup->step();
    }
}

// Arrêt d'urgence
void Holonomic_Basis::emergency_stop() {
    if (stepperGroup) {
        stepperGroup->emergencyStop();
    }
    // Désactivation physique par sécurité
    disable_motors();
}