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
            // On cast pour accéder à la méthode reset() du Mock
            ((PAA5100*)pmw3901)->reset(); 
        #endif
        // (Sur le vrai robot Bitcraze, il n'y a pas de reset soft nécessaire, 
        // car le vrai capteur donne des deltas relatifs matériels)
    }
}

// === GESTION ÉTAT MOTEURS ===

void Holonomic_Basis::enable_motors() {
    // Active LOW (classique pour les drivers type TB6600/DM542/A4988)
    digitalWrite(wheel1_enable_pin, LOW);
    digitalWrite(wheel2_enable_pin, LOW);
    digitalWrite(wheel3_enable_pin, LOW);
    
    // Active la logique logicielle
    if (wheel1) wheel1->enable();
    if (wheel2) wheel2->enable();
    if (wheel3) wheel3->enable();
}

void Holonomic_Basis::disable_motors() {
    // Désactive les drivers
    digitalWrite(wheel1_enable_pin, HIGH);
    digitalWrite(wheel2_enable_pin, HIGH);
    digitalWrite(wheel3_enable_pin, HIGH);
    
    // Désactive la logique logicielle
    if (wheel1) wheel1->disable();
    if (wheel2) wheel2->disable();
    if (wheel3) wheel3->disable();
}

// === ODOMÉTRIE & PID ===
void Holonomic_Basis::init_sensors() {
    printf("🔧 Initialisation des capteurs d'odométrie...\n"); //Si ca marche a supprimer
    bool pmw_initialized = false;
    // === CAPTEUR OPTIQUE PAA5100JE ===
    #ifdef WEBOTS_SIMULATION
        pmw3901 = new PAA5100();
        pmw_initialized = true; // On garde le Mock en simu
    #else
        pmw3901 = new Bitcraze_PMW3901(PAA5100_CS_PIN);

        if (pmw3901->begin()){
            printf("✅ PMW3901 : Capteur optique initialisé\n");
             pmw_initialized = true; //Si ca marche a supprimer
        } else {
            printf("❌ PMW3901 : Échec initialisation\n"); //Si ca marche a supprimer
        }
    #endif
    
        if (pmw3901 && pmw3901->begin()) {
            printf("✅ PAA5100 : Capteur optique initialisé\n"); //Si ca marche a supprimer
            pmw_initialized = true; //Si ca marche a supprimer
        } else {
            printf("❌ PAA5100 : Échec initialisation\n"); //Si ca marche a supprimer
        }
    
    
    // === IMU BNO085 ===
    #ifdef WEBOTS_SIMULATION
        bno085 = new Adafruit_BNO085();
        printf("BNO085 : IMU initialisée (Mock Webots)\n"); //Si ca marche a supprimer
    #else
        bno085 = new Adafruit_BNO085x(BNO085_RESET_PIN);
        
        if(bno085->begin_I2C()){
            Wire.setClock(400000); // Mettre I2C à 400kHz
            bno085->enableReport(SH2_ARVR_STABILIZED_RV, 10000); 
            printf("✅ BNO085 : IMU initialisée\n"); //Si ca marche a supprimer
        } else {
            printf("❌ BNO085 : Échec initialisation\n"); //Si ca marche a supprimer
        }
        
    #endif
    
        if (bno085 && bno085->begin_I2C()) {
            printf("✅ BNO085 : IMU initialisée\n"); //Si ca marche a supprimer
        } else {
            printf("❌ BNO085 : Échec initialisation\n"); //Si ca marche a supprimer
        }
    
    delay(100);
    if (bno085){
        #ifdef WEBOTS_SIMULATION
            odo_data.imu_yaw_offset = 0.0; odo_data.imu_calibrated = true;
            printf("BNO085 : IMU initialisée (Mock Webots)\n"); //Si ca marche a supprimer
        #else
            sh2_SensorValue_t sv;
            if(bno085->getSensorEvent(&sv) && sv.sensorId == SH2_ARVR_STABILIZED_RV) {
                float r = sv.un.arvrStabilizedRV.real;
                float i = sv.un.arvrStabilizedRV.i;
                float j = sv.un.arvrStabilizedRV.j;
                float k = sv.un.arvrStabilizedRV.k;
                odo_data.imu_yaw_offset = atan2(2.0f*(r*k + i*j), 1.0f-2.0f*(j*j + k*k));
                odo_data.imu_calibrated = true;
            }
        #endif
    }

    
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
    int16_t raw_x = 0, raw_y = 0;
    
    // 1. Lecture Capteur
    #ifdef WEBOTS_SIMULATION
        pmw3901->readMotion(deltaX, deltaY); // Mock retourne mm
        double dx_mm = deltaX; 
        double dy_mm = deltaY;
        raw_x = deltaX; raw_y = deltaY;
    #else 
        pmw3901->readMotionCounts(&deltaX, &deltaY);
        double dx_mm = deltaX * OPTICAL_SCALE; 
        double dy_mm = deltaY * OPTICAL_SCALE;
    #endif
    
    // 2. [NOUVEAU] Flag de démarrage (Anti-Saut)
    // Ignore la toute première lecture qui est souvent aberrante (ex: 84km)
    static bool is_first_run_opt = true;
    if (is_first_run_opt) {
        is_first_run_opt = false;
        return; 
    }

    // 3. Calculs Géométriques (Deltas)
    // On fait les calculs MAINTENANT pour pouvoir les afficher dans le debug
    
    // Rotation Capteur -> Robot
    double c_mnt = cos(OPTICAL_MOUNT_ANGLE);
    double s_mnt = sin(OPTICAL_MOUNT_ANGLE);
    double dx_robot = dx_mm * c_mnt - dy_mm * s_mnt;
    double dy_robot = dx_mm * s_mnt + dy_mm * c_mnt;
    
    // Compensation levier (Rotation robot créant faux mouvement)
    dx_robot -= -OPTICAL_OFFSET_Y * dtheta_robot;
    dy_robot -=  OPTICAL_OFFSET_X * dtheta_robot;

    // Projection Delta dans le Monde (Juste pour l'info debug pour l'instant)
    double c = cos(this->THETA);
    double s = sin(this->THETA);    
    double dx_world = dx_robot * c - dy_robot * s;
    double dy_world = dx_robot * s + dy_robot * c;

    // 4. [CORRECTIF] Debug Périodique (Placé AVANT le filtre de bruit)
    // Cela garantit que le printf tourne même si le robot est à l'arrêt
    static uint32_t debug_cnt = 0;
    if (++debug_cnt >= 20) {
        debug_cnt = 0;
        #ifdef WEBOTS_SIMULATION
        printf("📷 GPS: raw=[%4d,%4d]mm | robot=[%6.2f,%6.2f]mm | world=[%6.2f,%6.2f]mm | pos=[%7.1f,%7.1f]mm\n",
               raw_x, raw_y,
               dx_robot, dy_robot,
               dx_world, dy_world,
               odo_data.optical_x_acc, odo_data.optical_y_acc);
        #else
        // Pour le réel, on ajoute la qualité de surface (Squal)
        // (Assurez-vous que votre librairie Bitcraze supporte readMotionCount avec Squal, sinon simplifiez)
        printf("📷 PAA: raw=[%d,%d] | world=[%.2f,%.2f] | pos=[%.1f,%.1f]\n",
               deltaX, deltaY,
               dx_world, dy_world,
               odo_data.optical_x_acc, odo_data.optical_y_acc);
        #endif
    }

    // 5. Filtre de bruit (Seuil d'activation)
    // Si le mouvement est insignifiant, on quitte ICI (on n'intègre pas)
    if(abs(dx_mm) < 0.01 && abs(dy_mm) < 0.01) {
        return; 
    }

    // 6. Intégration (Seulement si mouvement réel)
    odo_data.optical_x_acc += dx_world;
    odo_data.optical_y_acc += dy_world;
}

/**
 * CORRECTION update_odometry() - Debug conditionnel
 * À remplacer dans holonomic_basis.cpp
 */

void Holonomic_Basis::update_odometry() {
    // Récupérer positions actuelles
    int32_t pos1 = wheel1 ? wheel1->getPosition() : 0;
    int32_t pos2 = wheel2 ? wheel2->getPosition() : 0;
    int32_t pos3 = wheel3 ? wheel3->getPosition() : 0;
    
    static bool is_first_run = true;
    
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
    
    // Conversion steps -> mm
    const double STEPS_TO_MM = (wheel_diameter * M_PI) / 
                               (steps_per_revolution * microsteps);
    
    double w1_mm = d1 * STEPS_TO_MM;
    double w2_mm = d2 * STEPS_TO_MM;
    double w3_mm = d3 * STEPS_TO_MM;
    
    // Cinématique Directe : Vitesses Roues -> Vitesse Robot
    // Configuration : Roue1 à 210° (-150°), Roue2 à 330° (-30°), Roue3 à 90°
    // Vitesse X = (2*W3 - W1 - W2) / 3
    double dx_enc = (2.0 * w3_mm - w1_mm - w2_mm) / 3.0;

    // Vitesse Y = (W1 - W2) / sqrt(3)
    double dy_enc = (w1_mm - w2_mm) / 1.73205;

    // Rotation = -(W1 + W2 + W3) / (3 * Rayon)
    double omega_enc = -(w1_mm + w2_mm + w3_mm) / (3.0 * robot_radius);
    
    
    static uint32_t enc_debug_counter = 0;
        if (++enc_debug_counter >= 20) {
            enc_debug_counter = 0;
            printf("📏 ENC: d[%+4.0f,%+4.0f,%+4.0f] v[%+5.1f,%+5.1f] dθ=%+.3f\n", 
                   (double)d1, (double)d2, (double)d3, 
                   dx_enc, dy_enc, omega_enc);
        }

    if (++odo_data.debug_counter >= 20) { 
        odo_data.debug_counter = 0;
        
        // On affiche :
        // 1. Les variations encodeurs brutes (d1, d2, d3)
        // 2. Le déplacement calculé en Y (dy_enc)
        // 3. La position Y globale
        printf("DEBUG: d1=%.1f d2=%.1f d3=%.1f  ->  dx_enc=%.4f dy_enc=%.4f omega_enc=%.4f ->  X_Global=%.2f Y_Global=%.2f\n Theta_Global=%.3f\n", 
               d1, d2, d3, dx_enc, dy_enc, omega_enc, this->X, this->Y, this->THETA);
    }
    
    // MÉTHODE 2 : ODOMÉTRIE OPTIQUE (PAA5100)
    bool optical_active=false;
    double dx_final_world=0.0;
    double dy_final_world=0.0;
    if (use_optical_flow && pmw3901) {
        double prev_acc_x = odo_data.optical_x_acc;
        double prev_acc_y = odo_data.optical_y_acc;

        // Mise à jour de l'accumulateur optique (avec correction centrifuge dtheta_enc)
        update_optical_odometry(omega_enc);
        
        // Calcul du delta perçu ce cycle
        double diff_opt_x = odo_data.optical_x_acc - prev_acc_x;
        double diff_opt_y = odo_data.optical_y_acc - prev_acc_y;

        // Si l'optique a bougé significativement, on l'utilise
        if (abs(diff_opt_x) > 0.001 || abs(diff_opt_y) > 0.001) {
            dx_final_world = diff_opt_x;
            dy_final_world = diff_opt_y;
            optical_active = true;
        }
    }
    // Méthode 3 : IMU BNO085
    bool imu_used = false;
    if (use_imu && bno085 && odo_data.imu_calibrated) {
        #ifdef WEBOTS_SIMULATION
            sh2_Quaternion_t quat;
            if (bno085->getQuat(quat)) {
                // Conversion quaternion -> yaw (unused)
                imu_used = true;
            }
        #else
            sh2_SensorValue_t sv;
            if (bno085->getSensorEvent(&sv) && sv.sensorId == SH2_ARVR_STABILIZED_RV) {
                float r = sv.un.arvrStabilizedRV.real;
                float i = sv.un.arvrStabilizedRV.i;
                float j = sv.un.arvrStabilizedRV.j;
                float k = sv.un.arvrStabilizedRV.k;
                double yaw = atan2(2.0f * (r * k + i * j), 1.0f - 2.0f * (j * j + k * k));
                imu_used = true;
            }
        #endif
        }
    if (imu_used) {
        this->THETA = normalizeAngle(this->THETA +omega_enc);
        // Fallback : intégration omega encodeurs
        
    }
    //Intégration position X,Y
    if (optical_active) {
        // PAA5100 prioritaire (pas de glissement)
        this->X+= dx_final_world;
        this->Y += dy_final_world;
    } else if (use_encoders) {
        // Encodeurs en fallback
        double cos_theta = cos(this->THETA);
        double sin_theta = sin(this->THETA);
        
        this->X += dx_enc * cos_theta - dy_enc * sin_theta;
        this->Y += dx_enc * sin_theta + dy_enc * cos_theta;
    }
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
        printf("🎮 [WEBOTS] Odo: X=%.1f Y=%.1f θ=%.3f | ENC:[%.1f,%.1f,%.1f] GPS:[%.2f,%.2f]\n",
               this->X, this->Y, this->THETA,
               w1_mm, w2_mm, w3_mm,
               dx_optical, dy_optical);
        #else
        printf("📊 Odo: X=%.1f Y=%.1f θ=%.3f | ENC:[%.1f,%.1f,%.1f] PAA:[%.2f,%.2f]\n",
               this->X, this->Y, this->THETA,
               w1_mm, w2_mm, w3_mm,
               dx_optical, dy_optical);
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
    if (++debug_err > 1000) {
        printf("📐 Erreurs: ΔX=%.1f ΔY=%.1f Δθ=%.2f\n", xerr, yerr, theta_error);
        debug_err = 0;
    }
    // 2. Calcul des vitesses cibles via PID (référentiel Monde)
    double vx_world, vy_world, omega;
    if (!use_pid_control) {
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
    
    if (distance_error < 1 && angle_error < 0.05) {  // 5mm et 3°
        vx_world = 0.0;
        vy_world = 0.0;
        omega = 0.0;
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
    double w1 = -(0.5 * vx_steps - 0.866025 * vy_steps + omega_steps);
    // Roue 2 avec axe à 240° : cos(240°) = -0.5, sin(240°) = -0.866
    double w2 = -(0.5 * vx_steps +0.866025 * vy_steps + omega_steps);
    // Roue 3 avec axe à 0° : cos(0°) = +1.0, sin(0°) = 0
    double w3 = 1.0*vx_steps - omega_steps;

    // DEBUG: Affichage des vitesses calculées
    
    // Stockage des vitesses cibles (bornées par max_speed)
    last_wheel1_speed = constrain(w1, -max_speed, max_speed);
    last_wheel2_speed = constrain(w2, -max_speed, max_speed);
    last_wheel3_speed = constrain(w3, -max_speed, max_speed);
}

// === EXÉCUTION DU MOUVEMENT (KARIBOU MOTION) ===

void Holonomic_Basis::run_motors() {
    // Obsolète avec KaribouMotion géré par interruption, mais gardé pour compatibilité API
}

// Convertit les vitesses calculées (PID) en commandes de pas pour le StepperGroup
void Holonomic_Basis::execute_movement() {

    if (wheel1) wheel1->setMaxSpeed(last_wheel1_speed);
    if (wheel2) wheel2->setMaxSpeed(last_wheel2_speed);
    if (wheel3) wheel3->setMaxSpeed(last_wheel3_speed);
    
    // DEBUG CRITIQUE : Voir ce qui est envoyé
    static uint32_t debug_counter = 0;
    if(++debug_counter > 1000) {  // Toutes les ~3 secondes
        printf("🎮 Wheel speeds: W1=%.1f W2=%.1f W3=%.1f steps/s\n", 
               last_wheel1_speed, last_wheel2_speed, last_wheel3_speed);
        debug_counter = 0;
    }
    // Période d'exécution (doit correspondre au Timer qui appelle cette fonction)
    // Ici on suppose 100Hz (10ms) comme le PID
    //float dt = 0.01f; 
    
    // Calcul du nombre de pas à effectuer durant ce delta T
    // Vitesse (steps/s) * Temps (s) = Distance (steps)
    //int32_t steps1 = (int32_t)(last_wheel1_speed * dt);
    //int32_t steps2 = (int32_t)(last_wheel2_speed * dt);
    //int32_t steps3 = (int32_t)(last_wheel3_speed * dt);
    
    // Envoi des cibles RELATIVES au groupe
    // "Avance de X pas par rapport à maintenant"
    //if (stepperGroup) {
        //stepperGroup->setTargetsRel(steps1, steps2, steps3);
        
        // Lance le calcul de synchronisation
       // stepperGroup->startMove();
    //}
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