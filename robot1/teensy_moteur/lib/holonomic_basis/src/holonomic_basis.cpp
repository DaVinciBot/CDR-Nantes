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
    paa5100 = nullptr;
    bno085 = nullptr;  
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    // Nettoyage de la mémoire si nécessaire
    delete wheel1;
    delete wheel2;
    delete wheel3;
    delete stepperGroup;
    delete paa5100;
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
    printf("🔧 Initialisation des capteurs d'odométrie...\n");
    
    // === CAPTEUR OPTIQUE PAA5100JE ===
    #ifdef WEBOTS_SIMULATION
        paa5100 = new PAA5100();
    #else
        paa5100 = new PAA5100(PAA5100_CS_PIN);
    
        // Initialisation SPI
        pinMode(PAA5100_CS_PIN, OUTPUT);
        digitalWrite(PAA5100_CS_PIN, HIGH);
        SPI.begin();
        SPI.setClockDivider(SPI_CLOCK_DIV16);  // 1MHz
    #endif
    
        if (paa5100 && paa5100->begin()) {
            printf("✅ PAA5100 : Capteur optique initialisé\n");
        } else {
            printf("❌ PAA5100 : Échec initialisation\n");
        }
    
    
    // === IMU BNO085 ===
    #ifdef WEBOTS_SIMULATION
        bno085 = new Adafruit_BNO085();
    #else
        bno085 = new Adafruit_BNO085(BNO085_RESET_PIN);
    
        // Initialisation I2C
        Wire.begin();
        Wire.setClock(400000);  // 400kHz Fast I2C
    #endif
    
        if (bno085 && bno085->begin_I2C()) {
            printf("✅ BNO085 : IMU initialisée\n");
        
    #ifndef WEBOTS_SIMULATION
        // Activer rapports quaternions (100Hz)
        bno085->enableReport(SH2_ARVR_STABILIZED_RV, 10000);
    #endif
        
        // Calibration initiale
        delay(500);
        
        sh2_Quaternion_t quat;
        if (bno085->getQuat(quat)) {
            odo_data.imu_yaw_offset = atan2(
                2.0 * (quat.real * quat.k + quat.i * quat.j),
                1.0 - 2.0 * (quat.j * quat.j + quat.k * quat.k)
            );
            odo_data.imu_calibrated = true;
            printf("✅ IMU calibrée : offset = %.3f rad\n", odo_data.imu_yaw_offset);
        }
    } else {
        printf("❌ BNO085 : Échec initialisation\n");
    }
    
    printf("✅ Capteurs initialisés\n\n");
}
//Encore besoin ?

Point Holonomic_Basis::get_current_position() {
    Point position;
    noInterrupts(); // Section critique pour lire les doubles de manière atomique
    position.x = this->X;
    position.y = this->Y;
    position.theta = this->THETA;
    interrupts();
    return position;
}
void Holonomic_Basis::update_odometry() {
    // =========================================
    // MÉTHODE 1 : ODOMÉTRIE ENCODEURS (Roues)
    // =========================================
    
    // Récupérer positions actuelles
    int32_t pos1 = wheel1 ? wheel1->getPosition() : 0;
    int32_t pos2 = wheel2 ? wheel2->getPosition() : 0;
    int32_t pos3 = wheel3 ? wheel3->getPosition() : 0;
    
    // Calculer deltas
    int32_t delta_steps1 = pos1 - odo_data.last_pos1;
    int32_t delta_steps2 = pos2 - odo_data.last_pos2;
    int32_t delta_steps3 = pos3 - odo_data.last_pos3;
    
    // Conversion steps -> mm
    const double STEPS_TO_MM = (wheel_diameter * M_PI) / 
                               (steps_per_revolution * microsteps);
    
    double w1_mm = delta_steps1 * STEPS_TO_MM;
    double w2_mm = delta_steps2 * STEPS_TO_MM;
    double w3_mm = delta_steps3 * STEPS_TO_MM;
    
    // Cinématique Directe : Vitesses Roues -> Vitesse Robot
    // Configuration : Roue1 à 210° (-150°), Roue2 à 330° (-30°), Roue3 à 90°
    double vx_robot = (-0.333 * w1_mm - 0.333 * w2_mm + 0.667 * w3_mm);
    double vy_robot = (0.577 * w1_mm - 0.577 * w2_mm);
    double omega_enc = -(w1_mm + w2_mm + w3_mm) / (3.0 * robot_radius);
    
    // =========================================
    // MÉTHODE 2 : ODOMÉTRIE OPTIQUE (PAA5100)
    // =========================================
    
    int16_t delta_x_paa = 0, delta_y_paa = 0;
    double dx_world_paa = 0.0, dy_world_paa = 0.0;
    
    if (paa5100) {
        paa5100->readMotion(delta_x_paa, delta_y_paa);
        
        // Rotation référentiel capteur -> robot -> monde
        double cos_theta = cos(this->THETA);
        double sin_theta = sin(this->THETA);
        
        double dx_robot_paa = delta_x_paa;  // Supposé aligné avec X robot
        double dy_robot_paa = delta_y_paa;
        
        dx_world_paa = dx_robot_paa * cos_theta - dy_robot_paa * sin_theta;
        dy_world_paa = dx_robot_paa * sin_theta + dy_robot_paa * cos_theta;
    }
    
    // =========================================
    // FUSION : PAA5100 prioritaire si mouvement
    // =========================================
    
    double dx_world, dy_world;
    
    // Critère : mouvement détecté par PAA5100 (seuil 1mm)
    bool paa_reliable = (abs(delta_x_paa) > 1 || abs(delta_y_paa) > 1);
    
    if (paa_reliable && paa5100) {
        // PAA5100 prioritaire (pas de glissement)
        dx_world = dx_world_paa;
        dy_world = dy_world_paa;
    } else {
        // Encodeurs en fallback
        double cos_theta = cos(this->THETA);
        double sin_theta = sin(this->THETA);
        
        dx_world = vx_robot * cos_theta - vy_robot * sin_theta;
        dy_world = vx_robot * sin_theta + vy_robot * cos_theta;
    }
    
    // Mise à jour X, Y
    this->X += dx_world;
    this->Y += dy_world;
    
    // =========================================
    // CORRECTION THETA PAR IMU (Source fiable)
    // =========================================
    
    if (bno085 && odo_data.imu_calibrated) {
        sh2_Quaternion_t quat;
        if (bno085->getQuat(quat)) {
            // Conversion quaternion -> yaw
            double yaw = atan2(
                2.0 * (quat.real * quat.k + quat.i * quat.j),
                1.0 - 2.0 * (quat.j * quat.j + quat.k * quat.k)
            );
            
            // Appliquer offset calibration
            this->THETA = yaw - odo_data.imu_yaw_offset;
            
            // Normalisation
            while (this->THETA > M_PI) this->THETA -= 2.0 * M_PI;
            while (this->THETA < -M_PI) this->THETA += 2.0 * M_PI;
        }
    } else {
        // Fallback : intégration omega encodeurs
        this->THETA += omega_enc;
        
        while (this->THETA > M_PI) this->THETA -= 2.0 * M_PI;
        while (this->THETA < -M_PI) this->THETA += 2.0 * M_PI;
    }
    
    // Sauvegarder pour prochaine itération
    odo_data.last_pos1 = pos1;
    odo_data.last_pos2 = pos2;
    odo_data.last_pos3 = pos3;
    
    // =========================================
    // DEBUG PÉRIODIQUE
    // =========================================
    
    if (++odo_data.debug_counter >= 200) {  // 200 * 10ms = 2s
        odo_data.debug_counter = 0;
        
        printf("📊 Odo: X=%.1f Y=%.1f θ=%.3f | ENC:[%.1f,%.1f,%.1f] PAA:[%d,%d]\n",
               this->X, this->Y, this->THETA,
               w1_mm, w2_mm, w3_mm,
               delta_x_paa, delta_y_paa);
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
        //printf("📐 Erreurs: ΔX=%.1f ΔY=%.1f Δθ=%.2f\n", xerr, yerr, theta_error);
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
    double w3 = -(-1.0*vx_steps +omega_steps);

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