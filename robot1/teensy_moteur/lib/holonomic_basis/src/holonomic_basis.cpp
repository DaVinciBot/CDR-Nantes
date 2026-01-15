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
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    // Nettoyage de la mémoire si nécessaire
    delete wheel1;
    delete wheel2;
    delete wheel3;
    delete stepperGroup;
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

Point Holonomic_Basis::get_current_position() {
    Point position;
    noInterrupts(); // Section critique pour lire les doubles de manière atomique
    position.x = this->X;
    position.y = this->Y;
    position.theta = this->THETA;
    interrupts();
    return position;
}

// Calcul de la boucle d'asservissement (PID + Cinématique Inverse)
void Holonomic_Basis::handle(Point target_position, Com* com) {
    static bool first_call = true;
    if (first_call) {
        printf("🔍 PREMIÈRE ITÉRATION handle()\n");
        printf("   Position actuelle: X=%.1f Y=%.1f θ=%.2f\n", X, Y, THETA);
        printf("   Position cible   : X=%.1f Y=%.1f θ=%.2f\n", 
               target_position.x, target_position.y, target_position.theta);
        first_call = false;
    }
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
    static uint32_t debug_vel = 0;
    if (++debug_vel > 1000) {
        printf("🚀 Vitesses: vx=%.1f vy=%.1f ω=%.2f\n", vx_world, vy_world, omega);
        debug_vel = 0;
    }

    double distance_error = sqrt(xerr*xerr + yerr*yerr);
    double angle_error = fabs(theta_error);
    
    if (distance_error < 0.01 && angle_error < 0.05) {  // 5mm et 3°
        vx_world = 0.0;
        vy_world = 0.0;
        omega = 0.0;
    }
    // 3. Mise à jour du Dead Reckoning (Odométrie théorique)
    // On suppose que le robot va réaliser exactement la vitesse demandée
    // dt = 0.01s (correspond à la fréquence d'appel de handle, ex: 100Hz)
    float dt = 0.01f; 
    this->X += vx_world * dt;
    this->Y += vy_world * dt;
    this->THETA = normalizeAngle(this->THETA + omega * dt);


    // 4. Changement de repère : Monde -> Robot
    double cos_theta = cosf(this->THETA);
    double sin_theta = sinf(this->THETA);
    
    double vx_robot = cos_theta * vx_world + sin_theta * vy_world;
    double vy_robot = -sin_theta * vx_world + cos_theta * vy_world;
    
    // 5. Cinématique Inverse : Vitesse Robot -> Vitesse Roues (steps/s)
    // steps_per_m = (steps_per_rev * microsteps) / (diameter * PI)
    double speed_factor = (steps_per_revolution * microsteps) / wheel_circumference();
    
    double vx_steps = vx_robot * speed_factor;
    double vy_steps = vy_robot * speed_factor;
    double omega_steps = omega * robot_radius * speed_factor;
    
    // Matrice de projection pour robot 3 roues (0°, 120°, 240°)
    // Wheel 1 (Front)
    double w3 = -0.5 * vx_steps - (sqrt(3.0) / 2.0) * vy_steps + omega_steps;
    // Wheel 2 (Back-Left)
    double w2 = 0.5 * vx_steps + (sqrt(3.0) / 2.0) * vy_steps + omega_steps;
    // Wheel 3 (Back-Right)
    double w1 = vx_steps + omega_steps;
    
    
    //Cinématique Inverser Vitesse Robot -> Vitesse Roues (formule corrigée)
    // Angles des roues en radians avec 30;90;150 degrés décalage
    //double alpha_A = -60.0 * M_PI / 180.0;  
    //double alpha_B = +60.0 * M_PI / 180.0;  
    //double alpha_C = 180.0 * M_PI / 180.0;  

    //double w1 = vx_steps * cos(alpha_A) + vy_steps * sin(alpha_A) + omega_steps;
    //double w2 = vx_steps * cos(alpha_B) + vy_steps * sin(alpha_B) + omega_steps;
    //double w3 = vx_steps * cos(alpha_C) + vy_steps * sin(alpha_C) + omega_steps;



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