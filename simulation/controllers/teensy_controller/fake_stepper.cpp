#include "stepper.h" // Il va chercher votre vrai header stepper.h
#include "ArduinoFake.h" // Pour avoir accès aux types Webots
#include <cstdio>

// Calibration (A ajuster selon vos réglages Webots vs Réel)
// Si 1 tour de roue (2*PI rad) = 200 steps * 32 microsteps
#define STEPS_PER_REV 200.0
#define MICROSTEPS 32.0
const double TOTAL_STEPS = STEPS_PER_REV * MICROSTEPS; 
const double STEPS_TO_RAD = (2.0 * M_PI) / (STEPS_PER_REV * MICROSTEPS);
const double RAD_TO_STEPS = TOTAL_STEPS / (2.0 * M_PI);

Stepper::Stepper(uint8_t step_pin, uint8_t dir_pin, bool inverted) {
    char name[32];
    
    // Chercher le moteur
    sprintf(name, "motor%d", step_pin);
    WbDeviceTag m = wb_robot_get_device(name);
    
    // Chercher l'encodeur (essayer plusieurs noms)
    char encoder_name[32] = "";
    sprintf(name, "encoder%d", step_pin);
    WbDeviceTag s = wb_robot_get_device(name);
    if(s) {
        sprintf(encoder_name, "encoder%d", step_pin);
    }
    if(!s) {
        sprintf(name, "motor%d_sensor", step_pin);
        s = wb_robot_get_device(name);
        if(s) sprintf(encoder_name, "motor%d_sensor", step_pin);
    }
    if(!s) {
        sprintf(name, "position_sensor%d", step_pin);
        s = wb_robot_get_device(name);
        if(s) sprintf(encoder_name, "position_sensor%d", step_pin);
    }
    
    // Messages d'erreur ou de succès
    if(!m) {
        printf("❌ ERREUR: Moteur 'motor%d' introuvable dans Webots !\n", step_pin);
    } else if(!s) {
        printf("⚠️  Moteur 'motor%d' trouvé mais encodeur manquant\n", step_pin);
    } else {
        printf("✅ Moteur 'motor%d' initialisé (encoder: %s)\n", step_pin, encoder_name);
    }
    
    // Stockage des tags Webots
    this->m_step_pin = (uint8_t)m;
    this->m_dir_pin = (uint8_t)s;
    
    if(m) {
        wb_motor_set_position(m, INFINITY); // Mode vitesse
        wb_motor_set_velocity(m, 0.0);
    }

    if(s) {
        wb_position_sensor_enable(s, 32); // Lecture toutes les 32ms
    }
    
    m_position = 0;
    m_target = 0;
}

// Commande de vitesse (Webots gère la physique, on lui donne juste la consigne)
void Stepper::setMaxSpeed(float speed_in_steps_per_sec) { 
    WbDeviceTag m = (WbDeviceTag)this->m_step_pin;
    if (!m) return;
    // On convertit steps/s en rad/s
    double rad_s = speed_in_steps_per_sec * STEPS_TO_RAD;
    // Limitation hard (20 rad/s max dans Webots)
    if(rad_s > 20.0) rad_s = 20.0; 
    if(rad_s < -20.0) rad_s = -20.0;
    
    if(m) wb_motor_set_velocity(m, rad_s);

    // DEBUG (à retirer après test)
    //printf("Motor%d: %.1f steps/s → %.3f rad/s\n", 
           //m_step_pin, speed_in_steps_per_sec, rad_s);
}

int32_t Stepper::getPosition() const {
    WbDeviceTag s = (WbDeviceTag)this->m_dir_pin;
    if(s) {
        double rad = wb_position_sensor_get_value(s);
        return (int32_t)(rad * RAD_TO_STEPS);
    }
    return 0;
}

// Fonctions vides (Stub) car inutiles en simulation physique pure
void Stepper::setTargetAbs(int32_t target) { m_target = target; }
void Stepper::setTargetRel(int32_t d) { m_target += d; }
void Stepper::setAcceleration(float a) {} 
void Stepper::setSpeedScale(float s) {}
int32_t Stepper::getStepsToTarget() const { return 0; }
void Stepper::enable() {}
void Stepper::disable() {}
void Stepper::reset() {}
void Stepper::emergencyStop() { setMaxSpeed(0); }
bool Stepper::isRunning() const { return true; } // Toujours pret
void Stepper::compute() {} // Webots calcule tout seul
void Stepper::step() {}

// --- Implémentation du StepperGroup (Passe-Plat) ---
StepperGroup::StepperGroup(Stepper* s1, Stepper* s2, Stepper* s3) {
    steppers[0] = s1; steppers[1] = s2; steppers[2] = s3;
}
void StepperGroup::setTargets(int32_t t1, int32_t t2, int32_t t3) {} // Inutilisé en mode vitesse
void StepperGroup::setTargetsRel(int32_t d1, int32_t d2, int32_t d3) {}
void StepperGroup::startMove() {}
void StepperGroup::compute() {}
void StepperGroup::step() {}
bool StepperGroup::isMoving() const { return true; }
void StepperGroup::emergencyStop() {
    for(int i=0; i<3; i++) if(steppers[i]) steppers[i]->setMaxSpeed(0);
}