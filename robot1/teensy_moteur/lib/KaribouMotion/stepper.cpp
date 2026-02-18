#include "stepper.h"

namespace Settings {
    namespace Stepper {
        constexpr uint8_t PULSE_WIDTH = 14;
        constexpr float MIN_SPEED = 50.0;
        constexpr float PULL_IN = 100.0;
        constexpr uint32_t MIN_STEP_DELAY = 20;
    }
}

// ============================================================================
// STEPPER - Implémentation
// ============================================================================

Stepper::Stepper(uint8_t step_pin, uint8_t dir_pin, bool inverted)
    : m_step_pin(step_pin),
      m_dir_pin(dir_pin),
      m_inverted_dir(inverted),
      m_enabled(false),
      m_position(0),
      m_target(0),
      m_max_speed(400.0f),
      m_max_accel(200.0f),
      m_speed_scale(1.0f),
      m_current_speed(0.0f),
      m_moving(false),
      m_direction(true),
      m_step_interval_us(0),
      m_last_step_time(0),
      m_steps_to_target(0),
      m_decel_steps(0.0f),
      m_pulse_state(IDLE),
      m_pulse_start(0)
{
    pinMode(m_step_pin, OUTPUT);
    pinMode(m_dir_pin, OUTPUT);
    digitalWrite(m_step_pin, LOW);
    digitalWrite(m_dir_pin, LOW);
}

void Stepper::setMaxSpeed(float speed) {
    m_max_speed = fabs(speed);
    if (m_max_speed < Settings::Stepper::MIN_SPEED) {
        m_max_speed = Settings::Stepper::MIN_SPEED;
    }
}

void Stepper::setAcceleration(float accel) {
    m_max_accel = fabs(accel);
    if (m_max_accel < 1.0f) {
        m_max_accel = 1.0f;
    }
}

void Stepper::setSpeedScale(float scale) {
    if (scale < 0.0f) scale = 0.0f;
    if (scale > 1.0f) scale = 1.0f;
    m_speed_scale = scale;
}

void Stepper::setTargetAbs(int32_t target) {
    noInterrupts();
    m_target = target;
    m_steps_to_target = m_target - m_position;
    
    if (m_steps_to_target == 0) {
        m_moving = false;
        interrupts();
        return;
    }
    
    bool new_direction = (m_steps_to_target > 0);
    setDirection(new_direction);
    m_moving = true;
    interrupts();
}
void Stepper::setTargetRel(int32_t delta) {
    setTargetAbs(m_position + delta);
}


int32_t Stepper::getPosition() const {
    // Lecture atomique sécurisée
    noInterrupts();
    int32_t pos = m_position;
    interrupts();
    return pos;
}

int32_t Stepper::getStepsToTarget() const {
    noInterrupts();
    int32_t steps = abs(m_target - m_position);
    interrupts();
    return steps;
}

void Stepper::enable() {
    m_enabled = true;
}

void Stepper::disable() {
    m_enabled = false;
    m_moving = false;
    m_current_speed = 0.0f;
}

void Stepper::reset() {
    noInterrupts();
    m_position = 0;
    m_target = 0;
    m_current_speed = 0.0f;
    m_moving = false;
    m_steps_to_target = 0;
    m_speed_scale = 1.0f;
    interrupts();
}

void Stepper::emergencyStop() {
    noInterrupts();
    m_moving = false;
    m_current_speed = 0.0f;
    m_target = m_position;
    interrupts();
}

bool Stepper::isRunning() const {
    return m_enabled && m_moving;
}

void Stepper::setDirection(bool forward) {
    bool desired = forward;
    if (m_inverted_dir) {
        desired = !desired;
    }
    
    if (desired != m_direction) {
        digitalWriteFast(m_dir_pin, desired ? HIGH : LOW);
        m_direction = desired;
        delayMicroseconds(2);
    }
}

float Stepper::computeDecelSteps(float speed) const {
    float effective_accel = getEffectiveMaxAccel();
    if (effective_accel < 1.0f) effective_accel = 1.0f; // Éviter division par zéro
    return (speed * speed) / (2.0f * effective_accel);
}

void Stepper::updateStepInterval() {
    if (m_current_speed < Settings::Stepper::MIN_SPEED) {
        m_step_interval_us = 0;
        return;
    }
    
    m_step_interval_us = (uint32_t)(1000000.0f / m_current_speed);
    
    if (m_step_interval_us < Settings::Stepper::MIN_STEP_DELAY) {
        m_step_interval_us = Settings::Stepper::MIN_STEP_DELAY;
    }
}

// ============================================================================
// COMPUTE - Profil trapézoïdal avec speed scaling
// ============================================================================
void Stepper::compute() {
    if (!m_enabled || !m_moving) {
        return;
    }
    
    // Lecture atomique de la position
    noInterrupts();
    int32_t current_pos = m_position;
    int32_t current_target = m_target;
    interrupts();
    
    int32_t steps_to_target = current_target - current_pos;
    int32_t steps_remaining = abs(steps_to_target);
    
    if (steps_remaining == 0) {
        noInterrupts();
        m_moving = false;
        m_current_speed = 0.0f;
        m_step_interval_us = 0;
        interrupts();
        return;
    }
    
    // Vitesse et accélération effectives (avec scaling)
    float effective_max_speed = getEffectiveMaxSpeed();
    float effective_max_accel = getEffectiveMaxAccel();
    
    // Assurer des valeurs minimales
    if (effective_max_speed < Settings::Stepper::MIN_SPEED) {
        effective_max_speed = Settings::Stepper::MIN_SPEED;
    }
    if (effective_max_accel < 1.0f) {
        effective_max_accel = 1.0f;
    }
    
    // Distance de décélération
    m_decel_steps = computeDecelSteps(m_current_speed);
    
    float dt = 0.01f;  // 10ms (100Hz)
    float margin = fmax(m_decel_steps * 0.15f, 10.0f); // Marge augmentée
    
    // === PHASE 1 : ACCÉLÉRATION ===
    if ((float)steps_remaining > m_decel_steps + margin) {
        m_current_speed += effective_max_accel * dt;
        
        if (m_current_speed > effective_max_speed) {
            m_current_speed = effective_max_speed;
        }
    }
    // === PHASE 2 : DÉCÉLÉRATION ===
    else {
        // Décélération plus progressive
        m_current_speed -= effective_max_accel * dt * 0.8f;
        
        if (m_current_speed < Settings::Stepper::MIN_SPEED) {
            m_current_speed = Settings::Stepper::MIN_SPEED;
        }
    }
    
    updateStepInterval();
}

// ============================================================================
// STEP - Génération pulse non-bloquant
// ============================================================================
void Stepper::step() {
    if (!m_enabled || !m_moving) {
        return;
    }
    
    uint32_t now = micros();
    
    // Fin du pulse ?
    if (m_pulse_state == PULSE_HIGH) {
        if (now - m_pulse_start >= Settings::Stepper::PULSE_WIDTH) {
            digitalWriteFast(m_step_pin, LOW);
            m_pulse_state = IDLE;
            m_last_step_time = now;
        }
        return;
    }
    
    // Nouveau step ?
    if (m_step_interval_us == 0) return;
    
    // Gérer le débordement de micros() (toutes les ~70 minutes)
    uint32_t elapsed;
    if (now >= m_last_step_time) {
        elapsed = now - m_last_step_time;
    } else {
        // Débordement détecté
        elapsed = (UINT32_MAX - m_last_step_time) + now + 1;
    }
    
    if (elapsed >= m_step_interval_us) {
        // Générer pulse
        digitalWriteFast(m_step_pin, HIGH);
        m_pulse_state = PULSE_HIGH;
        m_pulse_start = now;
        
        // Mettre à jour position de manière atomique
        noInterrupts();
        if (m_direction == !m_inverted_dir) {
            m_position++;
        } else {
            m_position--;
        }
        interrupts();
    }
}

// ============================================================================
// STEPPERGROUP - Synchronisation
// ============================================================================

StepperGroup::StepperGroup(Stepper* s1, Stepper* s2, Stepper* s3) {
    steppers[0] = s1;
    steppers[1] = s2;
    steppers[2] = s3;
}

void StepperGroup::setTargets(int32_t t1, int32_t t2, int32_t t3) {
    if (steppers[0]) steppers[0]->setTargetAbs(t1);
    if (steppers[1]) steppers[1]->setTargetAbs(t2);
    if (steppers[2]) steppers[2]->setTargetAbs(t3);
}

void StepperGroup::setTargetsRel(int32_t d1, int32_t d2, int32_t d3) {
    // On applique le mouvement relatif à chaque moteur individuellement
    if (steppers[0]) steppers[0]->setTargetRel(d1);
    if (steppers[1]) steppers[1]->setTargetRel(d2);
    if (steppers[2]) steppers[2]->setTargetRel(d3);
}
void StepperGroup::calculateSpeedScales() {
    // Trouver le moteur avec la plus grande distance
    int32_t max_distance = 0;
    
    for (int i = 0; i < 3; i++) {
        if (steppers[i]) {
            int32_t dist = steppers[i]->getStepsToTarget();
            if (dist > max_distance) {
                max_distance = dist;
            }
        }
    }
    
    // Éviter division par zéro
    if (max_distance == 0) {
        for (int i = 0; i < 3; i++) {
            if (steppers[i]) {
                steppers[i]->setSpeedScale(1.0f);
            }
        }
        return;
    }
    
    // Calculer les ratios de vitesse
    for (int i = 0; i < 3; i++) {
        if (steppers[i]) {
            int32_t dist = steppers[i]->getStepsToTarget();
            
            if (dist == 0) {
                // Moteur immobile
                steppers[i]->setSpeedScale(0.0f);
            } else {
                float scale = (float)dist / (float)max_distance;
                
                // Minimum 15% pour éviter vitesses trop faibles
                if (scale < 0.15f) {
                    scale = 0.15f;
                }
                
                steppers[i]->setSpeedScale(scale);
            }
        }
    }
}

void StepperGroup::startMove() {
    // Calculer les scaling factors
    calculateSpeedScales();
    
    // Les moteurs sont déjà configurés avec setTargets()
    // Les profils trapézoïdaux vont se synchroniser automatiquement
}

void StepperGroup::compute() {
    for (int i = 0; i < 3; i++) {
        if (steppers[i]) {
            steppers[i]->compute();
        }
    }
}

void StepperGroup::step() {
    for (int i = 0; i < 3; i++) {
        if (steppers[i]) {
            steppers[i]->step();
        }
    }
}

bool StepperGroup::isMoving() const {
    for (int i = 0; i < 3; i++) {
        if (steppers[i] && steppers[i]->isRunning()) {
            return true;
        }
    }
    return false;
}

void StepperGroup::emergencyStop() {
    for (int i = 0; i < 3; i++) {
        if (steppers[i]) {
            steppers[i]->emergencyStop();
        }
    }
}


