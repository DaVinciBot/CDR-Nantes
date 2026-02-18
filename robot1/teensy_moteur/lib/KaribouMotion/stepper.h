#pragma once

#include <Arduino.h>
#include <cmath>

/**
 * KaribouMotion Stepper - Version Synchronisée
 */
class Stepper {
public:
    Stepper(uint8_t step_pin, uint8_t dir_pin, bool inverted = false);
    
    // === API COMPATIBLE TEENSYSTEP4 ===
    void setMaxSpeed(float speed);
    void setAcceleration(float accel);
    void setTargetAbs(int32_t target);
    void setTargetRel(int32_t delta);
    int32_t getPosition() const;
    void emergencyStop();
    
    // === SYNCHRONISATION ===
    void setSpeedScale(float scale);  // Facteur de réduction vitesse (0.0 - 1.0)
    int32_t getStepsToTarget() const;
    
    // === GESTION DES MOUVEMENTS ===
    void enable();
    void disable();
    void reset();
    bool isRunning() const;
    
    // === MÉTHODES INTERNES ===
    void compute();
    void step();
    
private:
    // Pins
    uint8_t m_step_pin;
    uint8_t m_dir_pin;
    bool m_inverted_dir;
    bool m_enabled;
    
    // Position et cible
    volatile int32_t m_position;
    volatile int32_t m_target;
    
    // Vitesse et accélération
    float m_max_speed;
    float m_max_accel;
    float m_speed_scale;         // Nouveau : facteur de scaling
    volatile float m_current_speed;
    
    // État du mouvement
    volatile bool m_moving;
    volatile bool m_direction;
    
    // Timing
    volatile uint32_t m_step_interval_us;
    volatile uint32_t m_last_step_time;
    
    // Profil trapézoïdal
    volatile int32_t m_steps_to_target;
    volatile float m_decel_steps;
    
    // Pulse non-bloquant
    enum PulseState { IDLE, PULSE_HIGH };
    volatile PulseState m_pulse_state;
    volatile uint32_t m_pulse_start;
    
    // Helpers
    void setDirection(bool forward);
    void updateStepInterval();
    float computeDecelSteps(float speed) const;
    float getEffectiveMaxSpeed() const { return m_max_speed * m_speed_scale; }
    float getEffectiveMaxAccel() const { return m_max_accel * m_speed_scale; }
};

/**
 * StepperGroup - Mouvements synchronisés
 */
class StepperGroup {
public:
    StepperGroup(Stepper* s1, Stepper* s2, Stepper* s3);
    
    // === MOUVEMENTS SYNCHRONISÉS ===
    void setTargets(int32_t t1, int32_t t2, int32_t t3);  // Absolu
    void setTargetsRel(int32_t d1, int32_t d2, int32_t d3); //Relatif
    void startMove();      // Calculer les scales et lancer le mouvement
    
    // === GESTION ===
    void compute();
    void step();
    bool isMoving() const;
    void emergencyStop();
    
private:
    Stepper* steppers[3];
    
    // Calcule les facteurs de scaling pour synchronisation
    void calculateSpeedScales();
};