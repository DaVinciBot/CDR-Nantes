#include "TeensyStep4.h"

namespace TS4 {
    
    void begin() {
        // Initialisation minimale
    }
    
    // Stepper implementation
    Stepper::Stepper(int step_pin, int dir_pin) 
        : step_pin(step_pin), dir_pin(dir_pin) {
        pinMode(step_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }
    
    void Stepper::setMaxSpeed(int speed) {
        // Implémentation temporaire
    }
    
    void Stepper::setAcceleration(int accel) {
        // Implémentation temporaire  
    }
    
    void Stepper::setTargetAbs(long target) {
        // Implémentation temporaire
    }
    
    void Stepper::setTargetRel(long target) {
        // Implémentation temporaire
    }
    
    void Stepper::setPosition(long pos) {
        // Implémentation temporaire
    }
    
    long Stepper::getPosition() {
        return 0; // Implémentation temporaire
    }
    
    void Stepper::move(long steps) {
        // Implémentation temporaire
    }
    
    void Stepper::stop() {
        // Implémentation temporaire
    }
    
    void Stepper::emergencyStop() {
        // Implémentation temporaire
        stop();
    }
    
    // StepperGroup implementation
    StepperGroup::StepperGroup() {
        // Implémentation temporaire
    }
    
    void StepperGroup::addStepper(Stepper& stepper) {
        // Implémentation temporaire
    }
    
    void StepperGroup::move() {
        // Implémentation temporaire
    }
    
    void StepperGroup::startMove() {
        // Implémentation temporaire
        move();
    }
    
    void StepperGroup::emergencyStop() {
        // Implémentation temporaire
    }
}