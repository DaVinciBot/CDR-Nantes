#pragma once

// TeensyStep4.h minimal pour compilation temporaire
#include <Arduino.h>

namespace TS4 {
    void begin();
    
    class Stepper {
    public:
        Stepper(int step_pin, int dir_pin);
        void setMaxSpeed(int speed);
        void setAcceleration(int accel);
        void setTargetAbs(long target);
        void setTargetRel(long target);
        void setPosition(long pos);
        long getPosition();
        void move(long steps);
        void stop();
        void emergencyStop(); // Ajouté
        
    private:
        int step_pin, dir_pin;
    };
    
    class StepperGroup {
    public:
        StepperGroup();
        void addStepper(Stepper& stepper);
        void move();
        void startMove(); // Ajouté
        void emergencyStop();
        
    private:
        // Implémentation minimale
    };
}

using namespace TS4;