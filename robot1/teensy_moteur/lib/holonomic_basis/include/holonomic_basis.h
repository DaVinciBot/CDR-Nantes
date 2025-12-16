#pragma once

#include <Arduino.h>

// IMPORTANT : On inclut notre librairie "Maison" au lieu de TeensyStep4
// Le chemin relatif dépend de l'arborescence, ici on remonte vers KaribouMotion
#include "../../KaribouMotion/stepper.h"

#include <pid.h>
#include "structures.h"

class Com;

class Holonomic_Basis {
   public:
    // PID controllers (3 for X, Y, THETA)
    PID x_pid;
    PID y_pid;
    PID theta_pid;

    // Robot geometry parameters
    inline double wheel_circumference() { return this->wheel_diameter * PI; };

    // Stepper motors (KaribouMotion)
    // On utilise nos propres classes Stepper maintenant
    Stepper* wheel1;  // Front wheel (0°)
    Stepper* wheel2;  // Back-left wheel (120°)
    Stepper* wheel3;  // Back-right wheel (240°)
    
    // Le groupe de synchronisation KaribouMotion
    StepperGroup* stepperGroup;

    // Odometrie - Position du robot
    double X = 0.0;
    double Y = 0.0;
    double THETA = 0.0;

    // Robot parameters
    double robot_radius;      // Distance from center to wheels (mm)
    double wheel_diameter;    // Wheel diameter (mm)
    double max_speed;         // Maximum speed (steps/sec)
    double max_acceleration;  // Maximum acceleration (steps/sec²)
    unsigned short steps_per_revolution;
    unsigned short microsteps;

    // Variables pour stocker les vitesses calculées par le PID
    double last_wheel1_speed = 0.0;
    double last_wheel2_speed = 0.0;
    double last_wheel3_speed = 0.0;

    // Constructor
    Holonomic_Basis(double robot_radius,
                    double wheel_diameter,
                    double max_speed,
                    double max_acceleration,
                    unsigned short steps_per_revolution,
                    unsigned short microsteps,
                    const PID& x_pid,
                    const PID& y_pid,
                    const PID& theta_pid);

    ~Holonomic_Basis();

    // Initialization functions
    void define_wheel1(byte step_pin, byte dir_pin, byte enable_pin);
    void define_wheel2(byte step_pin, byte dir_pin, byte enable_pin);
    void define_wheel3(byte step_pin, byte dir_pin, byte enable_pin);

    void init_motors();
    void init_holonomic_basis(double x, double y, double theta);

    void enable_motors();
    void disable_motors();

    // Control
    void handle(Point target_position, Com* com);

    // Mouvement
    void run_motors();        // Obsolète (gardé pour compatibilité)
    void execute_movement();  // Convertit vitesse -> pas relatifs
    
    // === NOUVELLES MÉTHODES (Celles qui manquaient) ===
    void compute_steppers();  // Calcul des profils (Timer Lent)
    void step_steppers();     // Génération des pas (Timer Rapide)
    // ==================================================

    Point get_current_position();
    void emergency_stop();

   private:
    byte wheel1_enable_pin;
    byte wheel2_enable_pin;
    byte wheel3_enable_pin;
};