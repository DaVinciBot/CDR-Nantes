/**
 * This is the Holonomic Basis class header for STEPPER MOTORS WITH ENCODERS.
 * Combines stepper control with encoder feedback for precise odometry.
 * ADAPTED FOR 3-WHEEL HOLONOMIC BASE
 */

#pragma once

#include <Arduino.h>
#include <AccelStepper.h>
#include <pid.h>
#include "structures.h"
#include <com.h>

// Structure pour gérer un moteur stepper avec son encodeur
struct StepperWithEncoder {
    AccelStepper* stepper;
    volatile long ticks;  // Encoder ticks (accessible depuis interruptions)
    double distance;      // Distance parcourue depuis dernier handle
    byte enable_pin;
    
    StepperWithEncoder() : stepper(nullptr), ticks(0), distance(0.0), enable_pin(0) {}
};

class Holonomic_Basis {
   public:
    // PID controllers (3 for X, Y, THETA)
    PID x_pid;
    PID y_pid;
    PID theta_pid;

    // Robot geometry parameters
    inline double wheel_circumference() { return this->wheel_diameter * PI; };
    inline double wheel_unit_tick_cm() {
        return this->wheel_circumference() / this->encoder_resolution;
    };

    // Stepper motors with encoders (public pour accès depuis interruptions)
    StepperWithEncoder* wheel1;  // Front wheel (0°)
    StepperWithEncoder* wheel2;  // Back-left wheel (120°)
    StepperWithEncoder* wheel3;  // Back-right wheel (240°)

    // Odometrie - Position du robot
    double X = 0.0;
    double Y = 0.0;
    double THETA = 0.0;

    // Robot parameters
    unsigned short encoder_resolution;
    double robot_radius;      // Distance from center to wheels (cm)
    double wheel_diameter;    // Wheel diameter (cm)
    double max_speed;         // Maximum speed (steps/sec)
    double max_acceleration;  // Maximum acceleration (steps/sec²)
    unsigned short steps_per_revolution;
    unsigned short microsteps;

    // Constructor
    /**
     * @brief Constructor of the Holonomic Basis class
     */
    Holonomic_Basis(unsigned short encoder_resolution,
                    double robot_radius,
                    double wheel_diameter,
                    double max_speed,
                    double max_acceleration,
                    unsigned short steps_per_revolution,
                    unsigned short microsteps,
                    const PID& x_pid,
                    const PID& y_pid,
                    const PID& theta_pid);

    /**
     * @brief Destructor
     */
    ~Holonomic_Basis();

    // Initialization functions
    /**
     * @brief Define wheel 1 with stepper pins and encoder
     */
    void define_wheel1(byte step_pin, byte dir_pin, byte enable_pin);
    
    /**
     * @brief Define wheel 2 with stepper pins and encoder
     */
    void define_wheel2(byte step_pin, byte dir_pin, byte enable_pin);
    
    /**
     * @brief Define wheel 3 with stepper pins and encoder
     */
    void define_wheel3(byte step_pin, byte dir_pin, byte enable_pin);

    /**
     * @brief Initialize all three motors
     */
    void init_motors();

    /**
     * @brief Initialize Holonomic Basis state with starting position
     */
    void init_holonomic_basis(double x, double y, double theta);

    /**
     * @brief Enable all motors
     */
    void enable_motors();

    /**
     * @brief Disable all motors
     */
    void disable_motors();

    // Odometry and control
    /**
     * @brief Handle encoder-based odometry computation
     */
    void odometrie_handle();
    
    /**
     * @brief Handle control loop (PID + inverse kinematics)
     */
    void handle(Point target_position, Com* com);

    /**
     * @brief Update all stepper motors (must be called frequently in loop)
     */
    void run_motors();

    /**
     * @brief Get current position
     */
    Point get_current_position();

    /**
     * @brief Emergency stop
     */
    void emergency_stop();
};