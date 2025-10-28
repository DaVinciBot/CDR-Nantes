/**
 * This is the Holonomic Basis class header.
 * The Holonomic Basis class is the core of the Motor teensy code.
 * It provides method to control the 3 motors, the speed and the orientation.
 * It computes the odometry and correct the motors error with the PID class.
 * ADAPTED FOR 3-WHEEL HOLONOMIC BASE (120° between wheels)
 */

#pragma once

#include <Arduino.h>
#include <motors_driver.h>
#include <pid.h>
#include "structures.h"
#include <com.h>  // Communication object to manage the communication between the teensy and the Raspberry Pi

class Holonomic_Basis {
   public:
    // PID controllers (3 instead of 2)
    PID x_pid;
    PID y_pid;
    PID theta_pid;

    // Holonomic basis's params
    inline double wheel_perimeter() { return this->wheel_diameter * PI; };
    inline double wheel_unit_tick_cm() {
        return this->wheel_perimeter() / this->encoder_resolution;
    };

    // Properties
    /**
     * @brief Get current position (X, Y and THETA) of the robot
     *
     * @return Current position
     */
    Point get_current_position();

    // Holonomic basis's motors (3 wheels)
    Motor* wheel1;  // Front wheel (0°)
    Motor* wheel2;  // Back-left wheel (120°)
    Motor* wheel3;  // Back-right wheel (240°)

    // Odometrie
    double X = 0.0f;
    double Y = 0.0f;
    double THETA = 0.0f;

    // Holonomic basis params
    unsigned short encoder_resolution;
    double robot_radius;      // Distance from center to wheels
    double wheel_diameter;

    // Constructor
    /**
     * @brief constructor of the Holonomic Basis class
     *
     * Initializes the parameters of the Holonomic Basis
     */
    Holonomic_Basis(unsigned short encoder_resolution,
                    double robot_radius,
                    double wheel_diameter,
                    const PID& x_pid,
                    const PID& y_pid,
                    const PID& theta_pid);

    /**
     * @brief Destructor of Holonomic Basis class
     */
    ~Holonomic_Basis() = default;

    // Inits function
    /**
     * @brief Define wheel 1 (front - 0°) with pins, related encoders pin and properties
     * of the wheel attached to the motor.
     */
    void define_wheel1(byte enca,
                       byte encb,
                       byte pwm,
                       byte in2,
                       byte in1,
                       byte max_pwm);
    /**
     * @brief Define wheel 2 (back-left - 120°) with pins, related encoders pin and properties
     * of the wheel attached to the motor.
     */
    void define_wheel2(byte enca,
                       byte encb,
                       byte pwm,
                       byte in2,
                       byte in1,
                       byte max_pwm);
    /**
     * @brief Define wheel 3 (back-right - 240°) with pins, related encoders pin and properties
     * of the wheel attached to the motor.
     */
    void define_wheel3(byte enca,
                       byte encb,
                       byte pwm,
                       byte in2,
                       byte in1,
                       byte max_pwm);
    /**
     * @brief Initialize all three motors
     */
    void init_motors();
    /**
     * @brief Initialize Holonomic Basis state with starting position
     */
    void init_holonomic_basis(double x, double y, double theta);

    // Odometrie function
    /**
     * @brief Handle the odometry computation for holonomic base
     *
     * Update motors position then compute displacement of all three motors.
     * With these results, estimate the robot position and orientation using
     * forward kinematics for holonomic base.
     */
    void odometrie_handle();
    
    /**
     * @brief Handle the correction computation for holonomic base
     *
     * Compute the X, Y and theta error in terms of position.
     * Compute the 3 PIDs and set the motors new command using inverse kinematics.
     */
    void handle(Point target_position, Com* com);
};