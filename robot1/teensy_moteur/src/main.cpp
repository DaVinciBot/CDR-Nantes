#pragma once
// External libraries used: Arduino, TimerOne, ATOMIC
#include <Arduino.h>      // Arduino framework
#include <TimerOne.h>     // Timer interrupt library
#include <util/atomic.h>  // Atomic block library

// Custom libraries
#include <holonomic_basis.h>  // Holonomic Basis with steppers
#include <config.h>           // Configuration file

// 1. Instantiate the 3 PID controllers (X, Y, THETA)
PID x_pid(KP_X,
          KI_X,
          KD_X,
          -MAX_PWM,
          MAX_PWM,
          5.0);

PID y_pid(KP_Y,
          KI_Y,
          KD_Y,
          -MAX_PWM,
          MAX_PWM,
          5.0);

PID theta_pid(KP_THETA,
              KI_THETA,
              KD_THETA,
              -MAX_PWM,
              MAX_PWM,
              2.0);

// 2. Instantiate the Holonomic Basis object
Holonomic_Basis* holonomic_basis_ptr = new Holonomic_Basis(
    ROBOT_RADIUS,
    WHEEL_DIAMETER,
    MAX_SPEED,
    MAX_ACCELERATION,
    STEPS_PER_REVOLUTION,
    MICROSTEPS,
    x_pid,
    y_pid,
    theta_pid
);

// 3. Instantiate the Communication object
Com* com;

// 4. Define global variables for callback functions
Point target_position(START_X, START_Y, START_THETA);

// 5. Define callback functions for communication
void set_target_position(byte* msg, byte size) {
    msg_set_target_position* target_position_msg =
        (msg_set_target_position*)msg;

    // Update position
    target_position.x = target_position_msg->target_position_x;
    target_position.y = target_position_msg->target_position_y;
    target_position.theta = target_position_msg->target_position_theta;
}

void set_pid(byte* msg, byte size) {
    msg_set_pid* pid_msg = (msg_set_pid*)msg;
    PID* pid = nullptr;
    bool is_valid_pid = true;
    
    switch (pid_msg->pid_type) {
        case X_PID_ID:
            pid = &holonomic_basis_ptr->x_pid;
            break;
        case Y_PID_ID:
            pid = &holonomic_basis_ptr->y_pid;
            break;
        case THETA_PID_ID:
            pid = &holonomic_basis_ptr->theta_pid;
            break;
        default:
            is_valid_pid = false;
            break;
    }
    
    if (is_valid_pid) {
        pid->updateParameters(pid_msg->kp, pid_msg->ki, pid_msg->kd);
    }
}

void set_odometrie(byte* msg, byte size) {
    msg_set_odometrie* odometrie = (msg_set_odometrie*)msg;

    holonomic_basis_ptr->X = odometrie->x;
    holonomic_basis_ptr->Y = odometrie->y;
    holonomic_basis_ptr->THETA = odometrie->theta;

    // Update target position: avoid the usage of old stored target point
    target_position.x = odometrie->x;
    target_position.y = odometrie->y;
    target_position.theta = odometrie->theta;
}

void reset_teensy(byte* msg, byte size) {
    // Reset the teensy
    void (*reboot)(void) = 0;
    reboot();
}

// 6. Assign callback functions to message IDs
void (*callback_functions[256])(byte* msg, byte size);

void initialize_callback_functions() {
    callback_functions[SET_TARGET_POSITION] = &set_target_position;
    callback_functions[SET_PID] = &set_pid;
    callback_functions[SET_ODOMETRIE] = &set_odometrie;
    callback_functions[RESET_TEENSY] = &reset_teensy;
}

// 7. Timer interrupt handler (called every 10ms for control loop)
void handle() {
    holonomic_basis_ptr->handle(target_position, com);
}

void setup() {
    // Initialize serial communication
    com = new Com(&Serial, BAUDRATE);

    // Define the 3 stepper motors with their pins
    holonomic_basis_ptr->define_wheel1(W1_STEP_PIN, W1_DIR_PIN, W1_ENABLE_PIN);
    holonomic_basis_ptr->define_wheel2(W2_STEP_PIN, W2_DIR_PIN, W2_ENABLE_PIN);
    holonomic_basis_ptr->define_wheel3(W3_STEP_PIN, W3_DIR_PIN, W3_ENABLE_PIN);
    
    // Initialize motors
    holonomic_basis_ptr->init_motors();
    holonomic_basis_ptr->init_holonomic_basis(START_X, START_Y, START_THETA);
    
    // Enable motors
    holonomic_basis_ptr->enable_motors();

    // Initialize control loop timer (10ms = 100Hz)
    Timer1.initialize(ASSERVISSEMENT_FREQUENCY);
    Timer1.attachInterrupt(handle);

    // Initialize callback functions
    initialize_callback_functions();

    // Optional: Wait for serial connection
    delay(100);
}

uint_fast32_t counter = 0;

void loop() {
    // CRITICAL: Run stepper motors (must be called as often as possible)
    holonomic_basis_ptr->run_motors();

    // Handle communication
    com->handle_callback(callback_functions);

    // Send holonomic basis state periodically
    if (counter++ > 4096)  // 4096 = 2^12
    {
        msg_update_rolling_basis holonomic_basis_msg;
        // Holonomic Basis position
        holonomic_basis_msg.x = holonomic_basis_ptr->X;
        holonomic_basis_msg.y = holonomic_basis_ptr->Y;
        holonomic_basis_msg.theta = holonomic_basis_ptr->THETA;

        com->send_msg((byte*)&holonomic_basis_msg,
                      sizeof(msg_update_rolling_basis));
        counter = 0;
    }
}

/*

 This code was realized by Florian BARRE
    ____ __
   / __// /___
  / _/ / // _ \
 /_/  /_/ \___/
 
 Adapted for 3-wheel holonomic base with STEPPER MOTORS (NO ENCODERS)

*/