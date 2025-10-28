#pragma once
// Externe libraries used: Arduino, TimerOne, ATOMIC
#include <Arduino.h>      // Arduino framework
#include <TimerOne.h>     // Timer interrupt library
#include <util/atomic.h>  // Atomic block library

// Custom libraries used: Holonomic_Basis, Com
#include <holonomic_basis.h>  // Holonomic Basis object to manage the motors and robot position

// Configuration file (contains all the constants and pinout), it is just a
// main.cpp header file
#include <config.h>

// 1. Instanciate the Holonomic Basis object
// a. Define the 3 PID controllers (X, Y, THETA)
PID x_pid(KP_X,
          KI_X,
          KD_X,
          -240,
          240,
          5.0);

PID y_pid(KP_Y,
          KI_Y,
          KD_Y,
          -240,
          240,
          5.0);

PID theta_pid(KP_THETA,
              KI_THETA,
              KD_THETA,
              -200,
              200,
              2.0);

// b. Instanciate the Holonomic Basis object
Holonomic_Basis* holonomic_basis_ptr = new Holonomic_Basis(ENCODER_RESOLUTION,
                                                           ROBOT_RADIUS,
                                                           WHEEL_DIAMETER,
                                                           x_pid,
                                                           y_pid,
                                                           theta_pid);

// 2. Instanciate the Communication object
Com* com;

// c. Define the 3 motors interrupt functions
/******* Attach Interrupt *******/
inline void wheel1_read_encoder() {
    if (digitalRead(W1_ENCB))
        holonomic_basis_ptr->wheel1->ticks--;
    else
        holonomic_basis_ptr->wheel1->ticks++;
}

inline void wheel2_read_encoder() {
    if (digitalRead(W2_ENCB))
        holonomic_basis_ptr->wheel2->ticks--;
    else
        holonomic_basis_ptr->wheel2->ticks++;
}

inline void wheel3_read_encoder() {
    if (digitalRead(W3_ENCB))
        holonomic_basis_ptr->wheel3->ticks--;
    else
        holonomic_basis_ptr->wheel3->ticks++;
}

// 3. Define all com callback functions
// a. define globals variables to keep in memory callback functions updated
Point target_position(START_X, START_Y, START_THETA);

// b. define the callback functions 
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
    // TODO: reset the teensy, à tester !
    void (*reboot)(void) = 0;
    reboot();
}

// c. assign the callback functions to the right message id
void (*callback_functions[256])(byte* msg, byte size);

void initialize_callback_functions() {
    callback_functions[SET_TARGET_POSITION] = &set_target_position;
    callback_functions[SET_PID] = &set_pid;
    callback_functions[SET_ODOMETRIE] = &set_odometrie;
    callback_functions[RESET_TEENSY] = &reset_teensy;
}

// 4. Define the timer interrupt handle function (this function will be called
// every 10ms, and which manage the robot position and speed: asservissement)
void handle() {
    holonomic_basis_ptr->odometrie_handle();
    holonomic_basis_ptr->handle(target_position, com);
}

void setup() {
    com = new Com(&Serial, BAUDRATE);

    // Change pwm frequency for all 3 wheels
    analogWriteFrequency(W1_PWM, PWM_FREQUENCY);
    analogWriteFrequency(W2_PWM, PWM_FREQUENCY);
    analogWriteFrequency(W3_PWM, PWM_FREQUENCY);

    // Init Holonomic Basis - Define all 3 wheels
    holonomic_basis_ptr->define_wheel1(W1_ENCA, W1_ENCB, W1_PWM, W1_IN2, W1_IN1, MAX_PWM);
    holonomic_basis_ptr->define_wheel2(W2_ENCA, W2_ENCB, W2_PWM, W2_IN2, W2_IN1, MAX_PWM);
    holonomic_basis_ptr->define_wheel3(W3_ENCA, W3_ENCB, W3_PWM, W3_IN2, W3_IN1, MAX_PWM);
    
    holonomic_basis_ptr->init_motors();
    holonomic_basis_ptr->init_holonomic_basis(START_X, START_Y, START_THETA);
    
    // Attach interrupts for all 3 encoders
    attachInterrupt(digitalPinToInterrupt(W1_ENCA), wheel1_read_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(W2_ENCA), wheel2_read_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(W3_ENCA), wheel3_read_encoder, RISING);

    // Init motors handle timer
    Timer1.initialize(ASSERVISSEMENT_FREQUENCY);
    Timer1.attachInterrupt(handle);

    // Initialize callback functions
    initialize_callback_functions();
}

uint_fast32_t counter = 0;
void loop() {
    // Handle the communication
    com->handle_callback(callback_functions);

    // Send holonomic basis state
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
 
 Adapted for 3-wheel holonomic base

*/