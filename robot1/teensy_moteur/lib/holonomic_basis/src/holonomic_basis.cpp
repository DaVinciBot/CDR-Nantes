/**
 * Implementation of Holonomic Basis for STEPPER MOTORS (WITHOUT ENCODERS)
 * Uses steppers for control only
 */

#include <Arduino.h>
#include <holonomic_basis.h>
#include <util/atomic.h>

double normalizeAngle(double theta) {
    // shift by +PI, take modulo 2*PI, remap to [0,2*PI)
    theta = fmodf(theta + PI, 2.0f * PI);
    if (theta < 0.0f) {
        theta += 2.0f * PI;
    }
    // shift back to [-PI, +PI)
    return theta - PI;
}

// Constructor
Holonomic_Basis::Holonomic_Basis(double robot_radius,
                                 double wheel_diameter,
                                 double max_speed,
                                 double max_acceleration,
                                 unsigned short steps_per_revolution,
                                 unsigned short microsteps,
                                 const PID& x_pid,
                                 const PID& y_pid,
                                 const PID& theta_pid)
    : robot_radius(robot_radius),
      wheel_diameter(wheel_diameter),
      max_speed(max_speed),
      max_acceleration(max_acceleration),
      steps_per_revolution(steps_per_revolution),
      microsteps(microsteps),
      x_pid(x_pid),
      y_pid(y_pid),
      theta_pid(theta_pid) {
    wheel1 = new AccelStepper();
    wheel2 = new AccelStepper();
    wheel3 = new AccelStepper();
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    if (wheel1) delete wheel1;
    if (wheel2) delete wheel2;
    if (wheel3) delete wheel3;
}

// Define wheels
void Holonomic_Basis::define_wheel1(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel1 = new AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin);
    wheel1_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

void Holonomic_Basis::define_wheel2(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel2 = new AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin);
    wheel2_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

void Holonomic_Basis::define_wheel3(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel3 = new AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin);
    wheel3_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

// Initialize motors
void Holonomic_Basis::init_motors() {
    if (wheel1) {
        wheel1->setMaxSpeed(max_speed);
        wheel1->setAcceleration(max_acceleration);
    }
    if (wheel2) {
        wheel2->setMaxSpeed(max_speed);
        wheel2->setAcceleration(max_acceleration);
    }
    if (wheel3) {
        wheel3->setMaxSpeed(max_speed);
        wheel3->setAcceleration(max_acceleration);
    }
}

// Initialize position
void Holonomic_Basis::init_holonomic_basis(double x, double y, double theta) {
    this->X = x;
    this->Y = y;
    this->THETA = theta;
}

// Enable/Disable motors
void Holonomic_Basis::enable_motors() {
    digitalWrite(wheel1_enable_pin, LOW);  // Active LOW
    digitalWrite(wheel2_enable_pin, LOW);
    digitalWrite(wheel3_enable_pin, LOW);
}

void Holonomic_Basis::disable_motors() {
    digitalWrite(wheel1_enable_pin, HIGH);
    digitalWrite(wheel2_enable_pin, HIGH);
    digitalWrite(wheel3_enable_pin, HIGH);
}

// Get current position
Point Holonomic_Basis::get_current_position() {
    Point position;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        position.x = this->X;
        position.y = this->Y;
        position.theta = this->THETA;
    }
    return position;
}

// Control handle
void Holonomic_Basis::handle(Point target_position, Com* com) {
    // Compute position errors in world frame
    double xerr = target_position.x - this->X;
    double yerr = target_position.y - this->Y;
    double theta_error = normalizeAngle(target_position.theta - this->THETA);
    
    // Compute PID outputs (speed corrections in world frame)
    double vx_world = this->x_pid.compute(xerr);
    double vy_world = this->y_pid.compute(yerr);
    double omega = this->theta_pid.compute(theta_error);
    
    // Transform world frame corrections to robot frame
    double cos_theta = cosf(this->THETA);
    double sin_theta = sinf(this->THETA);
    
    double vx_robot = cos_theta * vx_world + sin_theta * vy_world;
    double vy_robot = -sin_theta * vx_world + cos_theta * vy_world;
    
    // Inverse kinematics: robot velocities -> wheel speeds
    double speed_factor = (steps_per_revolution * microsteps) / wheel_circumference();
    
    double vx_steps = vx_robot * speed_factor;
    double vy_steps = vy_robot * speed_factor;
    double omega_steps = omega * robot_radius * speed_factor;
    
    // Wheel 1 (front - 0°)
    double wheel1_speed = vx_steps + omega_steps;
    
    // Wheel 2 (back-left - 120°)
    double wheel2_speed = -0.5 * vx_steps + (sqrt(3.0) / 2.0) * vy_steps + omega_steps;
    
    // Wheel 3 (back-right - 240°)
    double wheel3_speed = -0.5 * vx_steps - (sqrt(3.0) / 2.0) * vy_steps + omega_steps;
    
    // Clamp speeds
    wheel1_speed = constrain(wheel1_speed, -max_speed, max_speed);
    wheel2_speed = constrain(wheel2_speed, -max_speed, max_speed);
    wheel3_speed = constrain(wheel3_speed, -max_speed, max_speed);
    
    // Set motor speeds
    wheel1->setSpeed(wheel1_speed);
    wheel2->setSpeed(wheel2_speed);
    wheel3->setSpeed(wheel3_speed);
}

// Run motors (must be called frequently)
void Holonomic_Basis::run_motors() {
    wheel1->runSpeed();
    wheel2->runSpeed();
    wheel3->runSpeed();
}

// Emergency stop
void Holonomic_Basis::emergency_stop() {
    wheel1->stop();
    wheel2->stop();
    wheel3->stop();
    wheel1->setSpeed(0);
    wheel2->setSpeed(0);
    wheel3->setSpeed(0);
}