/**
 * Implementation of Holonomic Basis for STEPPER MOTORS WITH ENCODERS
 * Uses steppers for control and encoders for precise odometry
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
Holonomic_Basis::Holonomic_Basis(unsigned short encoder_resolution,
                                 double robot_radius,
                                 double wheel_diameter,
                                 double max_speed,
                                 double max_acceleration,
                                 unsigned short steps_per_revolution,
                                 unsigned short microsteps,
                                 const PID& x_pid,
                                 const PID& y_pid,
                                 const PID& theta_pid)
    : encoder_resolution(encoder_resolution),
      robot_radius(robot_radius),
      wheel_diameter(wheel_diameter),
      max_speed(max_speed),
      max_acceleration(max_acceleration),
      steps_per_revolution(steps_per_revolution),
      microsteps(microsteps),
      x_pid(x_pid),
      y_pid(y_pid),
      theta_pid(theta_pid) {
    wheel1 = new StepperWithEncoder();
    wheel2 = new StepperWithEncoder();
    wheel3 = new StepperWithEncoder();
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    if (wheel1) {
        if (wheel1->stepper) delete wheel1->stepper;
        delete wheel1;
    }
    if (wheel2) {
        if (wheel2->stepper) delete wheel2->stepper;
        delete wheel2;
    }
    if (wheel3) {
        if (wheel3->stepper) delete wheel3->stepper;
        delete wheel3;
    }
}

// Define wheels
void Holonomic_Basis::define_wheel1(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel1->stepper = new AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin);
    wheel1->enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

void Holonomic_Basis::define_wheel2(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel2->stepper = new AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin);
    wheel2->enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

void Holonomic_Basis::define_wheel3(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel3->stepper = new AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin);
    wheel3->enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

// Initialize motors
void Holonomic_Basis::init_motors() {
    if (wheel1->stepper) {
        wheel1->stepper->setMaxSpeed(max_speed);
        wheel1->stepper->setAcceleration(max_acceleration);
    }
    if (wheel2->stepper) {
        wheel2->stepper->setMaxSpeed(max_speed);
        wheel2->stepper->setAcceleration(max_acceleration);
    }
    if (wheel3->stepper) {
        wheel3->stepper->setMaxSpeed(max_speed);
        wheel3->stepper->setAcceleration(max_acceleration);
    }
}

// Initialize position
void Holonomic_Basis::init_holonomic_basis(double x, double y, double theta) {
    this->X = x;
    this->Y = y;
    this->THETA = theta;
    
    // Reset encoder ticks
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        wheel1->ticks = 0;
        wheel2->ticks = 0;
        wheel3->ticks = 0;
    }
    
    wheel1->distance = 0.0;
    wheel2->distance = 0.0;
    wheel3->distance = 0.0;
}

// Enable/Disable motors
void Holonomic_Basis::enable_motors() {
    digitalWrite(wheel1->enable_pin, LOW);  // Active LOW
    digitalWrite(wheel2->enable_pin, LOW);
    digitalWrite(wheel3->enable_pin, LOW);
}

void Holonomic_Basis::disable_motors() {
    digitalWrite(wheel1->enable_pin, HIGH);
    digitalWrite(wheel2->enable_pin, HIGH);
    digitalWrite(wheel3->enable_pin, HIGH);
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

// Odometry using encoders (like DC motors)
void Holonomic_Basis::odometrie_handle() {
    // Read encoder ticks atomically
    long current_ticks1, current_ticks2, current_ticks3;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        current_ticks1 = wheel1->ticks;
        current_ticks2 = wheel2->ticks;
        current_ticks3 = wheel3->ticks;
        
        // Reset ticks for next iteration
        wheel1->ticks = 0;
        wheel2->ticks = 0;
        wheel3->ticks = 0;
    }
    
    // Convert ticks to distance
    double d1 = current_ticks1 * wheel_unit_tick_cm();
    double d2 = current_ticks2 * wheel_unit_tick_cm();
    double d3 = current_ticks3 * wheel_unit_tick_cm();
    
    // Store distance for debugging if needed
    wheel1->distance = d1;
    wheel2->distance = d2;
    wheel3->distance = d3;
    
    // Forward kinematics: wheel displacements -> robot displacements (robot frame)
    double dx_robot = (2.0 * d1 - d2 - d3) / 3.0;
    double dy_robot = sqrt(3.0) * (d2 - d3) / 3.0;
    double dtheta = (d1 + d2 + d3) / (3.0 * this->robot_radius);
    
    // Transform from robot frame to world frame
    double cos_theta = cosf(this->THETA);
    double sin_theta = sinf(this->THETA);
    
    this->X += cos_theta * dx_robot - sin_theta * dy_robot;
    this->Y += sin_theta * dx_robot + cos_theta * dy_robot;
    this->THETA = normalizeAngle(this->THETA + dtheta);
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
    // Pour steppers: on utilise directement les valeurs PID comme vitesse (steps/sec)
    // La conversion est faite en multipliant par un facteur
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
    wheel1->stepper->setSpeed(wheel1_speed);
    wheel2->stepper->setSpeed(wheel2_speed);
    wheel3->stepper->setSpeed(wheel3_speed);
}

// Run motors (must be called frequently)
void Holonomic_Basis::run_motors() {
    wheel1->stepper->runSpeed();
    wheel2->stepper->runSpeed();
    wheel3->stepper->runSpeed();
}

// Emergency stop
void Holonomic_Basis::emergency_stop() {
    wheel1->stepper->stop();
    wheel2->stepper->stop();
    wheel3->stepper->stop();
    wheel1->stepper->setSpeed(0);
    wheel2->stepper->setSpeed(0);
    wheel3->stepper->setSpeed(0);
}