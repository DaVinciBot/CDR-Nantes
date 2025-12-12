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
    // TeensyStep ne nécessite pas d'allocation dynamique
    wheel1 = nullptr;
    wheel2 = nullptr;
    wheel3 = nullptr;
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    // TeensyStep utilise des objets statiques, pas de delete nécessaire
}

// Define wheels avec TeensyStep
void Holonomic_Basis::define_wheel1(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel1 = new Stepper(step_pin, dir_pin);
    wheel1_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

void Holonomic_Basis::define_wheel2(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel2 = new Stepper(step_pin, dir_pin);
    wheel2_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

void Holonomic_Basis::define_wheel3(byte step_pin, byte dir_pin, byte enable_pin) {
    wheel3 = new Stepper(step_pin, dir_pin);
    wheel3_enable_pin = enable_pin;
    pinMode(enable_pin, OUTPUT);
}

// Initialize motors avec TeensyStep
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
    
    // Wheel 1 (front - roue A)
    double wheel1_speed = 0.5 * vx_steps - (sqrt(3.0) / 2.0) * vy_steps - omega_steps;
    
    // Wheel 2 (back-left - roue B)
    double wheel2_speed = 0.5 * vx_steps + (sqrt(3.0) / 2.0) * vy_steps - omega_steps;
    
    // Wheel 3 (back-right - roue C)
    double wheel3_speed = -vx_steps - omega_steps;
    
    // Clamp speeds
    wheel1_speed = constrain(wheel1_speed, -max_speed, max_speed);
    wheel2_speed = constrain(wheel2_speed, -max_speed, max_speed);
    wheel3_speed = constrain(wheel3_speed, -max_speed, max_speed);
    
    // Stocker les vitesses calculées (ne pas appliquer ici)
    last_wheel1_speed = wheel1_speed;
    last_wheel2_speed = wheel2_speed;
    last_wheel3_speed = wheel3_speed;
}
}

// Run motors (TeensyStep gère automatiquement)
void Holonomic_Basis::run_motors() {
    // TeensyStep gère les moteurs automatiquement via des interruptions
    // Cette fonction peut rester vide ou être utilisée pour d'autres tâches
}

// Execute movement avec TeensyStep
void Holonomic_Basis::execute_movement() {
    // Période fixe correspondant au timer (5ms)
    float dt = 0.005; // 5ms = fréquence du Timer3
    
    // Calculer les steps pour cette période (vitesse en steps/sec * temps en sec)
    int32_t steps1 = (int32_t)(last_wheel1_speed * dt);
    int32_t steps2 = (int32_t)(last_wheel2_speed * dt);
    int32_t steps3 = (int32_t)(last_wheel3_speed * dt);
    
    // Appliquer les steps incrémentaux
    wheel1->setTargetRel(steps1);
    wheel2->setTargetRel(steps2);
    wheel3->setTargetRel(steps3);
    
    // Mouvement coordonné
    controller.move(*wheel1, *wheel2, *wheel3);
}

// Emergency stop
void Holonomic_Basis::emergency_stop() {
    controller.emergencyStop();
    wheel1->setTargetRel(0);
    wheel2->setTargetRel(0);
    wheel3->setTargetRel(0);
}