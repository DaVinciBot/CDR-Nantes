/**
 * This is the implementation of the Holonomic Basis class.
 * The Holonomic Basis class is the core of the Motor teensy code.
 * It provides method to control the 3 motors, the speed and the orientation.
 * It computes the odometry and correct the motors error with the PID class.
 * ADAPTED FOR 3-WHEEL HOLONOMIC BASE (120° between wheels)
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

// Properties
/**
 * @brief Get current position (X, Y and THETA) of the robot
 *
 * @return Current position
 */
Point Holonomic_Basis::get_current_position() {
    Point position;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        position.x = this->X;
        position.y = this->Y;
        position.theta = this->THETA;
    }
    return position;
}

// Constructor
/**
 * @brief constructor of the Holonomic Basis class
 *
 * Initializes the parameters of the Holonomic Basis
 */
Holonomic_Basis::Holonomic_Basis(unsigned short encoder_resolution,
                                 double robot_radius,
                                 double wheel_diameter,
                                 const PID& x_pid,
                                 const PID& y_pid,
                                 const PID& theta_pid)
    : encoder_resolution(encoder_resolution),
      robot_radius(robot_radius),
      wheel_diameter(wheel_diameter),
      x_pid(x_pid),
      y_pid(y_pid),
      theta_pid(theta_pid) {}

// Methods
// Inits function
/**
 * @brief Define wheel 1 (front - 0°) with pins, related encoders pin and properties
 */
void Holonomic_Basis::define_wheel1(byte enca,
                                    byte encb,
                                    byte pwm,
                                    byte in2,
                                    byte in1,
                                    byte max_pwm) {
    this->wheel1 = new Motor(in1, in2, pwm, enca, encb,
                             this->wheel_unit_tick_cm(), max_pwm);
}

/**
 * @brief Define wheel 2 (back-left - 120°) with pins, related encoders pin and properties
 */
void Holonomic_Basis::define_wheel2(byte enca,
                                    byte encb,
                                    byte pwm,
                                    byte in2,
                                    byte in1,
                                    byte max_pwm) {
    this->wheel2 = new Motor(in1, in2, pwm, enca, encb,
                             this->wheel_unit_tick_cm(), max_pwm);
}

/**
 * @brief Define wheel 3 (back-right - 240°) with pins, related encoders pin and properties
 */
void Holonomic_Basis::define_wheel3(byte enca,
                                    byte encb,
                                    byte pwm,
                                    byte in2,
                                    byte in1,
                                    byte max_pwm) {
    this->wheel3 = new Motor(in1, in2, pwm, enca, encb,
                             this->wheel_unit_tick_cm(), max_pwm);
}

/**
 * @brief Initialize all three motors
 */
void Holonomic_Basis::init_motors() {
    this->wheel1->init();
    this->wheel2->init();
    this->wheel3->init();
}

/**
 * @brief Initialize Holonomic Basis state with starting position
 */
void Holonomic_Basis::init_holonomic_basis(double x, double y, double theta) {
    this->X = x;
    this->Y = y;
    this->THETA = theta;
}

// Odometrie function
/**
 * @brief Handle the odometry computation for holonomic base
 *
 * Forward kinematics for 3-wheel holonomic base:
 * Vx_robot = (2*V1 - V2 - V3) / 3
 * Vy_robot = sqrt(3) * (V2 - V3) / 3
 * Vtheta = (V1 + V2 + V3) / (3 * R)
 */
void Holonomic_Basis::odometrie_handle() {
    /* Update motors positions by calling odometer_handle */
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        this->wheel1->handle_odometrie();
        this->wheel2->handle_odometrie();
        this->wheel3->handle_odometrie();
    }

    /* Get displacements from the 3 wheels */
    double d1 = this->wheel1->distance;
    double d2 = this->wheel2->distance;
    double d3 = this->wheel3->distance;

    /* Forward kinematics: wheel velocities -> robot velocities (in robot frame) */
    double dx_robot = (2.0 * d1 - d2 - d3) / 3.0;
    double dy_robot = sqrt(3.0) * (d2 - d3) / 3.0;
    double dtheta = (d1 + d2 + d3) / (3.0 * this->robot_radius);

    /* Transform robot frame velocities to world frame */
    double cos_theta = cosf(this->THETA);
    double sin_theta = sinf(this->THETA);
    
    this->X += cos_theta * dx_robot - sin_theta * dy_robot;
    this->Y += sin_theta * dx_robot + cos_theta * dy_robot;
    this->THETA = normalizeAngle(this->THETA + dtheta);
}

/**
 * @brief Handle the correction computation for holonomic base
 *
 * Inverse kinematics for 3-wheel holonomic base:
 * V1 = Vx_robot + ω * R
 * V2 = -0.5 * Vx_robot + sqrt(3)/2 * Vy_robot + ω * R
 * V3 = -0.5 * Vx_robot - sqrt(3)/2 * Vy_robot + ω * R
 */
void Holonomic_Basis::handle(Point target_position, Com* com) {
    /* Compute position errors in world frame */
    double xerr = target_position.x - this->X;
    double yerr = target_position.y - this->Y;
    double theta_error = normalizeAngle(target_position.theta - this->THETA);

    /* Compute PID outputs (corrections in world frame) */
    double vx_world = this->x_pid.compute(xerr);
    double vy_world = this->y_pid.compute(yerr);
    double omega = this->theta_pid.compute(theta_error);

    /* Transform world frame corrections to robot frame */
    double cos_theta = cosf(this->THETA);
    double sin_theta = sinf(this->THETA);
    
    double vx_robot = cos_theta * vx_world + sin_theta * vy_world;
    double vy_robot = -sin_theta * vx_world + cos_theta * vy_world;

    /* Inverse kinematics: robot velocities -> wheel velocities */
    // Wheel 1 (front - 0°): V1 = Vx + ω*R
    double wheel1_pwm = vx_robot + omega * this->robot_radius;
    
    // Wheel 2 (back-left - 120°): V2 = -0.5*Vx + sqrt(3)/2*Vy + ω*R
    double wheel2_pwm = -0.5 * vx_robot + (sqrt(3.0) / 2.0) * vy_robot + omega * this->robot_radius;
    
    // Wheel 3 (back-right - 240°): V3 = -0.5*Vx - sqrt(3)/2*Vy + ω*R
    double wheel3_pwm = -0.5 * vx_robot - (sqrt(3.0) / 2.0) * vy_robot + omega * this->robot_radius;

    /* Optional: Debug output */
    // static long ticks_counter = 0;
    // if (ticks_counter++ > 10) {
    //     ticks_counter = 0;
    //     String pwms = "PWMs: " + String(wheel1_pwm) + ", " + String(wheel2_pwm) + ", " + String(wheel3_pwm);
    //     com->print((char *)pwms.c_str());
    // }

    /* Apply PWM to motors */
    this->wheel1->set_motor(wheel1_pwm);
    this->wheel2->set_motor(wheel2_pwm);
    this->wheel3->set_motor(wheel3_pwm);
}