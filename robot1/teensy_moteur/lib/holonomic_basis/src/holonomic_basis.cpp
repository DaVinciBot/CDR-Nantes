/**
 * Implementation of Holonomic Basis for MKS SERVO57D over RS485.
 */

#include <Arduino.h>
#include <holonomic_basis.h>
#include <cstdio>

// Fonction utilitaire pour normaliser l'angle entre -PI et PI
double normalizeAngle(double theta) {
    // shift by +PI, take modulo 2*PI, remap to [0,2*PI)
    theta = fmod(theta + M_PI, 2.0 * M_PI);
    if (theta < 0.0) {
        theta += 2.0 * M_PI;
    }
    // shift back to [-PI, +PI)
    return theta - M_PI;
}

namespace {

// Control period, s: the ISR runs on a fixed timer, so the nominal period is
// the tick duration.
constexpr double CONTROL_PERIOD_S = ASSERVISSEMENT_FREQUENCY * 1e-6;

// Dead zone around the target: closer than this, the robot is brought to rest.
constexpr double ARRIVED_DISTANCE_MM = 10.0;
constexpr double ARRIVED_ANGLE_RAD = 0.15;

motion::MotionParams motion_params(double robot_radius, double wheel_diameter,
                                   double max_speed_rpm) {
    motion::MotionParams p;
    p.robot_radius_mm = robot_radius;
    p.wheel_diameter_mm = wheel_diameter;
    p.max_wheel_rpm = max_speed_rpm;
    p.max_wheel_accel_rpm_s = MAX_WHEEL_ACCEL_RPM_S;
    p.max_accel_mm_s2 = ROBOT_MAX_ACCEL_MM_S2;
    // The same wheel-level budget spent on rotation: a = alpha * R.
    p.max_alpha_rad_s2 = ROBOT_MAX_ACCEL_MM_S2 / robot_radius;
    return p;
}

}  // namespace

// Constructor
Holonomic_Basis::Holonomic_Basis(double robot_radius,
                                 double wheel_diameter,
                                 double max_speed_rpm,
                                 const PID& x_pid,
                                 const PID& y_pid,
                                 const PID& theta_pid)
    : x_pid(x_pid),
      y_pid(y_pid),
      theta_pid(theta_pid),
      robot_radius(robot_radius),
      wheel_diameter(wheel_diameter),
      max_speed_rpm(max_speed_rpm),
      shaper(motion_params(robot_radius, wheel_diameter, max_speed_rpm)) {
    // Initialisation des pointeurs à null
    wheel1 = nullptr;
    wheel2 = nullptr;
    wheel3 = nullptr;
    mksGroup = nullptr;

    odo_data.last_enc1 = 0;
    odo_data.last_enc2 = 0;
    odo_data.last_enc3 = 0;
}

// Destructor
Holonomic_Basis::~Holonomic_Basis() {
    // Nettoyage de la mémoire si nécessaire
    delete wheel1;
    delete wheel2;
    delete wheel3;
    delete mksGroup;
    // The sensors are owned by the caller (attach_sensors).
}

// === DÉFINITION DES MOTEURS ===

void Holonomic_Basis::define_wheel1(HardwareSerial& serial, uint8_t addr) {
    wheel1 = new MKSServo(serial, addr, MKS_MSTEP);
    wheel1->begin(MKS_BAUDRATE);
}
void Holonomic_Basis::define_wheel2(HardwareSerial& serial, uint8_t addr) {
    wheel2 = new MKSServo(serial, addr, MKS_MSTEP);
    wheel2->begin(MKS_BAUDRATE);
}
void Holonomic_Basis::define_wheel3(HardwareSerial& serial, uint8_t addr) {
    wheel3 = new MKSServo(serial, addr, MKS_MSTEP);
    wheel3->begin(MKS_BAUDRATE);
}

// === INITIALISATION ===
void Holonomic_Basis::init_motors() {
    mksGroup = new MKSGroup(wheel1, wheel2, wheel3);
}
void Holonomic_Basis::init_holonomic_basis(double x, double y, double theta) {
    noInterrupts();
    this->X = x;
    this->Y = y;
    this->THETA = theta;
    if (estimator) estimator->reset(x, y, theta);
    interrupts();

    if (optical) optical->reset_tracking();
}

void Holonomic_Basis::calibrate_imu_origin() {
    if (use_imu && imu) imu->zero_heading();
    noInterrupts();
    this->THETA = 0.0;
    if (estimator) estimator->reset(this->X, this->Y, 0.0);
    interrupts();
}

// === ESTIMATEUR DE POSE ===

void Holonomic_Basis::set_estimator(pose::PoseEstimator* next) {
    if (!next) return;
    noInterrupts();
    next->reset(this->X, this->Y, this->THETA);
    estimator = next;
    interrupts();
}

bool Holonomic_Basis::select_estimator(pose::PoseEstimator* next) {
    if (!next) return false;
    // Only at rest: a switch while moving would hand over a pose mid-tick, and
    // the new estimator would start with a wrong velocity.
    if (filtered_wheel1_rpm != 0.0 || filtered_wheel2_rpm != 0.0 ||
        filtered_wheel3_rpm != 0.0) {
        return false;
    }
    set_estimator(next);
    return true;
}

// === GESTION ÉTAT MOTEURS ===

void Holonomic_Basis::enable_motors() {
    if (wheel1) wheel1->enable();
    if (wheel2) wheel2->enable();
    if (wheel3) wheel3->enable();
}

void Holonomic_Basis::disable_motors() {
    if (wheel1) wheel1->disable();
    if (wheel2) wheel2->disable();
    if (wheel3) wheel3->disable();
}

// === ODOMÉTRIE & PID ===
void Holonomic_Basis::attach_sensors(sensors::Bno085Sensor* imu_sensor,
                                     sensors::OtosSensor* optical_sensor) {
    noInterrupts();
    imu = imu_sensor;
    optical = optical_sensor;
    interrupts();
}

Point Holonomic_Basis::get_current_position() {
    Point position;
    noInterrupts(); // Section critique pour lire les doubles de manière atomique
    position.x = this->X;
    position.y = this->Y;
    position.theta = this->THETA;
    interrupts();
    return position;
}

// Builds the estimator input of the tick from the buffers filled in loop().
// Called from the ISR: no I/O here, only copies under noInterrupts().
void Holonomic_Basis::update_odometry() {
    // Ask loop() for the next sensor samples
    if (use_imu && imu) need_read_imu = true;
    if (use_optical_flow && optical) need_read_optical = true;

    int64_t enc1, enc2, enc3;
    uint32_t seq, enc_us;
    {
        noInterrupts();
        enc1 = odo_data.buffered_enc1;
        enc2 = odo_data.buffered_enc2;
        enc3 = odo_data.buffered_enc3;
        seq = odo_data.buffer_seq;
        enc_us = odo_data.buffer_timestamp_us;
        interrupts();
    }
    const uint32_t now_us = micros();

    pose::EstimatorInput in = pose::empty_input();

    // First tick: no motion computed at startup, only the references are taken.
    if (!odo_data.started) {
        in.first_frame = true;
        odo_data.started = true;
    } else {
        // Encoder counts increase in the opposite direction to commanded RPM on
        // all 3 motors. Negate so that positive d = forward wheel rotation, the
        // convention of lib/kinematics.
        in.d1 = -static_cast<int32_t>(enc1 - odo_data.last_enc1);
        in.d2 = -static_cast<int32_t>(enc2 - odo_data.last_enc2);
        in.d3 = -static_cast<int32_t>(enc3 - odo_data.last_enc3);
        in.dt_us = now_us - odo_data.last_tick_us;
    }

    // The deltas span exactly one tick only if the buffer was refreshed this tick
    // AND the previous one. After a missed read they are zero (stale buffer),
    // then cover two ticks at once: a speed computed over one dt would be wrong.
    const bool fresh_read = (seq != odo_data.last_seq);
    in.encoders_fresh = !in.first_frame && fresh_read && odo_data.prev_read_fresh;
    in.encoder_age_us = now_us - enc_us;

    // IMU: heading in the table frame, origin set by calibrate_imu_origin()
    if (use_imu && imu) {
        const sensors::HeadingSample h = imu->latest();
        in.imu_valid = h.valid;
        in.heading_rad = h.heading_rad;
        in.imu_timestamp_us = h.timestamp_us;
    }

    // Optical: a NEW sample only, handed over once. Robot-frame step for the
    // complementary filter, robot-frame velocity for the EKF.
    if (use_optical_flow && optical) {
        const sensors::OpticalSample o = optical->latest();
        if (o.valid && o.seq != odo_data.last_optical_seq) {
            odo_data.last_optical_seq = o.seq;
            in.optical_timestamp_us = o.timestamp_us;
            in.optical_step_valid = true;
            in.optical_dx_mm = o.dx_mm;
            in.optical_dy_mm = o.dy_mm;
            in.optical_step_dt_us = o.step_dt_us;
            in.optical_velocity_valid = true;
            in.optical_vx_mm_s = o.vx_mm_s;
            in.optical_vy_mm_s = o.vy_mm_s;
        }
    }

    odo_data.last_enc1 = enc1;
    odo_data.last_enc2 = enc2;
    odo_data.last_enc3 = enc3;
    odo_data.last_seq = seq;
    odo_data.prev_read_fresh = fresh_read;
    odo_data.last_tick_us = now_us;

    if (!estimator) return;
    estimator->step(in);
    const pose::PoseEstimate p = estimator->pose();
    this->X = p.x_mm;
    this->Y = p.y_mm;
    this->THETA = p.theta_rad;
}

void Holonomic_Basis::handle(Point target_position, Com* com) {
    (void)com;

    // Errors in the table frame
    const double xerr = target_position.x - this->X;
    const double yerr = target_position.y - this->Y;
    const double theta_error = normalizeAngle(target_position.theta - this->THETA);
    const double distance_error = sqrt(xerr * xerr + yerr * yerr);

    motion::VelocityCommand position_loop = {0.0, 0.0, 0.0};
    motion::VelocityCommand ff = {0.0, 0.0, 0.0};

    // Dead zone: close enough, the robot is brought to rest through the
    // acceleration limit (zero command), not by a jump of the wheels to zero.
    const bool arrived = distance_error < ARRIVED_DISTANCE_MM &&
                         fabs(theta_error) < ARRIVED_ANGLE_RAD;
    if (!arrived) {
        // Anti-windup: while the previous command was capped after the PIDs,
        // their error is not theirs to correct; hold the integrals.
        const motion::ShapedCommand& previous = shaper.last();
        const bool capped = previous.braking || previous.speed_saturated ||
                            previous.accel_limited;
        position_loop.vx_mm_s = this->x_pid.compute(xerr, capped);
        position_loop.vy_mm_s = this->y_pid.compute(yerr, capped);
        position_loop.omega_rad_s = this->theta_pid.compute(theta_error, capped);
        ff = this->feed_forward;
    }

    const motion::ShapedCommand& command =
        shaper.step(position_loop, ff, this->THETA, distance_error,
                    fabs(theta_error), CONTROL_PERIOD_S);
    filtered_wheel1_rpm = command.wheel_rpm[0];
    filtered_wheel2_rpm = command.wheel_rpm[1];
    filtered_wheel3_rpm = command.wheel_rpm[2];
}

void Holonomic_Basis::set_velocity_feedforward(double vx_mm_s, double vy_mm_s,
                                               double omega_rad_s) {
    noInterrupts();
    feed_forward.vx_mm_s = vx_mm_s;
    feed_forward.vy_mm_s = vy_mm_s;
    feed_forward.omega_rad_s = omega_rad_s;
    interrupts();
}

motion::ShapedCommand Holonomic_Basis::last_command() {
    noInterrupts();
    const motion::ShapedCommand copy = shaper.last();
    interrupts();
    return copy;
}

// Arrêt d'urgence
void Holonomic_Basis::emergency_stop() {
    if (mksGroup) {
        mksGroup->emergencyStopAll();
    }
    // The drivers stopped without a ramp: the shaper must restart from rest,
    // not ramp down from speeds the wheels no longer have.
    noInterrupts();
    shaper.reset();
    filtered_wheel1_rpm = 0.0;
    filtered_wheel2_rpm = 0.0;
    filtered_wheel3_rpm = 0.0;
    interrupts();
}

// ===== ARCHITECTURE NON-BLOQUANTE: RS485 SORTIS DE L'ISR =====

// Lire les encodeurs DEPUIS loop() - rafale synchronisée, ~2ms total
bool Holonomic_Basis::read_encoders_nonblocking() {
    if (!mksGroup) return false;
    
    int64_t enc1 = 0, enc2 = 0, enc3 = 0;
    
    // Appel RS485 bloquant optimisé : envoie 3 requêtes en rafale, lit 3 réponses en parallèle
    // ~2ms au lieu de ~150ms avec readAllEncoders() séquentiel
    bool success = mksGroup->readAllEncodersSynced(enc1, enc2, enc3);
    
    if (success) {
        // Stocker dans buffers pour que update_odometry() (ISR) puisse les lire.
        // 64-bit values: written with interrupts off so the ISR never sees half.
        noInterrupts();
        odo_data.buffered_enc1 = enc1;
        odo_data.buffered_enc2 = enc2;
        odo_data.buffered_enc3 = enc3;
        odo_data.buffer_timestamp_us = micros();
        odo_data.buffer_seq++;
        interrupts();
        
        #ifdef DEBUG_RS485
        static uint32_t enc_read_counter = 0;
        if (++enc_read_counter >= 10) {  // Log toutes les 10 lectures
            enc_read_counter = 0;
            Serial.printf("[RS485 OK] Enc: %lld, %lld, %lld\n", enc1, enc2, enc3);
        }
        #endif
    } else {
        // Timeout RS485 - log mais pas de crash
        static uint32_t enc_fail_counter = 0;
        if (++enc_fail_counter >= 5) {  // Log une fois sur 5
            enc_fail_counter = 0;
            Serial.printf("[RS485 TIMEOUT] readAllEncoders() failed, retry next loop\n");
        }
    }
    
    return success;
}

// Envoyer les commandes vitesse DEPUIS loop() - 30ms max (3x 10ms), pas d'impact sur ISR
bool Holonomic_Basis::send_movement_commands_nonblocking() {
    if (!mksGroup) return false;
    
    // Appel RS485 bloquant - c'est OK ici, on est dans loop() pas dans ISR
    // setSpeedsSynced envoie 3 commandes 0xF6 (10ms timeout chacune)
    mksGroup->setSpeedsSynced(filtered_wheel1_rpm,
                              filtered_wheel2_rpm,
                              filtered_wheel3_rpm,
                              MKS_ACC);
    
    #ifdef DEBUG_RS485
    static uint32_t movement_counter = 0;
    if (++movement_counter >= 10) {  // Log toutes les 10 fois
        movement_counter = 0;
        Serial.printf("[RS485 MOVE] W1: %.0f, W2: %.0f, W3: %.0f RPM\n",
                      filtered_wheel1_rpm, filtered_wheel2_rpm, filtered_wheel3_rpm);
    }
    #endif
    
    return true;  // On considère OK même si timeout silencieux (moteur peut fonctionner)
}
