/*
 * Match firmware of the base: USB link to the Raspberry Pi, control tick,
 * telemetry.
 *
 * What runs where:
 *   - control_tick(), IntervalTimer every ASSERVISSEMENT_FREQUENCY us: pose
 *     estimate, position loop while a target is held, requests to loop(),
 *     telemetry of the tick (frames copied into a ring, never sent from here).
 *     No I/O;
 *   - loop(): USB messages, watchdog, RS485 (encoder read, then speed
 *     commands), IMU and optical polls, ODOMETRY and STATUS, then the frames
 *     waiting in the ring.
 *
 * Serial carries the protocol: nothing may print on it. Text goes out as a
 * PRINT frame through the telemetry ring.
 *
 * At boot nothing moves: the wheels only turn once a SET_TARGET arrived, and
 * stop again on STOP, SET_ODOMETRIE, or when the Raspberry Pi falls silent for
 * WATCHDOG_TIMEOUT_MS.
 *
 * Not here yet, left to the complete match firmware: movement states and
 * timeout, detection of a blocked base, loss of the drivers' power, PID output
 * bounds in physical units.
 *
 * Built with -D SYNTHETIC_ENCODERS (environment teensy41_synthetic), the
 * encoder counts are generated from the wheel commands instead of read from
 * the drivers: the whole firmware runs on a Teensy alone.
 */

#include <Arduino.h>
#include <string.h>

#include <config.h>
#include <com.h>
#include <holonomic_basis.h>
#include <complementary_estimator.h>
#include <ekf_estimator.h>
#include <bno085_sensor.h>
#include <otos_sensor.h>
#include <log_ring.h>
#include <telemetry.h>

#if __has_include(<fw_version.h>)
#include <fw_version.h>
#endif
#ifndef FW_GIT_DESCRIBE
#define FW_GIT_DESCRIBE "unknown"
#endif
#ifndef FW_BUILD_DATE
#define FW_BUILD_DATE "unknown"
#endif

// ===== CONSTANTS =====

constexpr uint32_t PERIOD_US = ASSERVISSEMENT_FREQUENCY;
// ODOMETRY every this many ticks (33 Hz at 15 ms)
constexpr uint8_t ODOMETRY_DIVIDER = 2;
constexpr uint32_t STATUS_PERIOD_MS = 200;
// Cost of the loop, sent as text this often
constexpr uint32_t WORK_REPORT_PERIOD_MS = 5000;
// Log frames handed to the link per pass of loop(), at most
constexpr uint16_t DRAIN_MAX_FRAMES = 16;
// Ring of log frames: 511 frames, ~3.8 s of LOG_TICK + PID_TRACE at 66.7 Hz
constexpr uint16_t LOG_SLOTS = 512;

// ===== CONTROL =====

// Output bounds still in rpm, read as mm/s and rad/s: to be set in physical
// units with the complete match firmware.
PID x_pid(KP_X, KI_X, KD_X, -MAX_SPEED_RPM, MAX_SPEED_RPM, 5.0);
PID y_pid(KP_Y, KI_Y, KD_Y, -MAX_SPEED_RPM, MAX_SPEED_RPM, 5.0);
PID theta_pid(KP_THETA, KI_THETA, KD_THETA, -MAX_SPEED_RPM, MAX_SPEED_RPM, 2.0);

Holonomic_Basis* hb = new Holonomic_Basis(ROBOT_RADIUS, WHEEL_DIAMETER, MAX_SPEED_RPM,
                                          x_pid, y_pid, theta_pid);

// ===== POSE ESTIMATORS =====

static pose::ComplementaryParams complementary_params() {
    pose::ComplementaryParams p;
    p.robot_radius_mm = ROBOT_RADIUS;
    p.counts_to_mm = COUNTS_TO_MM;
    // Without it, a base without a working optical sensor never moves its pose.
    p.use_encoders = true;
    return p;
}

static kalman::Geometry ekf_geometry() {
    kalman::Geometry g;
    g.robot_radius_mm = static_cast<float>(ROBOT_RADIUS);
    g.counts_to_mm = static_cast<float>(COUNTS_TO_MM);
    return g;
}

pose::ComplementaryEstimator complementary(complementary_params());
pose::EkfEstimator ekf(ekf_geometry(), kalman::provisional_ekf6_params());

// ===== SENSORS =====

static sensors::OtosConfig otos_config() {
    sensors::OtosConfig c;
    c.linear_scalar = OTOS_LINEAR_SCALAR;
    c.angular_scalar = OTOS_ANGULAR_SCALAR;
    c.mount_angle_rad = OTOS_MOUNT_ANGLE_RAD;
    c.i2c_clock_hz = OTOS_I2C_CLOCK_HZ;
    return c;
}

sensors::Bno085Sensor imu(BNO085_RESET_PIN);
sensors::OtosSensor otos(otos_config());
bool imu_ok = false;
bool otos_ok = false;

// ===== LINK AND TELEMETRY =====

Com* com;
DMAMEM static uint8_t log_storage[LOG_SLOTS * logring::SLOT_BYTES];
logring::LogRing log_ring(log_storage, LOG_SLOTS);
telemetry::Telemetry telem(log_ring, PERIOD_US, static_cast<float>(MKS_COUNTS_PER_REV));

IntervalTimer control_timer;

// ===== STATE SHARED WITH THE TICK =====

// Written in loop() with interrupts off, read by the tick.
Point target(START_X, START_Y, START_THETA);
volatile bool has_target = false;  // the position loop runs
volatile uint16_t cmd_count = 0;   // SET_TARGET accepted since boot
volatile uint32_t cmd_t_us = 0;    // micros() of the last one
volatile uint32_t loop_work_us = 0;  // loop() work since the last tick

// Written by the tick, read in loop() with interrupts off.
msg_odometry odometry_msg;
volatile bool odometry_due = false;

// Tick only.
uint8_t odometry_divider = 0;
uint32_t last_tick_isr_us = 0;

// loop() only.
uint32_t last_message_ms = 0;
bool motors_enabled = false;
bool watchdog_tripped = false;
bool estimator_refused = false;
uint8_t active_estimator = ESTIMATOR_COMPLEMENTARY;
uint16_t last_status_flags = 0xFFFF;
uint32_t last_status_ms = 0;
bool status_now = false;

// ===== COST OF THE CYCLE =====

// Histogram of the work per tick: 250 us bins up to 16 ms, last one open-ended.
constexpr uint32_t WORK_BIN_US = 250;
constexpr uint16_t WORK_BINS = 64;

// Work of each tick: its ISR plus the loop() work it caused (RS485, sensors,
// link). Filled by the tick, read and cleared by loop() with interrupts off.
struct WorkStats {
    uint32_t count = 0;
    uint64_t sum_us = 0;
    uint32_t max_us = 0;
    uint32_t overruns = 0;
    uint16_t bins[WORK_BINS] = {0};

    void add(uint32_t work_us) {
        ++count;
        sum_us += work_us;
        if (work_us > max_us) max_us = work_us;
        if (work_us > PERIOD_US) ++overruns;
        uint32_t bin = work_us / WORK_BIN_US;
        if (bin >= WORK_BINS) bin = WORK_BINS - 1;
        if (bins[bin] < 0xFFFF) ++bins[bin];
    }

    // Upper edge of the bin holding the 99th percentile, us.
    uint32_t p99_us() const {
        const uint32_t rank = count - count / 100;
        uint32_t seen = 0;
        for (uint16_t b = 0; b < WORK_BINS; ++b) {
            seen += bins[b];
            if (seen >= rank) return (b + 1) * WORK_BIN_US;
        }
        return WORK_BINS * WORK_BIN_US;
    }
};
WorkStats work_stats;
uint32_t last_work_report_ms = 0;

// ===== SYNTHETIC ENCODERS =====

#ifdef SYNTHETIC_ENCODERS
// Wheels that follow their command exactly: the counts are the integral of
// the speeds the drivers were sent, with the sign of the real drivers (their
// counts run opposite to the commanded speed).
struct SyntheticWheels {
    double counts[3] = {0.0, 0.0, 0.0};
    uint32_t last_us = 0;
    bool started = false;

    void read(Holonomic_Basis& base) {
        const uint32_t now_us = micros();
        const double dt_s = started ? (now_us - last_us) * 1e-6 : 0.0;
        started = true;
        last_us = now_us;

        noInterrupts();
        const double rpm[3] = {base.filtered_wheel1_rpm, base.filtered_wheel2_rpm,
                               base.filtered_wheel3_rpm};
        interrupts();
        for (int i = 0; i < 3; ++i) {
            counts[i] -= rpm[i] / 60.0 * MKS_COUNTS_PER_REV * dt_s;
        }
        base.inject_encoder_read(llround(counts[0]), llround(counts[1]),
                                 llround(counts[2]));
    }
};
SyntheticWheels synthetic_wheels;
#endif

static void read_encoders() {
#ifdef SYNTHETIC_ENCODERS
    synthetic_wheels.read(*hb);
#else
    hb->read_encoders_nonblocking();
#endif
}

// ===== FRAMES BUILT IN loop() =====

static void push_text(const char* text) {
    byte msg[logring::MAX_FRAME];
    size_t length = strlen(text);
    if (length > sizeof(msg) - 1) length = sizeof(msg) - 1;
    msg[0] = PRINT;
    memcpy(msg + 1, text, length);
    telem.push(msg, static_cast<uint16_t>(length + 1));
}

static void copy_padded(char* field, size_t size, const char* text) {
    memset(field, 0, size);
    strncpy(field, text, size - 1);
}

static void push_fw_info() {
    msg_log_fw_info m;
    copy_padded(m.git_describe, sizeof(m.git_describe), FW_GIT_DESCRIBE);
    copy_padded(m.build_date, sizeof(m.build_date), FW_BUILD_DATE);
    telem.push(&m, sizeof(m));
}

static void push_params() {
    msg_log_params m;
    m.estimator = active_estimator;
    m.period_us = PERIOD_US;

    m.robot_radius_mm = ROBOT_RADIUS;
    // Wheel axes of lib/kinematics: 120, 240 and 0 deg.
    const float wheel_angle_deg[3] = {120.0f, 240.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        m.wheel_diameter_mm[i] = WHEEL_DIAMETER;
        m.wheel_angle_rad[i] = wheel_angle_deg[i] * static_cast<float>(DEG_TO_RAD);
    }
    m.counts_per_rev = MKS_COUNTS_PER_REV;

    const pose::ComplementaryParams cp = complementary_params();
    m.alpha_encoders = cp.alpha_encoders;
    m.encoder_noise_counts = cp.encoder_noise_counts;
    m.imu_max_age_us = cp.imu_max_age_us;
    m.optical_outlier_mm_s = cp.optical_outlier_mm_s;
    m.optical_noise_mm_s = cp.optical_noise_mm_s;
    m.pure_rotation_min_rad = cp.pure_rotation_min_rad;
    m.pure_rotation_min_counts = cp.pure_rotation_min_counts;
    m.pure_rotation_tolerance = cp.pure_rotation_tolerance;
    m.use_encoders = cp.use_encoders ? 1 : 0;

    m.otos_linear_scalar = OTOS_LINEAR_SCALAR;
    m.otos_angular_scalar = OTOS_ANGULAR_SCALAR;
    // No software scale applied after the read yet.
    m.otos_linear_scale_soft = 1.0f;
    m.otos_angular_scale_soft = 1.0f;
    m.otos_mount_angle_rad = OTOS_MOUNT_ANGLE_RAD;

    const PID* pids[3] = {&hb->x_pid, &hb->y_pid, &hb->theta_pid};
    for (int i = 0; i < 3; ++i) {
        m.kp[i] = pids[i]->kp();
        m.ki[i] = pids[i]->ki();
        m.kd[i] = pids[i]->kd();
    }

    m.max_wheel_rpm = MAX_SPEED_RPM;
    m.max_wheel_accel_rpm_s = MAX_WHEEL_ACCEL_RPM_S;
    telem.push(&m, sizeof(m));
}

// ===== CONTROL TICK (ISR) =====

static void fill_snapshot(telemetry::TickSnapshot& s, uint32_t t_us, bool controlling,
                          uint32_t work_us) {
    s.t_us = t_us;
    s.input = hb->last_input();
    s.estimate = hb->last_estimate();
    s.est_health = hb->last_estimator_health();
    s.est_flags = hb->last_step_flags();

    const Holonomic_Basis::EncoderRead& er = hb->last_encoder_read();
    for (int i = 0; i < 3; ++i) {
        s.enc_counts[i] = er.counts[i];
        s.rs485_fail[i] = er.fail[i];
    }
    s.enc_t_us_mid = er.t_us_mid;
    s.enc_read_ok = er.ok_mask;

    if (otos_ok) {
        const sensors::OtosRaw raw = otos.raw();
        for (int i = 0; i < 3; ++i) {
            s.otos_pos[i] = raw.pos[i];
            s.otos_vel[i] = raw.vel[i];
            s.otos_acc[i] = raw.acc[i];
        }
        s.otos_read_us = raw.read_us;
    }
    if (imu_ok) {
        const sensors::HeadingRaw raw = imu.raw();
        s.imu_raw_yaw_rad = raw.raw_yaw_rad;
        s.imu_yaw_offset_rad = raw.yaw_offset_rad;
        s.imu_accuracy = raw.accuracy;
    }

    s.cmd_count = cmd_count;
    s.cmd_t_us = cmd_t_us;
    s.work_us = work_us;
    s.write_failed = hb->last_write_failed();

    s.controlling = controlling;
    const Holonomic_Basis::ControlRecord& c = hb->last_control();
    for (int i = 0; i < 3; ++i) {
        s.target[i] = static_cast<float>(c.target[i]);
        s.velocity_cmd[i] = static_cast<float>(c.velocity_cmd[i]);
        s.p_term[i] = static_cast<float>(c.p[i]);
        s.i_term[i] = static_cast<float>(c.i[i]);
        s.d_term[i] = static_cast<float>(c.d[i]);
    }
    const motion::ShapedCommand shaped = hb->last_command();
    for (int i = 0; i < 3; ++i) {
        s.wheel_requested[i] = static_cast<float>(shaped.requested_rpm[i]);
        s.wheel_cmd[i] = static_cast<float>(shaped.wheel_rpm[i]);
    }
    uint8_t flags = 0;
    if (shaped.braking) flags |= TRACE_BRAKING;
    if (shaped.speed_saturated) flags |= TRACE_SPEED_SATURATED;
    if (shaped.accel_limited) flags |= TRACE_ACCEL_LIMITED;
    if (controlling && c.integral_frozen) flags |= TRACE_INTEGRAL_FROZEN;
    s.trace_flags = flags;
}

void control_tick() {
    const uint32_t t0 = micros();

    hb->update_odometry();
    const bool controlling = has_target;
    if (controlling) {
        hb->handle(target, nullptr);
        hb->need_send_movement = true;
    }
    hb->need_read_encoders = true;

    if (++odometry_divider >= ODOMETRY_DIVIDER) {
        odometry_divider = 0;
        const pose::PoseEstimate& p = hb->last_estimate();
        odometry_msg.x = p.x_mm;
        odometry_msg.y = p.y_mm;
        odometry_msg.theta = p.theta_rad;
        odometry_msg.vx = p.vx_mm_s;
        odometry_msg.vy = p.vy_mm_s;
        odometry_msg.omega = p.omega_rad_s;
        odometry_msg.t_us = t0;
        odometry_due = true;
    }

    // Work of the previous tick: its ISR and what loop() did for it since.
    const uint32_t work_us = last_tick_isr_us + loop_work_us;
    loop_work_us = 0;
    work_stats.add(work_us);

    if (telem.start_tick(t0)) {
        telemetry::TickSnapshot s{};
        fill_snapshot(s, t0, controlling, work_us);
        telem.emit_tick(s);
    }

    last_tick_isr_us = micros() - t0;
}

// ===== USB MESSAGES (loop) =====

static void message_received() {
    last_message_ms = millis();
}

// Stops the base where it is; the next SET_TARGET starts it again.
static void stop_base() {
    has_target = false;
    hb->emergency_stop();
}

static void on_set_target(byte* msg, byte size) {
    if (size != sizeof(msg_set_target)) return;
    msg_set_target m;
    memcpy(&m, msg, sizeof(m));
    message_received();

    // Outside the critical section: it has its own.
    hb->set_velocity_feedforward(m.vx, m.vy, m.omega);
    noInterrupts();
    target.x = m.x;
    target.y = m.y;
    target.theta = m.theta;
    has_target = true;
    cmd_count = cmd_count + 1;
    cmd_t_us = micros();
    interrupts();
    watchdog_tripped = false;
}

static void on_set_pid(byte* msg, byte size) {
    if (size != sizeof(msg_set_pid)) return;
    msg_set_pid m;
    memcpy(&m, msg, sizeof(m));
    message_received();

    PID* pid = nullptr;
    switch (m.pid_type) {
        case X_PID_ID: pid = &hb->x_pid; break;
        case Y_PID_ID: pid = &hb->y_pid; break;
        case THETA_PID_ID: pid = &hb->theta_pid; break;
        default: return;
    }
    // The tick runs the PIDs: never let it see half the gains.
    noInterrupts();
    pid->updateParameters(m.kp, m.ki, m.kd);
    interrupts();
    push_params();
}

static void on_set_odometrie(byte* msg, byte size) {
    if (size != sizeof(msg_set_odometrie)) return;
    msg_set_odometrie m;
    memcpy(&m, msg, sizeof(m));
    message_received();

    // At rest first, then the new pose: the estimator restarts from rest.
    stop_base();
    hb->init_holonomic_basis(m.x, m.y, m.theta);
    noInterrupts();
    target.x = m.x;
    target.y = m.y;
    target.theta = m.theta;
    interrupts();
    telem.event(LOG_EVENT_POSE_SET, static_cast<float>(m.x), static_cast<float>(m.y),
                static_cast<float>(m.theta));
}

static void on_stop(byte* msg, byte size) {
    (void)msg;
    if (size != sizeof(msg_stop)) return;
    message_received();
    stop_base();
    telem.event(LOG_EVENT_STOP);
}

static void on_set_estimator(byte* msg, byte size) {
    if (size != sizeof(msg_set_estimator)) return;
    msg_set_estimator m;
    memcpy(&m, msg, sizeof(m));
    message_received();

    pose::PoseEstimator* next = nullptr;
    if (m.estimator == ESTIMATOR_COMPLEMENTARY) next = &complementary;
    if (m.estimator == ESTIMATOR_EKF6) next = &ekf;
    // Refused while the wheels turn, and for an unknown estimator.
    const bool accepted = next != nullptr && hb->select_estimator(next);
    estimator_refused = !accepted;
    status_now = true;
    if (!accepted) return;
    active_estimator = m.estimator;
    telem.event(LOG_EVENT_ESTIMATOR_SELECTED, 0.0f, 0.0f, 0.0f, m.estimator);
    push_params();
}

static void on_set_trace(byte* msg, byte size) {
    if (size != sizeof(msg_set_trace)) return;
    msg_set_trace m;
    memcpy(&m, msg, sizeof(m));
    message_received();

    telem.set_mask(m.mask & (TRACE_MASK_PID | TRACE_MASK_LOG_TICK));
    // The host often opens the port after the boot of the board, and the boot
    // frames are lost: every SET_TRACE gets them again.
    push_fw_info();
    push_params();
    status_now = true;
}

static void on_heartbeat(byte* msg, byte size) {
    (void)msg;
    if (size != sizeof(msg_heartbeat)) return;
    message_received();
}

static void on_reset_teensy(byte* msg, byte size) {
    (void)msg;
    (void)size;
    SCB_AIRCR = 0x05FA0004;  // system reset request
}

void (*callbacks[256])(byte* msg, byte size);

static void init_callbacks() {
    callbacks[SET_TARGET] = &on_set_target;
    callbacks[SET_PID] = &on_set_pid;
    callbacks[SET_ODOMETRIE] = &on_set_odometrie;
    callbacks[STOP] = &on_stop;
    callbacks[SET_ESTIMATOR] = &on_set_estimator;
    callbacks[SET_TRACE] = &on_set_trace;
    callbacks[HEARTBEAT] = &on_heartbeat;
    callbacks[RESET_TEENSY] = &on_reset_teensy;
}

// ===== PERIODIC MESSAGES (loop) =====

static void check_watchdog() {
    if (!has_target) return;
    if (millis() - last_message_ms <= WATCHDOG_TIMEOUT_MS) return;
    stop_base();
    watchdog_tripped = true;
    telem.event(LOG_EVENT_WATCHDOG);
}

// ODOMETRY and STATUS go out with send_log() too: a periodic message is
// replaced by the next one, it must never block loop() on a host that stopped
// reading, nor be sent again on a NACK.
static bool send_odometry() {
    if (!odometry_due) return false;
    noInterrupts();
    const msg_odometry m = odometry_msg;
    odometry_due = false;
    interrupts();
    com->send_log(reinterpret_cast<const byte*>(&m), sizeof(m));
    return true;
}

static uint16_t status_flags() {
    uint16_t flags = 0;
    if (motors_enabled) flags |= STATUS_MOTORS_ENABLED;
    if (has_target) flags |= STATUS_MOVING;
    if (watchdog_tripped) flags |= STATUS_WATCHDOG_TRIPPED;
    if (imu_ok) flags |= STATUS_IMU_OK;
    if (otos_ok) flags |= STATUS_OPTICAL_OK;
    if (estimator_refused) flags |= STATUS_ESTIMATOR_REFUSED;
    const uint8_t mask = telem.mask();
    if (mask & TRACE_MASK_PID) flags |= STATUS_TRACE_ON;
    if (mask & TRACE_MASK_LOG_TICK) flags |= STATUS_LOG_TICK_ON;
    return flags;
}

static bool send_status() {
    const uint32_t now_ms = millis();
    const uint16_t flags = status_flags();
    if (!status_now && flags == last_status_flags &&
        now_ms - last_status_ms < STATUS_PERIOD_MS) {
        return false;
    }
    msg_status m;
    m.t_us = micros();
    m.flags = flags;
    m.estimator = active_estimator;
    m.estimator_health = hb->active_estimator()->health_flags();
    m.rs485_read_failures = hb->rs485_read_failures();
    m.rs485_write_failures = hb->rs485_write_failures();
    if (com->send_log(reinterpret_cast<const byte*>(&m), sizeof(m))) {
        last_status_flags = flags;
        last_status_ms = now_ms;
        status_now = false;
    }
    return true;
}

static void report_work() {
    const uint32_t now_ms = millis();
    if (now_ms - last_work_report_ms < WORK_REPORT_PERIOD_MS) return;
    last_work_report_ms = now_ms;

    noInterrupts();
    const WorkStats stats = work_stats;
    work_stats = WorkStats();
    interrupts();
    if (stats.count == 0) return;

    char text[120];
    snprintf(text, sizeof(text),
             "work_us %lu ticks: mean %lu, p99 <= %lu, max %lu, overruns %lu, mask %u",
             static_cast<unsigned long>(stats.count),
             static_cast<unsigned long>(stats.sum_us / stats.count),
             static_cast<unsigned long>(stats.p99_us()),
             static_cast<unsigned long>(stats.max_us),
             static_cast<unsigned long>(stats.overruns),
             static_cast<unsigned>(telem.mask()));
    push_text(text);
}

// ===== SETUP / LOOP =====

void setup() {
    com = new Com(&Serial, BAUDRATE);
    init_callbacks();

    hb->define_wheel1(W1_SERIAL, W1_ADDR);
    hb->define_wheel2(W2_SERIAL, W2_ADDR);
    hb->define_wheel3(W3_SERIAL, W3_ADDR);
    hb->init_motors();

    // Robot still during begin(): the OTOS calibrates its IMU.
    imu_ok = imu.begin(IMU_REPORT_PERIOD_US);
    otos_ok = otos.begin(OTOS_WIRE);
    hb->attach_sensors(imu_ok ? &imu : nullptr, otos_ok ? &otos : nullptr);
    hb->use_imu = imu_ok;
    hb->use_optical_flow = otos_ok;

#if POSE_ESTIMATOR_DEFAULT == POSE_ESTIMATOR_EKF6
    hb->set_estimator(&ekf);
    active_estimator = ESTIMATOR_EKF6;
#else
    hb->set_estimator(&complementary);
    active_estimator = ESTIMATOR_COMPLEMENTARY;
#endif
    hb->init_holonomic_basis(START_X, START_Y, START_THETA);

    hb->enable_motors();
    motors_enabled = true;

    // A first encoder read before the tick starts: its first frame takes it
    // as the reference of the deltas.
    read_encoders();

    // Boot frames, kept in the ring until the host reads them.
    telem.event(LOG_EVENT_BOOT, 0.0f, 0.0f, 0.0f,
                (imu_ok ? 1u : 0u) | (otos_ok ? 2u : 0u));
    push_fw_info();
    push_params();

    last_message_ms = millis();
    control_timer.begin(control_tick, PERIOD_US);
}

void loop() {
    com->handle_callback(callbacks);
    check_watchdog();

    // Work caused by the tick, timed for its cost.
    const uint32_t start_us = micros();
    bool worked = false;

    if (hb->need_read_encoders) {
        hb->need_read_encoders = false;
        read_encoders();
        worked = true;
    }
    if (hb->need_send_movement) {
        hb->need_send_movement = false;
        hb->send_movement_commands_nonblocking();
        worked = true;
    }
    if (hb->need_read_imu) {
        hb->need_read_imu = false;
        imu.poll();
        worked = true;
    }
    if (hb->need_read_optical) {
        hb->need_read_optical = false;
        otos.poll();
        worked = true;
    }
    worked |= send_odometry();
    worked |= send_status();
    worked |= telem.drain(*com, DRAIN_MAX_FRAMES) > 0;

    if (worked) {
        const uint32_t spent_us = micros() - start_us;
        noInterrupts();
        loop_work_us = loop_work_us + spent_us;
        interrupts();
    }

    report_work();
}
