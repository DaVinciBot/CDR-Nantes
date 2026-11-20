#ifndef MESSAGES_H
#define MESSAGES_H

#include <Arduino.h>

// ====== Frame ======
// On the wire, every message is COBS-encoded and ends with FRAME_DELIMITER:
//
//     COBS( [ID][data][CRC8] ) 0x00
//
// COBS removes every 0x00 from the encoded bytes, so the delimiter can never
// appear inside a frame, whatever the payload holds. The CRC8 covers ID + data.
// Must stay byte-for-byte identical to common/usb_com/python/com/framing.py.
const byte FRAME_DELIMITER = 0x00;

// ID + data of one message, at most. The CRC adds one byte, COBS at most two.
const uint16_t COM_MAX_PAYLOAD = 255;

// ====== Message Types ======
/* Definition of message IDs */
// rasp -> teensy : 0-127 (Convention)

// Rolling Basis
#define SET_TARGET 0
#define SET_PID 1
#define SET_ODOMETRIE 2

// Actuators
#define SET_SERVO_ANGLE_I2C 3
#define STEPPER_STEP 4
#define SET_SERVO_ANGLE_DETACH 5
#define ATTACH_SWITCH 6
#define SET_SERVO_ANGLE 7
#define SET_STEPPER_DRIVER_ACTIVATION_STATE 8

// Rolling Basis
#define STOP 9
#define SET_ESTIMATOR 10
#define SET_TRACE 11
#define HEARTBEAT 12

// Common (Rolling Basis + Actuators)
#define RESET_TEENSY 126

// two ways : 127 (Convention)
#define NACK 127

// teensy -> rasp : 128-255 (Convention)
// Rolling Basis
#define ODOMETRY 128

// Actuators
#define SWITCH_STATE_RETURN 129

// Rolling Basis
#define STATUS 130
#define PID_TRACE 131

// Rolling Basis, logs: what a replay or a calibration needs off the robot
#define LOG_TICK 200
#define LOG_EVENT 201
#define LOG_FW_INFO 202
#define LOG_PARAMS 203

// Common (Rolling Basis + Actuators)
#define PRINT 254
#define UNKNOWN_MSG_TYPE 255

// ====== Values carried by the messages ======

// msg_set_estimator::estimator (same values as POSE_ESTIMATOR_* on the board)
#define ESTIMATOR_COMPLEMENTARY 0
#define ESTIMATOR_EKF6 1

// msg_set_trace::mask: the per-tick messages to send. Both are off at boot.
// The value 1 keeps the meaning it had when the field was a plain on / off.
#define TRACE_MASK_PID (1u << 0)       // PID_TRACE
#define TRACE_MASK_LOG_TICK (1u << 1)  // LOG_TICK

// msg_status::flags
#define STATUS_MOTORS_ENABLED (1u << 0)
#define STATUS_MOVING (1u << 1)
#define STATUS_WATCHDOG_TRIPPED (1u << 2)  // no message for WATCHDOG_TIMEOUT_MS
#define STATUS_TIMEOUT_TRIPPED (1u << 3)   // movement took too long
#define STATUS_IMU_OK (1u << 4)            // BNO085 started
#define STATUS_OPTICAL_OK (1u << 5)        // OTOS started
#define STATUS_ESTIMATOR_REFUSED (1u << 6) // last SET_ESTIMATOR refused (moving)
#define STATUS_TRACE_ON (1u << 7)          // PID_TRACE is being sent
// Bits 8 and 9 are kept for the "base blocked" and "drivers unpowered" flags.
#define STATUS_LOG_TICK_ON (1u << 10)      // LOG_TICK is being sent

// msg_pid_trace::flags (the command shaper's, plus the anti-windup)
#define TRACE_BRAKING (1u << 0)
#define TRACE_SPEED_SATURATED (1u << 1)
#define TRACE_ACCEL_LIMITED (1u << 2)
#define TRACE_INTEGRAL_FROZEN (1u << 3)

// Layout version of the LOG_* messages, carried by each of them. Any change of
// a LOG_* struct bumps it: a log file is decoded with the layout it was
// recorded with.
#define LOG_SCHEMA_VERSION 1

// msg_log_tick::input_flags: the booleans of the estimator input
#define LOG_IN_FIRST_FRAME (1u << 0)
#define LOG_IN_ENCODERS_FRESH (1u << 1)
#define LOG_IN_IMU_VALID (1u << 2)
#define LOG_IN_OPTICAL_STEP_VALID (1u << 3)
#define LOG_IN_OPTICAL_VELOCITY_VALID (1u << 4)

// msg_log_tick::est_flags: what the estimator did with the optical sample
#define LOG_EST_OPTICAL_USED (1u << 0)
#define LOG_EST_OPTICAL_REJECTED (1u << 1)
#define LOG_EST_PURE_ROTATION (1u << 2)

// msg_log_tick::health_flags
#define LOG_HEALTH_OVERRUN (1u << 0)       // the work of the tick overran the period
#define LOG_HEALTH_OPTICAL_READ (1u << 1)  // a new optical sample this tick
#define LOG_HEALTH_IMU_READ (1u << 2)      // a new IMU sample this tick
#define LOG_HEALTH_WRITE_FAILED (1u << 3)  // wheel speeds not sent this tick

// msg_log_tick::rs485_fail[i]: why the encoder read of wheel i + 1 failed
#define RS485_READ_OK 0
#define RS485_NO_ANSWER 1
#define RS485_BAD_CHECKSUM 2
#define RS485_SHORT_FRAME 3
#define RS485_BAD_ADDRESS 4

// msg_log_event::event
#define LOG_EVENT_BOOT 0
#define LOG_EVENT_ESTIMATOR_SELECTED 1  // arg = ESTIMATOR_*
#define LOG_EVENT_POSE_SET 2            // value = x, y, theta imposed on the estimator
#define LOG_EVENT_WATCHDOG 3
#define LOG_EVENT_STOP 4
#define LOG_EVENT_BLOCKED 5
#define LOG_EVENT_DRIVERS_UNPOWERED 6   // arg = 1 when lost, 0 when back
#define LOG_EVENT_ERROR 7               // arg = error code

/* Definition of the messages content. Units: mm, rad, s, us, rpm. */

// No padding between the fields: the byte layout IS the protocol. Scoped with
// push / pop: a bare pack(1) would leak into every header included after this
// one, and a struct packed in one file but not in another has two layouts.
#pragma pack(push, 1)

// rasp -> teensy : 0-127

// Rolling Basis
struct msg_set_target {
    byte command = SET_TARGET;
    // Target pose, table frame
    float x;
    float y;
    float theta;
    // Velocity of the trajectory at the target, table frame; zero for a plain
    // pose target (feed-forward, added after the braking cap)
    float vx;
    float vy;
    float omega;
};

struct msg_set_pid {
    byte command = SET_PID;
    byte pid_type;
    float kp;
    float ki;
    float kd;
};

struct msg_set_odometrie {
    byte command = SET_ODOMETRIE;
    double x;
    double y;
    double theta;
};

// Immediate stop, back to idle (not "go to the current position")
struct msg_stop {
    byte command = STOP;
};

// Pose estimator to run; refused while the wheels turn (see msg_status)
struct msg_set_estimator {
    byte command = SET_ESTIMATOR;
    byte estimator;  // ESTIMATOR_*
};

// Per-tick messages to send: PID_TRACE and LOG_TICK, one bit each. Both are off
// at boot and cost nothing while off; what a run needs is its sender's choice.
struct msg_set_trace {
    byte command = SET_TRACE;
    byte mask;  // TRACE_MASK_*
};

// Keeps the watchdog fed when there is nothing else to send
struct msg_heartbeat {
    byte command = HEARTBEAT;
};

// Actuators
struct msg_set_servo_angle {
    byte command = SET_SERVO_ANGLE;
    byte pin;            // pin to which the servo is connected
    uint16_t angle;      // angle to which the servo should be moved in degrees
    uint16_t max_angle;  // max angle reachable by the servo ex : 180, 270
};

struct msg_set_servo_angle_I2C {
    byte command = SET_SERVO_ANGLE_I2C;
    byte pin;            // pin to which the servo is connected
    uint16_t angle;      // angle to which the servo should be moved in degrees
    uint16_t max_angle;  // max angle reachable by the servo ex : 180, 270
};

struct msg_set_servo_angle_detach {
    byte command = SET_SERVO_ANGLE_DETACH;
    byte pin;            // pin to which the servo is connected
    uint16_t angle;      // angle to which the servo should be moved in degrees
    uint16_t max_angle;  // max angle reachable by the servo ex : 180, 270
    int detach_delay;    // delay in milliseconds before detaching the servo
};

struct msg_stepper_step {
    byte command = STEPPER_STEP;
    int steps;               // total number of steps this motor can take
    bool dir;                // direction of the motor
    int speed;               // speed
    byte pin_dir;            // pin to which the direction pin is connected
    byte pin_step;           // pin to which the step pin is connected
    byte enable_pin_driver;  // pin to which the driver pin is connected
};

struct msg_set_stepper_driver_activation_state {
    byte command = SET_STEPPER_DRIVER_ACTIVATION_STATE;
    byte pin;  // stepper driver's enable pin
    bool
        enable_driver_state;  // active low so true means the driver is disabled
};

struct msg_attach_switch {
    byte command = ATTACH_SWITCH;
    byte pin;  // pin to which the servo is connected
};

// Common (Rolling Basis + Actuators)
struct msg_reset_teensy {
    byte command = RESET_TEENSY;
};

// teensy -> rasp : 128-255

// Rolling Basis
struct msg_odometry {
    byte command = ODOMETRY;
    // Pose and velocity, table frame
    float x;
    float y;
    float theta;
    float vx;
    float vy;
    float omega;
    uint32_t t_us;  // board time of the estimate (micros(), wraps)
};

struct msg_status {
    byte command = STATUS;
    uint32_t t_us;
    uint16_t flags;            // STATUS_*
    byte estimator;            // ESTIMATOR_* running
    byte estimator_health;     // the estimator's health flags
    uint32_t rs485_read_failures;   // encoder reads unanswered, since boot
    uint32_t rs485_write_failures;  // speed commands not sent, since boot
};

// One line per control tick, while enabled by SET_TRACE: what a PID tuning
// needs (step responses, which term dominates, whether the limiter caps it,
// whether the drivers follow).
struct msg_pid_trace {
    byte command = PID_TRACE;
    uint32_t t_us;
    float target[3];          // x, y, theta
    float pose[3];            // x, y, theta
    float velocity_cmd[3];    // PID output + feed-forward, table frame
    float wheel_requested[3]; // rpm, before the acceleration limit
    float wheel_cmd[3];       // rpm, sent to the drivers
    float wheel_measured[3];  // rpm, from the encoders
    float p_term[3];          // x, y, theta
    float i_term[3];
    float d_term[3];
    byte flags;               // TRACE_*
};

// Rolling Basis, logs

// Head of LOG_TICK and LOG_EVENT.
struct log_header {
    uint8_t schema_version = LOG_SCHEMA_VERSION;
    uint16_t seq;   // +1 per message of this type: a gap is a loss
    uint32_t tick;  // control ticks since boot
    uint32_t t_us;  // micros() at the start of the tick (wraps)
};

// One line per control tick, while enabled by SET_TRACE: the raw sensors, the
// exact input of the estimator and its output. The input is logged as it was
// handed over, not rebuilt from the raw values: an off-board replay of the
// estimator must be fed the very same numbers. Raw values are uncorrected
// (no mounting rotation, no software scale), so a calibration can be redone.
struct msg_log_tick {
    byte command = LOG_TICK;
    log_header header;

    // --- Estimator input of the tick ---
    int32_t d[3];                  // encoder count deltas of wheels 1, 2, 3
    uint32_t dt_us;                // time since the previous tick
    uint32_t encoder_age_us;       // age of the last successful encoder read
    float heading_rad;             // IMU heading, table frame
    uint32_t imu_timestamp_us;
    uint32_t imu_age_us;
    uint32_t optical_timestamp_us;
    float optical_dx_mm;           // robot-frame step since the previous sample
    float optical_dy_mm;
    uint32_t optical_step_dt_us;   // time that step covers
    float optical_vx_mm_s;         // robot-frame velocity
    float optical_vy_mm_s;
    uint8_t input_flags;           // LOG_IN_*

    // --- Raw encoders ---
    int64_t enc_counts[3];         // absolute counts of wheels 1, 2, 3
    uint32_t enc_t_us_mid;         // middle of the read
    uint8_t enc_read_ok;           // bit i set: wheel i + 1 answered this tick

    // --- Raw optical sensor, sensor axes ---
    float otos_pos[3];             // x, y (mm), h (rad), since its last reset
    float otos_vel[3];             // mm/s, mm/s, rad/s
    float otos_acc[3];             // mm/s^2, mm/s^2, rad/s^2

    // --- Raw IMU ---
    float imu_raw_yaw_rad;         // yaw as the sensor gives it
    float imu_yaw_offset_rad;      // heading_rad = raw yaw - offset, wrapped
    uint8_t imu_accuracy;          // accuracy status of the report, 0 to 3

    // --- Estimator output ---
    float est_pose[3];             // x, y (mm), theta (rad), table frame
    float est_vel[3];              // mm/s, mm/s, rad/s, table frame
    uint8_t est_health;            // the estimator's health flags
    uint8_t est_flags;             // LOG_EST_*

    // --- Commands ---
    uint16_t cmd_count;            // SET_TARGET accepted since boot (wraps)
    uint32_t cmd_t_us;             // micros() when the last one was taken

    // --- Cycle health ---
    uint16_t work_us;              // work done in this tick
    uint8_t health_flags;          // LOG_HEALTH_*
    uint8_t rs485_fail[3];         // RS485_* of wheels 1, 2, 3 this tick
    uint16_t log_dropped;          // log frames dropped since boot (wraps)
    uint16_t otos_read_us;         // duration of the optical read
};

// Something happened: sent once, whatever SET_TRACE says.
struct msg_log_event {
    byte command = LOG_EVENT;
    log_header header;
    uint8_t event;   // LOG_EVENT_*
    float value[3];  // meaning set by the event
    uint32_t arg;    // meaning set by the event
};

// Which firmware produced a log. Sent at boot.
struct msg_log_fw_info {
    byte command = LOG_FW_INFO;
    uint8_t schema_version = LOG_SCHEMA_VERSION;
    char git_describe[24];  // NUL-padded
    char build_date[20];    // "YYYY-MM-DD HH:MM:SS", NUL-padded
};

// Every constant a replay or a calibration depends on. Sent at boot.
struct msg_log_params {
    byte command = LOG_PARAMS;
    uint8_t schema_version = LOG_SCHEMA_VERSION;
    uint8_t estimator;            // ESTIMATOR_* running
    uint32_t period_us;           // control period

    // Geometry
    float robot_radius_mm;        // centre to wheel contact
    float wheel_diameter_mm[3];
    float wheel_angle_rad[3];
    float counts_per_rev;

    // Complementary filter
    float alpha_encoders;
    float encoder_noise_counts;
    uint32_t imu_max_age_us;
    float optical_outlier_mm_s;
    float optical_noise_mm_s;
    float pure_rotation_min_rad;
    float pure_rotation_min_counts;
    float pure_rotation_tolerance;
    uint8_t use_encoders;

    // Optical sensor: scalars written in the sensor, scales applied in
    // software after the read, mounting angle
    float otos_linear_scalar;
    float otos_angular_scalar;
    float otos_linear_scale_soft;
    float otos_angular_scale_soft;
    float otos_mount_angle_rad;

    // Position PIDs: x, y, theta
    float kp[3];
    float ki[3];
    float kd[3];

    // Command limits
    float max_wheel_rpm;
    float max_wheel_accel_rpm_s;
};

// Actuators
struct msg_switch_state_return {
    byte command = SWITCH_STATE_RETURN;
    byte pin;    // pin of the switch
    bool state;  // switch state
};

// Common (Rolling Basis + Actuators)
struct msg_unknown_msg_type {
    byte command = UNKNOWN_MSG_TYPE;
    byte type_id;  // ID of the unknown message
};

#pragma pack(pop)

// The byte layout is the protocol: a size that moves is a protocol change, and
// the Python side (PAYLOAD_FORMATS) must move with it. Checked at every build.
static_assert(sizeof(msg_set_target) == 25, "msg_set_target layout");
static_assert(sizeof(msg_set_pid) == 14, "msg_set_pid layout");
static_assert(sizeof(msg_set_odometrie) == 25, "msg_set_odometrie layout");
static_assert(sizeof(msg_stop) == 1, "msg_stop layout");
static_assert(sizeof(msg_set_estimator) == 2, "msg_set_estimator layout");
static_assert(sizeof(msg_set_trace) == 2, "msg_set_trace layout");
static_assert(sizeof(msg_heartbeat) == 1, "msg_heartbeat layout");
static_assert(sizeof(msg_odometry) == 29, "msg_odometry layout");
static_assert(sizeof(msg_status) == 17, "msg_status layout");
static_assert(sizeof(msg_pid_trace) == 114, "msg_pid_trace layout");
static_assert(sizeof(msg_log_tick) == 185, "msg_log_tick layout");
static_assert(sizeof(msg_log_event) == 29, "msg_log_event layout");
static_assert(sizeof(msg_log_fw_info) == 46, "msg_log_fw_info layout");
static_assert(sizeof(msg_log_params) == 136, "msg_log_params layout");
static_assert(sizeof(msg_log_tick) <= COM_MAX_PAYLOAD, "msg_log_tick fits in a frame");

#endif // MESSAGES_H
