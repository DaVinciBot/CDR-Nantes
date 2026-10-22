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

// Common (Rolling Basis + Actuators)
#define PRINT 254
#define UNKNOWN_MSG_TYPE 255

// ====== Values carried by the messages ======

// msg_set_estimator::estimator (same values as POSE_ESTIMATOR_* on the board)
#define ESTIMATOR_COMPLEMENTARY 0
#define ESTIMATOR_EKF6 1

// msg_status::flags
#define STATUS_MOTORS_ENABLED (1u << 0)
#define STATUS_MOVING (1u << 1)
#define STATUS_WATCHDOG_TRIPPED (1u << 2)  // no message for WATCHDOG_TIMEOUT_MS
#define STATUS_TIMEOUT_TRIPPED (1u << 3)   // movement took too long
#define STATUS_IMU_OK (1u << 4)            // BNO085 started
#define STATUS_OPTICAL_OK (1u << 5)        // OTOS started
#define STATUS_ESTIMATOR_REFUSED (1u << 6) // last SET_ESTIMATOR refused (moving)
#define STATUS_TRACE_ON (1u << 7)

// msg_pid_trace::flags (the command shaper's, plus the anti-windup)
#define TRACE_BRAKING (1u << 0)
#define TRACE_SPEED_SATURATED (1u << 1)
#define TRACE_ACCEL_LIMITED (1u << 2)
#define TRACE_INTEGRAL_FROZEN (1u << 3)

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

// PID_TRACE on (1) or off (0). Off in a match.
struct msg_set_trace {
    byte command = SET_TRACE;
    byte enabled;
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

#endif // MESSAGES_H
