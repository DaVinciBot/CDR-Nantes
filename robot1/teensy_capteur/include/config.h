/*
 * Configuration of the Teensy Capteur (sensor board): constants and pinout.
 *
 * This board runs the EKF. It reads the optical odometry sensor and the IMU,
 * both on I2C, and it talks to nothing but the Teensy Moteur.
 *
 * It has no wheel, no PID and no USB protocol. If you find yourself adding a
 * gain or a wheel diameter that is not the two geometry constants below, it
 * belongs on the other board.
 */

#ifndef CONFIG_H
#define CONFIG_H

// === INTER-TEENSY LINK ===
// Frame layout: see lib/teensy_link/include/teensy_link.h.
// On a Teensy 4.1, Serial4 is RX = 16, TX = 17. Cross the pair and share ground.

#define LINK_SERIAL Serial4
#define LINK_BAUD 2000000
#define LINK_RX_BUFFER_SIZE 512

// === I2C SENSOR BUS ===
// The Qwiic OTOS and the BNO085 share Wire (Wire0). Fast Mode.

#define PIN_SDA 18
#define PIN_SCL 19
#define I2C_CLOCK_HZ 400000

// BNO085 reset line. -1 when it is tied to the Teensy reset or unused.
#define BNO085_RESET_PIN -1

// === OPTICAL SENSOR MOUNTING ===
// Lever arm from the centre of rotation to the optical sensor, in mm.
//
// The mechanical target is the sensor AT the centre of rotation, so both default
// to 0. The fields are kept anyway, and on purpose: a residual 15 mm offset
// produces 45 mm/s at 3 rad/s, which the filter reads as real velocity. It is a
// systematic bias, not noise, so the EKF does not absorb it.

#define OPTICAL_LEVER_X_MM 0.0f
#define OPTICAL_LEVER_Y_MM 0.0f

// === ENCODER NOISE THRESHOLD ===
// Expressed in RAW COUNTS, never in millimetres.
//
// This is the fix for the most expensive bug of the 2026 code. A 2 mm per 10 ms
// cycle threshold silences every measurement below 200 mm/s, while the robot is
// capped at 200 mm/s: the usable band was empty except exactly at saturation.
// At COUNTS_TO_MM = 0.011507, one millimetre is about 87 counts, so 16 counts is
// roughly 0.18 mm per cycle, i.e. 18 mm/s.

#define ENCODER_NOISE_THRESHOLD_COUNTS 16

// === ROBOT GEOMETRY ===
// WARNING: these two constants are DUPLICATED from
// robot1/teensy_moteur/include/config.h, which is the authority. The sensor
// board needs them for the forward kinematics that turns three wheel count
// deltas into a body-frame displacement.
//
// Never edit them here alone. The repository already carries two independent
// homonyms of ROBOT_RADIUS (160.0 mm here and in the motor config, a physical
// quantity; 120 mm as ROBOT_RADIUS_MM in robot1/rasp/world.py, an A* inflation
// radius that must NEVER be aligned with it). Do not make this a third.
//
// Unifying them in common/ is a decision for the November firmware rewrite.

#define ROBOT_RADIUS 160.0f   // mm - centre to wheel axis
#define WHEEL_DIAMETER 60.0f  // mm - effective diameter
#define MKS_COUNTS_PER_REV 16384.0f
#define COUNTS_TO_MM ((WHEEL_DIAMETER * 3.14159265358979f) / MKS_COUNTS_PER_REV)

// === CONTROL LOOP ===
// The Teensy Moteur is the time master: this board computes only when an
// EncoderFrame arrives, and uses the dt_us that the frame carries. This period
// is therefore only the housekeeping tick (link watchdog, counters, debug), not
// the filter rate.

#define HOUSEKEEPING_PERIOD_US 10000  // 10 ms = 100 Hz

// === DEBUG ===
// USB serial, development only. This board has no production USB protocol.

#define DEBUG_BAUD 115200
#define DEBUG_PERIOD_MS 500

#endif  // CONFIG_H
