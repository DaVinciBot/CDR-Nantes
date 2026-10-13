// Test firmware configuration: Teensy 4.1 + SparkFun Qwiic OTOS only.
// Every setting is applied at boot: edit, rebuild, upload.

#pragma once

// === I2C BUS ===
// On Teensy the pins are fixed by the Wire instance: Wire1 = SDA1 (17) / SCL1 (16).
// The pin numbers below are documentation only. Do not name them SDA/SCL: the
// Teensy core already declares constants with those names.
#define OTOS_WIRE           Wire1 // C'est Wire (si 19 et 18) ou Wire1 (si 17 et 16) ou Wire2 (si 24 et 25)
#define OTOS_SDA_PIN        17  //Pin a changé en fonction de l'elek 
#define OTOS_SCL_PIN        16  //Pin a changé en fonction de l'elek 
#define OTOS_I2C_CLOCK_HZ   400000   // 100000 or 400000

// === SERIAL ===
#define SERIAL_BAUD         115200
#define WAIT_FOR_SERIAL_MS  3000     // time left to open the serial monitor

// === BOOT SEQUENCE ===
#define CALIB_SETTLE_MS         3000 // countdown before IMU calibration (hands off)
#define RUN_DRIFT_TEST          1    // 1 = static drift test at boot
#define DRIFT_TEST_DURATION_S   60
#define RUN_I2C_BENCHMARK       1    // 1 = timed burst of pose reads at boot
#define BENCHMARK_READS         1000

// === MEASUREMENT STREAM ===
#define STREAM_PERIOD_MS    50       // one measurement line per period
#define STREAM_CSV          0        // 0 = human readable, 1 = CSV
#define SUMMARY_PERIOD_MS   1000     // summary line period
#define RECONNECT_PERIOD_MS 1000     // retry period when the sensor is lost

// === SENSOR SETTINGS (applied after calibration) ===
// Scalars must stay within [0.872, 1.127]: scalar = true value / measured value.
#define LINEAR_SCALAR       1.0f
#define ANGULAR_SCALAR      1.0f

// Sensor position relative to the robot center (0 while off the robot).
#define OFFSET_X_MM         0.0f
#define OFFSET_Y_MM         0.0f
#define OFFSET_H_DEG        0.0f

// Pose the tracking starts from after reset.
#define START_X_MM          0.0f
#define START_Y_MM          0.0f
#define START_H_DEG         0.0f
