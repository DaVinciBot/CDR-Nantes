/*
 * SparkFun Qwiic OTOS optical odometry sensor: robot-frame motion.
 *
 * CONFIGURATION, written at every boot (the sensor forgets it at power-off):
 *   - units metres / radians (the library defaults to inches / degrees);
 *   - enRot = 0: the sensor does NOT rotate its motion by its own heading, so
 *     position and velocity stay in the sensor frame, which is what the EKF
 *     expects. The sensor's heading is never used: the IMU gives the heading;
 *   - enAcc = 0: its accelerometer stays out of its internal filter;
 *   - linear and angular scalars from config.h, read back and refused outside
 *     [0.95, 1.05] (a scalar further off is a mounting problem, not a tuning);
 *   - offset 0: the lever arm from the centre of rotation is compensated in the
 *     EKF only (Ekf6Params::optical_lever_*). Doing it in both places would
 *     make the robot "move" while spinning in place.
 *
 * READING: poll() in loop(), never in the ISR. One burst read of position,
 * velocity and acceleration (getPosVelAcc; the sensor's own standard
 * deviations are not used, and skipping them halves the transfer). The step is
 * the position difference since the previous read; both step and velocity are
 * rotated from the sensor axes to the robot axes by the mounting angle.
 *
 * The duration of every read is recorded: it is the number that decides
 * whether one board can run the whole control loop.
 */

#ifndef OTOS_SENSOR_H
#define OTOS_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

namespace sensors {

/** @brief Mounting and calibration of the sensor. */
struct OtosConfig {
    /** Linear scalar written at boot; must stay within [0.95, 1.05]. */
    float linear_scalar = 1.0f;
    /** Angular scalar written at boot; must stay within [0.95, 1.05]. */
    float angular_scalar = 1.0f;
    /** Angle from the robot x axis to the sensor x axis, rad. */
    float mount_angle_rad = 0.0f;
    /** I2C clock of the sensor's bus, Hz. */
    uint32_t i2c_clock_hz = 400000;
};

/** @brief One optical sample, robot frame. */
struct OpticalSample {
    /** False until a first step is available (the first read is a reference). */
    bool valid;
    /** Step since the previous read, mm. */
    float dx_mm;
    float dy_mm;
    /** Velocity, mm/s. */
    float vx_mm_s;
    float vy_mm_s;
    /** micros() at the read. */
    uint32_t timestamp_us;
    /** +1 per new sample: tells a new sample from the same one read twice. */
    uint32_t seq;
};

/** @brief Durations of the burst reads, us. */
struct ReadStats {
    uint32_t count;
    uint32_t errors;
    uint32_t min_us;
    uint32_t max_us;
    float mean_us;
};

class OtosSensor {
public:
    explicit OtosSensor(const OtosConfig &config);

    /**
     * @brief Starts the bus and the sensor, writes the configuration, calibrates
     * the IMU of the sensor and resets its tracking. Robot still: blocks up to
     * about 2 s.
     *
     * @param wire The I2C bus of the sensor (see OTOS_WIRE in config.h).
     * @return False if the sensor does not answer or refuses the configuration.
     */
    bool begin(TwoWire &wire);

    /** @brief One burst read. Call from loop() only.
     *  @return True if a new sample was buffered. */
    bool poll();

    /** @brief Zeroes the sensor's position; the next read is a new reference.
     *  I2C: call from loop() or setup(), never from the ISR. */
    void reset_tracking();

    /** @return A copy of the last sample; safe from the ISR. */
    OpticalSample latest() const;

    /** @return A copy of the read durations. */
    ReadStats read_stats() const;

    /** @return True once begin() succeeded. */
    bool available() const { return available_; }

private:
    /** Rotates a sensor-frame vector into the robot frame. */
    void to_robot(float sx, float sy, float &rx, float &ry) const;

    QwiicOTOS otos_;
    OtosConfig config_;
    float mount_cos_ = 1.0f;
    float mount_sin_ = 0.0f;
    bool available_ = false;

    bool has_reference_ = false;
    float last_x_m_ = 0.0f;
    float last_y_m_ = 0.0f;

    // Written by poll() (loop), read by latest() (ISR).
    OpticalSample sample_ = {false, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0};

    ReadStats stats_ = {0, 0, 0xFFFFFFFFu, 0, 0.0f};
    uint64_t stats_sum_us_ = 0;
};

}  // namespace sensors

#endif  // OTOS_SENSOR_H
