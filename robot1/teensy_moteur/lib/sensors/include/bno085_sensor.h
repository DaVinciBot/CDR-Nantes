/*
 * BNO085 IMU: heading of the robot, from the game rotation vector.
 *
 * Moved out of holonomic_basis unchanged in behaviour: same report, same 500 ms
 * wait for a first sample at boot, same yaw formula. What changed is the shape:
 *   - poll() runs in loop(), never in the ISR (I2C);
 *   - latest() hands the ISR a copy taken with interrupts off;
 *   - a sensor with nothing new keeps its last sample and timestamp: a consumer
 *     sees it is stale from the timestamp, it never receives a zero heading.
 *
 * The game rotation vector integrates the gyro (no magnetometer): its rate is
 * NOT a second measurement, and nothing corrects its drift but the estimator.
 */

#ifndef BNO085_SENSOR_H
#define BNO085_SENSOR_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

namespace sensors {

/** @brief One heading sample. */
struct HeadingSample {
    /** False until a first sample arrived. */
    bool valid;
    /** Heading in the table frame, origin set by zero_heading(), rad. */
    float heading_rad;
    /** micros() at reception. */
    uint32_t timestamp_us;
};

class Bno085Sensor {
public:
    /** @param reset_pin Reset pin of the BNO085, -1 if not wired. */
    explicit Bno085Sensor(int8_t reset_pin = -1);

    /**
     * @brief Starts the IMU on Wire at 400 kHz and enables the report.
     *
     * Waits up to 500 ms for a first sample, which becomes the heading origin.
     *
     * @param report_period_us Period of the game rotation vector report; keep it
     *     equal to the control-loop period, or events queue up and the heading
     *     arrives late.
     * @return False if the IMU does not answer.
     */
    bool begin(uint32_t report_period_us);

    /** @brief Reads one pending event. Call from loop() only.
     *  @return True if a new heading was buffered. */
    bool poll();

    /** @brief The current heading becomes 0. No-op before a first sample. */
    void zero_heading();

    /** @return A copy of the last sample; safe from the ISR. */
    HeadingSample latest() const;

    /** @return True once begin() succeeded. */
    bool available() const { return available_; }

private:
    Adafruit_BNO08x bno_;
    bool available_ = false;

    // Written by poll() and zero_heading() (loop), read by latest() (ISR).
    double yaw_offset_ = 0.0;
    double raw_yaw_ = 0.0;
    bool valid_ = false;
    uint32_t timestamp_us_ = 0;
};

}  // namespace sensors

#endif  // BNO085_SENSOR_H
