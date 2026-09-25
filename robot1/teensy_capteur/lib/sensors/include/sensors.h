/*
 * Optical odometry sensor and IMU sampling. PLACEHOLDER — filled with the 3-state EKF.
 *
 * Both sensors sit on Wire (I2C, 400 kHz): SparkFun Qwiic OTOS for optical
 * displacement, BNO085 for heading.
 *
 * FOUR RULES THIS MODULE MUST HOLD, each one a lesson from existing code:
 *
 * 1. NEVER read I2C from an interrupt. On main, the OTOS read is a blocking I2C
 *    transaction inside the 100 Hz ISR — the one sensor that was not moved out to
 *    loop(). Sampling belongs on the reactive path, behind freshness flags.
 *
 * 2. A sensor that is not ready yields a cleared flag, never a zero value. The
 *    prototype's read_imu_yaw() returned 0.0f when no event was pending, which is
 *    indistinguishable from "did not turn".
 *
 * 3. Take the OTOS (dx, dy) and IGNORE its theta. It fuses its own IMU
 *    internally, so using both its pose and the BNO085 would count the same
 *    measurement twice. Note the OTOS API
 *    reports an absolute pose in METRES since resetTracking(), so deltas must be
 *    differenced and scaled to mm.
 *
 * 4. Thresholds in raw units, and no silent band-pass. The existing code rejects
 *    optical magnitudes above 15 mm per cycle and zeroes anything below 2 mm,
 *    which leaves a usable band of 200 to 1500 mm/s on a robot capped at
 *    200 mm/s: the band is empty except exactly at saturation. Rejecting an
 *    outlier is legitimate; silencing the whole low-speed range is the bug.
 *
 * The lever arm from the centre of rotation (OPTICAL_LEVER_X_MM / _Y_MM in
 * config.h) defaults to zero because the mechanical target puts the sensor at the
 * centre. Keep the compensation anyway: a residual 15 mm gives 45 mm/s at
 * 3 rad/s, and that is a systematic bias the filter reads as real velocity.
 *
 * Unlike lib/kalman and lib/teensy_link, this module talks to hardware and may
 * include Arduino headers. It is therefore not part of the "native" environment.
 */

#ifndef SENSORS_H
#define SENSORS_H

namespace sensors {

// Sampling goes here.

}  // namespace sensors

#endif  // SENSORS_H
