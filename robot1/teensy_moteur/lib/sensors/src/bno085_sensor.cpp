/*
 * BNO085 IMU. See include/bno085_sensor.h.
 */

#include "bno085_sensor.h"

#include <Wire.h>
#include <math.h>

namespace sensors {

namespace {

/** Yaw of a unit quaternion (real, i, j, k), rad. */
double quaternion_yaw(float r, float i, float j, float k) {
    return atan2(2.0 * (r * k + i * j), 1.0 - 2.0 * (j * j + k * k));
}

/** Wraps an angle into [-pi, pi). */
double wrap_pi(double theta) {
    theta = fmod(theta + PI, 2.0 * PI);
    if (theta < 0.0) {
        theta += 2.0 * PI;
    }
    return theta - PI;
}

}  // namespace

Bno085Sensor::Bno085Sensor(int8_t reset_pin) : bno_(reset_pin) {}

bool Bno085Sensor::begin(uint32_t report_period_us) {
    if (!bno_.begin_I2C()) {
        available_ = false;
        return false;
    }
    Wire.setClock(400000);  // I2C fast mode
    bno_.enableReport(SH2_GAME_ROTATION_VECTOR, report_period_us);

    // Brief wait for a first sample, which sets the origin. Never blocks for
    // more than 500 ms: without a sample the origin stays at 0.
    const uint32_t start_ms = millis();
    sh2_SensorValue_t sv;
    while ((millis() - start_ms) < 500) {
        if (bno_.getSensorEvent(&sv) && sv.sensorId == SH2_GAME_ROTATION_VECTOR) {
            yaw_offset_ = quaternion_yaw(sv.un.gameRotationVector.real,
                                         sv.un.gameRotationVector.i,
                                         sv.un.gameRotationVector.j,
                                         sv.un.gameRotationVector.k);
            break;
        }
        delay(10);
    }

    available_ = true;
    return true;
}

bool Bno085Sensor::poll() {
    if (!available_) return false;

    sh2_SensorValue_t sv;
    if (!bno_.getSensorEvent(&sv)) return false;
    if (sv.sensorId != SH2_GAME_ROTATION_VECTOR) return false;

    const double yaw = quaternion_yaw(sv.un.gameRotationVector.real,
                                      sv.un.gameRotationVector.i,
                                      sv.un.gameRotationVector.j,
                                      sv.un.gameRotationVector.k);

    noInterrupts();
    raw_yaw_ = yaw;
    valid_ = true;
    timestamp_us_ = micros();
    interrupts();
    return true;
}

void Bno085Sensor::zero_heading() {
    noInterrupts();
    if (valid_) {
        yaw_offset_ = raw_yaw_;
    }
    interrupts();
}

HeadingSample Bno085Sensor::latest() const {
    HeadingSample s;
    noInterrupts();
    const double yaw = raw_yaw_;
    const double offset = yaw_offset_;
    s.valid = valid_;
    s.timestamp_us = timestamp_us_;
    interrupts();
    s.heading_rad = static_cast<float>(wrap_pi(yaw - offset));
    return s;
}

}  // namespace sensors
