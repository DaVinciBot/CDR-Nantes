/*
 * SparkFun Qwiic OTOS. See include/otos_sensor.h.
 */

#include "otos_sensor.h"

#include <math.h>

namespace sensors {

namespace {

constexpr float SCALAR_MIN = 0.95f;
constexpr float SCALAR_MAX = 1.05f;
/** The sensor stores scalars in 0.1 % steps: read-back tolerance. */
constexpr float SCALAR_READBACK_TOLERANCE = 0.0015f;
/** Longest IMU calibration accepted at boot (255 samples take ~0.6 s). */
constexpr uint32_t CALIBRATION_TIMEOUT_MS = 2000;

bool scalar_in_range(float scalar) {
    return scalar >= SCALAR_MIN && scalar <= SCALAR_MAX;
}

}  // namespace

OtosSensor::OtosSensor(const OtosConfig &config) : config_(config) {
    mount_cos_ = cosf(config.mount_angle_rad);
    mount_sin_ = sinf(config.mount_angle_rad);
}

bool OtosSensor::begin(TwoWire &wire) {
    available_ = false;

    if (!scalar_in_range(config_.linear_scalar) || !scalar_in_range(config_.angular_scalar)) {
        return false;
    }

    wire.begin();
    wire.setClock(config_.i2c_clock_hz);
    if (!otos_.begin(wire)) {
        return false;
    }

    otos_.setLinearUnit(kSfeOtosLinearUnitMeters);
    otos_.setAngularUnit(kSfeOtosAngularUnitRadians);

    sfe_otos_signal_process_config_t processing;
    processing.value = 0;
    processing.enLut = 1;
    processing.enAcc = 0;
    processing.enRot = 0;
    processing.enVar = 1;
    if (otos_.setSignalProcessConfig(processing) != ksfTkErrOk) {
        return false;
    }

    if (otos_.setLinearScalar(config_.linear_scalar) != ksfTkErrOk ||
        otos_.setAngularScalar(config_.angular_scalar) != ksfTkErrOk) {
        return false;
    }
    float linear = 0.0f;
    float angular = 0.0f;
    if (otos_.getLinearScalar(linear) != ksfTkErrOk ||
        otos_.getAngularScalar(angular) != ksfTkErrOk) {
        return false;
    }
    if (fabsf(linear - config_.linear_scalar) > SCALAR_READBACK_TOLERANCE ||
        fabsf(angular - config_.angular_scalar) > SCALAR_READBACK_TOLERANCE) {
        return false;
    }

    sfe_otos_pose2d_t no_offset = {0.0f, 0.0f, 0.0f};
    if (otos_.setOffset(no_offset) != ksfTkErrOk) {
        return false;
    }

    // IMU calibration of the sensor, robot still. Polled with a timeout so that
    // a sensor that stops answering cannot hang the boot.
    if (otos_.calibrateImu(255, false) != ksfTkErrOk) {
        return false;
    }
    const uint32_t start_ms = millis();
    uint8_t samples_left = 255;
    while (samples_left > 0) {
        if (otos_.getImuCalibrationProgress(samples_left) != ksfTkErrOk) {
            return false;
        }
        if (millis() - start_ms > CALIBRATION_TIMEOUT_MS) {
            return false;
        }
        delay(3);
    }

    available_ = true;
    reset_tracking();
    return true;
}

void OtosSensor::reset_tracking() {
    if (!available_) return;
    otos_.resetTracking();
    has_reference_ = false;
}

void OtosSensor::to_robot(float sx, float sy, float &rx, float &ry) const {
    rx = mount_cos_ * sx - mount_sin_ * sy;
    ry = mount_sin_ * sx + mount_cos_ * sy;
}

bool OtosSensor::poll() {
    if (!available_) return false;

    sfe_otos_pose2d_t pos;
    sfe_otos_pose2d_t vel;
    sfe_otos_pose2d_t acc;
    const uint32_t start_us = micros();
    const sfTkError_t err = otos_.getPosVelAcc(pos, vel, acc);
    const uint32_t now_us = micros();
    const uint32_t duration_us = now_us - start_us;

    if (err != ksfTkErrOk) {
        ++stats_.errors;
        return false;
    }
    ++stats_.count;
    stats_sum_us_ += duration_us;
    if (duration_us < stats_.min_us) stats_.min_us = duration_us;
    if (duration_us > stats_.max_us) stats_.max_us = duration_us;
    stats_.mean_us = static_cast<float>(stats_sum_us_) / static_cast<float>(stats_.count);

    // The first read after a reset only sets the reference of the steps.
    if (!has_reference_) {
        has_reference_ = true;
        last_x_m_ = pos.x;
        last_y_m_ = pos.y;
        return false;
    }

    float dx_mm = 0.0f;
    float dy_mm = 0.0f;
    float vx_mm_s = 0.0f;
    float vy_mm_s = 0.0f;
    to_robot((pos.x - last_x_m_) * 1000.0f, (pos.y - last_y_m_) * 1000.0f, dx_mm, dy_mm);
    to_robot(vel.x * 1000.0f, vel.y * 1000.0f, vx_mm_s, vy_mm_s);
    last_x_m_ = pos.x;
    last_y_m_ = pos.y;

    noInterrupts();
    sample_.valid = true;
    sample_.dx_mm = dx_mm;
    sample_.dy_mm = dy_mm;
    sample_.vx_mm_s = vx_mm_s;
    sample_.vy_mm_s = vy_mm_s;
    sample_.timestamp_us = now_us;
    ++sample_.seq;
    interrupts();
    return true;
}

OpticalSample OtosSensor::latest() const {
    noInterrupts();
    const OpticalSample copy = sample_;
    interrupts();
    return copy;
}

ReadStats OtosSensor::read_stats() const {
    return stats_;
}

}  // namespace sensors
