/*
 * Telemetry of the base. See include/telemetry.h.
 */

#include "telemetry.h"

#include <string.h>

namespace telemetry {

namespace {

uint16_t saturate_u16(uint32_t value) {
    return value > 0xFFFFu ? 0xFFFFu : static_cast<uint16_t>(value);
}

}  // namespace

Telemetry::Telemetry(logring::LogRing &ring, uint32_t period_us, float counts_per_rev)
    : ring_(ring), period_us_(period_us), counts_per_rev_(counts_per_rev) {}

uint8_t Telemetry::start_tick(uint32_t t_us) {
    (void)t_us;
    tick_ = tick_ + 1;
    return mask_;
}

void Telemetry::emit_tick(const TickSnapshot &s) {
    const uint8_t mask = mask_;
    if (mask & TRACE_MASK_LOG_TICK) {
        msg_log_tick m;
        fill_log_tick(m, s);
        ring_.push(&m, sizeof(m));
    }
    // Measured wheel speeds, updated at every snapshot even when only LOG_TICK
    // is on: the value held over a missed read is then the last real one.
    const pose::EstimatorInput &in = s.input;
    if (in.encoders_fresh && in.dt_us > 0 && counts_per_rev_ > 0.0f) {
        const float to_rpm = 60.0e6f / (counts_per_rev_ * static_cast<float>(in.dt_us));
        wheel_measured_[0] = static_cast<float>(in.d1) * to_rpm;
        wheel_measured_[1] = static_cast<float>(in.d2) * to_rpm;
        wheel_measured_[2] = static_cast<float>(in.d3) * to_rpm;
    }
    if (mask & TRACE_MASK_PID) {
        msg_pid_trace m;
        fill_pid_trace(m, s);
        ring_.push(&m, sizeof(m));
    }
}

void Telemetry::fill_log_tick(msg_log_tick &m, const TickSnapshot &s) {
    m.header.schema_version = LOG_SCHEMA_VERSION;
    m.header.seq = seq_tick_++;
    m.header.tick = tick_;
    m.header.t_us = s.t_us;

    const pose::EstimatorInput &in = s.input;
    m.d[0] = in.d1;
    m.d[1] = in.d2;
    m.d[2] = in.d3;
    m.dt_us = in.dt_us;
    m.encoder_age_us = in.encoder_age_us;
    m.heading_rad = in.heading_rad;
    m.imu_timestamp_us = in.imu_timestamp_us;
    m.imu_age_us = in.imu_age_us;
    m.optical_timestamp_us = in.optical_timestamp_us;
    m.optical_dx_mm = in.optical_dx_mm;
    m.optical_dy_mm = in.optical_dy_mm;
    m.optical_step_dt_us = in.optical_step_dt_us;
    m.optical_vx_mm_s = in.optical_vx_mm_s;
    m.optical_vy_mm_s = in.optical_vy_mm_s;
    uint8_t input_flags = 0;
    if (in.first_frame) input_flags |= LOG_IN_FIRST_FRAME;
    if (in.encoders_fresh) input_flags |= LOG_IN_ENCODERS_FRESH;
    if (in.imu_valid) input_flags |= LOG_IN_IMU_VALID;
    if (in.optical_step_valid) input_flags |= LOG_IN_OPTICAL_STEP_VALID;
    if (in.optical_velocity_valid) input_flags |= LOG_IN_OPTICAL_VELOCITY_VALID;
    m.input_flags = input_flags;

    for (int i = 0; i < 3; ++i) {
        m.enc_counts[i] = s.enc_counts[i];
        m.otos_pos[i] = s.otos_pos[i];
        m.otos_vel[i] = s.otos_vel[i];
        m.otos_acc[i] = s.otos_acc[i];
        m.rs485_fail[i] = s.rs485_fail[i];
    }
    m.enc_t_us_mid = s.enc_t_us_mid;
    m.enc_read_ok = s.enc_read_ok;

    m.imu_raw_yaw_rad = s.imu_raw_yaw_rad;
    m.imu_yaw_offset_rad = s.imu_yaw_offset_rad;
    m.imu_accuracy = s.imu_accuracy;

    m.est_pose[0] = s.estimate.x_mm;
    m.est_pose[1] = s.estimate.y_mm;
    m.est_pose[2] = s.estimate.theta_rad;
    m.est_vel[0] = s.estimate.vx_mm_s;
    m.est_vel[1] = s.estimate.vy_mm_s;
    m.est_vel[2] = s.estimate.omega_rad_s;
    m.est_health = s.est_health;
    m.est_flags = s.est_flags;

    m.cmd_count = s.cmd_count;
    m.cmd_t_us = s.cmd_t_us;

    uint8_t health = 0;
    if (s.work_us > period_us_) health |= LOG_HEALTH_OVERRUN;
    if (in.optical_step_valid) health |= LOG_HEALTH_OPTICAL_READ;
    if (in.imu_valid && in.imu_timestamp_us != last_imu_timestamp_us_) {
        health |= LOG_HEALTH_IMU_READ;
    }
    last_imu_timestamp_us_ = in.imu_timestamp_us;
    if (s.write_failed) health |= LOG_HEALTH_WRITE_FAILED;
    m.work_us = saturate_u16(s.work_us);
    m.health_flags = health;
    m.log_dropped = static_cast<uint16_t>(ring_.dropped());
    m.otos_read_us = saturate_u16(s.otos_read_us);
}

void Telemetry::fill_pid_trace(msg_pid_trace &m, const TickSnapshot &s) {
    m.t_us = s.t_us;
    m.pose[0] = s.estimate.x_mm;
    m.pose[1] = s.estimate.y_mm;
    m.pose[2] = s.estimate.theta_rad;
    for (int i = 0; i < 3; ++i) {
        // At rest the loop holds no target: the trace shows the pose, so a
        // plot of the error stays at zero instead of jumping to a stale target.
        m.target[i] = s.controlling ? s.target[i] : m.pose[i];
        m.velocity_cmd[i] = s.controlling ? s.velocity_cmd[i] : 0.0f;
        m.wheel_requested[i] = s.wheel_requested[i];
        m.wheel_cmd[i] = s.wheel_cmd[i];
        m.wheel_measured[i] = wheel_measured_[i];
        m.p_term[i] = s.controlling ? s.p_term[i] : 0.0f;
        m.i_term[i] = s.controlling ? s.i_term[i] : 0.0f;
        m.d_term[i] = s.controlling ? s.d_term[i] : 0.0f;
    }
    m.flags = s.trace_flags;
}

void Telemetry::event(uint8_t code, float v0, float v1, float v2, uint32_t arg) {
    msg_log_event m;
    m.header.schema_version = LOG_SCHEMA_VERSION;
    m.header.seq = seq_event_++;
    m.header.tick = tick_;
    m.header.t_us = micros();
    m.event = code;
    m.value[0] = v0;
    m.value[1] = v1;
    m.value[2] = v2;
    m.arg = arg;
    push(&m, sizeof(m));
}

bool Telemetry::push(const void *frame, uint16_t length) {
    // The ISR pushes too: with interrupts off, the two never interleave.
    noInterrupts();
    const bool ok = ring_.push(frame, length);
    interrupts();
    return ok;
}

}  // namespace telemetry
