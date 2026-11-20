/*
 * Telemetry of the base: the LOG_* messages and PID_TRACE.
 *
 * Who calls what:
 *   - the control tick (ISR) calls start_tick() once per tick, whatever is
 *     enabled. If it returns a non-zero mask, the tick fills a TickSnapshot and
 *     hands it to emit_tick(), which builds the enabled frames (LOG_TICK,
 *     PID_TRACE) and copies them into the ring. Nothing is built for a stream
 *     that is off: it costs nothing in the tick;
 *   - loop() sets the mask (SET_TRACE), pushes the rare frames (event(),
 *     push()), and empties the ring towards the link with drain();
 *   - drain() writes with Com::send_log(): never waits for the host. A frame
 *     the link cannot take now stays in the ring for the next call; frames are
 *     lost only when the ring is full, and then counted (log_dropped of
 *     LOG_TICK, and a gap in the seq of the stream).
 *
 * This file and telemetry.cpp only need messages.h and the Arduino time and
 * interrupt calls: the frame builders are checked on a PC against the Python
 * decoder.
 */

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <messages.h>

#include "log_ring.h"
#include "pose_estimator.h"

namespace telemetry {

/** @brief Everything one LOG_TICK and one PID_TRACE carry, gathered by the tick. */
struct TickSnapshot {
    /** micros() at the start of the tick. */
    uint32_t t_us;

    // --- Estimator, as stepped this tick ---
    pose::EstimatorInput input;
    pose::PoseEstimate estimate;
    uint8_t est_health;
    uint8_t est_flags;  // pose::StepFlag bits

    // --- Encoders: last counts handed to the estimator, last read attempt ---
    int64_t enc_counts[3];
    uint32_t enc_t_us_mid;
    uint8_t enc_read_ok;    // bit i: wheel i + 1 answered
    uint8_t rs485_fail[3];  // RS485_* of each wheel

    // --- Raw sensors (zeros when absent) ---
    float otos_pos[3];
    float otos_vel[3];
    float otos_acc[3];
    uint32_t otos_read_us;
    float imu_raw_yaw_rad;
    float imu_yaw_offset_rad;
    uint8_t imu_accuracy;

    // --- Commands received ---
    uint16_t cmd_count;  // SET_TARGET accepted since boot
    uint32_t cmd_t_us;   // micros() when the last one was taken

    // --- Cycle ---
    uint32_t work_us;    // work of the previous tick (ISR + loop work it caused)
    bool write_failed;   // last wheel speeds not sent to every driver

    // --- Control (PID_TRACE) ---
    bool controlling;    // the position loop ran this tick
    float target[3];     // reference chased, table frame
    float velocity_cmd[3];
    float wheel_requested[3];
    float wheel_cmd[3];
    float p_term[3];
    float i_term[3];
    float d_term[3];
    uint8_t trace_flags;  // TRACE_*
};

class Telemetry {
public:
    /**
     * @param ring Where the frames wait for the link.
     * @param period_us Control period: a tick whose work exceeds it is flagged.
     * @param counts_per_rev Encoder counts per wheel turn, for the measured rpm.
     */
    Telemetry(logring::LogRing &ring, uint32_t period_us, float counts_per_rev);

    /** @brief Per-tick messages to send, TRACE_MASK_* bits. From loop(). */
    void set_mask(uint8_t mask) { mask_ = mask; }
    uint8_t mask() const { return mask_; }

    /**
     * @brief Counts one tick. ISR, once per tick.
     * @return The mask: non-zero means emit_tick() wants a snapshot.
     */
    uint8_t start_tick(uint32_t t_us);

    /** @brief Builds the enabled per-tick frames. ISR, after start_tick(). */
    void emit_tick(const TickSnapshot &snapshot);

    /** @brief Queues a LOG_EVENT. From loop(). */
    void event(uint8_t code, float v0 = 0.0f, float v1 = 0.0f, float v2 = 0.0f,
               uint32_t arg = 0);

    /** @brief Queues any frame (LOG_FW_INFO, LOG_PARAMS). From loop().
     *  @return False if it was dropped. */
    bool push(const void *frame, uint16_t length);

    /**
     * @brief Sends what is waiting, oldest first. From loop().
     * @param link Anything with `bool send_log(const byte*, byte)` (Com).
     * @param max_frames At most this many per call, so that a backlog never
     *     holds loop() away from the RS485 for long.
     * @return Frames sent.
     */
    template <typename Link>
    uint16_t drain(Link &link, uint16_t max_frames) {
        uint16_t sent = 0;
        uint16_t length = 0;
        while (sent < max_frames) {
            const uint8_t *frame = ring_.front(length);
            if (frame == nullptr) break;
            if (!link.send_log(frame, static_cast<byte>(length))) break;
            ring_.pop();
            ++sent;
        }
        return sent;
    }

    /** @return Ticks since boot. */
    uint32_t ticks() const { return tick_; }

    /** @return Frames dropped since boot, ring full. */
    uint32_t dropped() const { return ring_.dropped(); }

    /** @brief Builds a LOG_TICK. Public for the tests. */
    void fill_log_tick(msg_log_tick &m, const TickSnapshot &s);
    /** @brief Builds a PID_TRACE. Public for the tests. */
    void fill_pid_trace(msg_pid_trace &m, const TickSnapshot &s);

private:
    logring::LogRing &ring_;
    uint32_t period_us_;
    float counts_per_rev_;

    volatile uint8_t mask_ = 0;
    volatile uint32_t tick_ = 0;   // written in the ISR, read by event()
    uint16_t seq_tick_ = 0;        // ISR only
    uint16_t seq_event_ = 0;       // loop only
    uint32_t last_imu_timestamp_us_ = 0;
    float wheel_measured_[3] = {0.0f, 0.0f, 0.0f};  // held over a missed read
};

}  // namespace telemetry

#endif  // TELEMETRY_H
