/*
 * Teensy Capteur (sensor board) — transport only.
 *
 * WHAT THIS FIRMWARE DOES TODAY: nothing but transport. It receives the
 * EncoderFrame from the Teensy Moteur and answers a CONSTANT dummy pose. No
 * sensor is read, no filter runs.
 *
 * That is the point: the transport is validated before any computation runs on
 * it. A constant pose circulating at 100 Hz with a measured RTT and a watchdog
 * that really cuts the motors is a deliverable in itself, and it is precisely
 * what was missing in
 * 2026 — where the A* pathfinder, the Umeyama SVD and the EKF prototype were all
 * theoretically correct and none of them was ever wired in.
 *
 * Because the pose is constant while seq and echo_seq vary, any corruption in
 * the pose fields is unambiguous during the link bench.
 *
 * SCHEDULING: this board is purely reactive and has no IntervalTimer. The Teensy
 * Moteur is the time master, so the filter will run when a frame arrives, using
 * the dt_us that the frame carries — not on a local clock. The only timed duties
 * here are the link watchdog and the debug output, both non-critical and both
 * served by millis() comparisons. Adding an ISR would mean guarding the decoder
 * state against it for no gain.
 *
 * There is no delay() anywhere, and there must never be one. The 2026 prototype
 * paced itself with delay(10) at the tail of loop(), which made it run at 83-87
 * Hz while announcing 100 Hz.
 */

#include <Arduino.h>

#include "config.h"
#include "teensy_link.h"

// === LINK STATE ===

static teensy_link::Decoder decoder;
static teensy_link::SeqTracker encoder_seq;

/** RX buffer handed to the UART driver, sized in config.h. */
static uint8_t link_rx_buffer[LINK_RX_BUFFER_SIZE];

/** Our own sequence number for outgoing PoseFrames. */
static uint8_t pose_seq = 0;

/** Frames we emitted. The decoder owns the receive-side counters. */
static uint32_t frames_sent = 0;

/** millis() of the last valid EncoderFrame, for the link watchdog. */
static uint32_t last_encoder_frame_ms = 0;
static bool link_degraded = true;  // degraded until the first frame proves otherwise

static uint32_t last_debug_ms = 0;

// === DUMMY POSE ===
// Deliberately constant and deliberately not zero: a zero pose is
// indistinguishable from an all-zero buffer, which is what a broken link
// produces.

static constexpr float DUMMY_X_MM = 1000.0f;
static constexpr float DUMMY_Y_MM = 500.0f;
static constexpr float DUMMY_THETA_RAD = 0.5f;

/**
 * @brief Answer one EncoderFrame with a PoseFrame.
 *
 * @param frame The decoded, CRC-checked incoming frame.
 */
static void answer_encoder_frame(const teensy_link::EncoderFrame &frame) {
    const uint8_t missed = encoder_seq.observe(frame.seq);
    decoder.counters().seq_gaps += missed;

    last_encoder_frame_ms = millis();
    link_degraded = false;

    teensy_link::PoseFrame pose;
    pose.seq = pose_seq++;
    // This is what makes the RTT and the age of the pose at the PID measurable
    // on the other board: it can look up when it sent this very seq.
    pose.echo_seq = frame.seq;
    pose.x = DUMMY_X_MM;
    pose.y = DUMMY_Y_MM;
    pose.theta = DUMMY_THETA_RAD;

    // No FILTER_READY, no OPTICAL_OK, no IMU_OK: there is no filter and no
    // sensor yet, and saying otherwise would be a lie the Moteur would act on.
    pose.flags = 0;

    teensy_link::seal(pose);

    LINK_SERIAL.write(reinterpret_cast<const uint8_t *>(&pose), sizeof(pose));
    ++frames_sent;
}

/**
 * @brief Raise the degraded flag when the tick stops arriving.
 *
 * Without a tick this board has no dt, so it does not predict into the void: it
 * waits and reports. Inventing time would corrupt the filter silently.
 */
static void check_link_watchdog() {
    if (link_degraded) {
        return;
    }
    if (millis() - last_encoder_frame_ms > teensy_link::LINK_TIMEOUT_MS) {
        link_degraded = true;
        ++decoder.counters().timeouts;
        decoder.reset();
        encoder_seq.reset();
    }
}

/** Periodic health line on the development USB serial. */
static void print_debug() {
    if (millis() - last_debug_ms < DEBUG_PERIOD_MS) {
        return;
    }
    last_debug_ms = millis();

    const teensy_link::Counters &c = decoder.counters();
    Serial.printf(
        "link=%s tx=%lu rx=%lu crc_err=%lu unknown=%lu dropped=%lu gaps=%lu "
        "timeouts=%lu\n",
        link_degraded ? "DEGRADED" : "OK", (unsigned long)frames_sent,
        (unsigned long)c.frames_received, (unsigned long)c.crc_errors,
        (unsigned long)c.unknown_type, (unsigned long)c.bytes_discarded,
        (unsigned long)c.seq_gaps, (unsigned long)c.timeouts);
}

void setup() {
    Serial.begin(DEBUG_BAUD);  // Never wait on the USB host: the board must run headless.

    LINK_SERIAL.begin(LINK_BAUD);
    LINK_SERIAL.addMemoryForRead(link_rx_buffer, sizeof(link_rx_buffer));

    last_encoder_frame_ms = millis();
    last_debug_ms = millis();

    Serial.println("Teensy Capteur - transport only, constant pose");
}

void loop() {
    // Drain the UART. The decoder is byte-at-a-time and never allocates, so
    // there is nothing to bound here beyond what the driver already buffered.
    while (LINK_SERIAL.available() > 0) {
        if (!decoder.push(static_cast<uint8_t>(LINK_SERIAL.read()))) {
            continue;
        }
        const teensy_link::EncoderFrame *frame = decoder.as_encoder_frame();
        if (frame != nullptr) {
            answer_encoder_frame(*frame);
        }
        // A PoseFrame arriving here would mean the link is looped back on
        // itself. The decoder already counted it; ignoring it is correct.
    }

    check_link_watchdog();
    print_debug();
}
