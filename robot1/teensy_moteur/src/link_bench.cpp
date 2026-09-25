/*
 * Teensy Moteur — inter-Teensy link bench.
 *
 * Build with the dedicated environment, not the default one:
 *     pio run -e link_bench -t upload
 *
 * WHAT IT DOES: emits an EncoderFrame at 100 Hz, consumes the PoseFrame the
 * Teensy Capteur answers, and reports round-trip time, pose age, lost frames and
 * CRC errors. This board is the time master, so it owns the IntervalTimer and it
 * is the one that measures.
 *
 * WHAT IT DOES NOT DO — read on, these two limits matter for the bench session:
 *
 *  - The encoder deltas are SYNTHETIC. This bench validates transport, and it has
 *    to run with two bare Teensys and no MKS driver wired. Measuring the real
 *    readAllEncodersSynced() duration is a separate exercise, and mixing the two
 *    would tell you nothing about either.
 *
 *  - It does not drive motors, so it cannot fully validate "unplug the link ->
 *    motors actually stop". What it does validate is the
 *    DETECTION and its latency: the cut decision is taken, timestamped and
 *    reported. The actual cut belongs to the production firmware.
 */

#include <Arduino.h>

#include "config.h"
#include "teensy_link.h"

// === LINK CONFIGURATION ===
// Serial4 is RX 16 / TX 17 on a Teensy 4.1, and is free: the three MKS drivers
// use Serial1 (0/1), Serial2 (7/8) and Serial6 (24/25). Keep this wiring
// identical to teensy_capteur/include/config.h; the 2026 prototype carried three
// contradictory versions of it.

#define LINK_SERIAL Serial4
#define LINK_BAUD 2000000
#define LINK_RX_BUFFER_SIZE 512

#define TICK_PERIOD_US 10000  // 10 ms = 100 Hz
#define DEBUG_PERIOD_MS 1000

// === TICK (ISR) ===

static IntervalTimer tick_timer;

/** Measured interval between ticks. This is what goes on the wire as dt_us. */
static elapsedMicros since_last_tick;

static volatile uint32_t tick_dt_us = 0;
static volatile bool tick_pending = false;

/** Ticks lost because loop() had not consumed the previous one. */
static volatile uint32_t ticks_dropped = 0;

/**
 * @brief 100 Hz tick. Deliberately tiny: it only measures time and raises a flag.
 *
 * Nothing is framed or written here. Serial4.write() can block when the transmit
 * buffer is full, and the existing firmware already follows this pattern
 * (flags raised by the ISR, work done in loop()).
 */
static void tick_isr() {
    if (tick_pending) {
        ++ticks_dropped;
    }
    tick_dt_us = since_last_tick;
    since_last_tick = 0;
    tick_pending = true;
}

// === LINK STATE ===

static teensy_link::Decoder decoder;
static teensy_link::SeqTracker pose_seq_tracker;

static uint8_t link_rx_buffer[LINK_RX_BUFFER_SIZE];

static uint8_t encoder_seq = 0;
static uint32_t frames_sent = 0;

/**
 * @brief micros() at which each EncoderFrame seq was written to the link.
 *
 * Indexed by seq, so the wrap at 256 is exactly what makes this work: 256 frames
 * at 100 Hz is 2.56 s of history, far beyond any plausible round trip.
 */
static uint32_t sent_at_us[256] = {};

static uint32_t rtt_us_last = 0;
static uint32_t rtt_us_max = 0;
static uint32_t isr_to_send_us_max = 0;

static uint32_t last_pose_ms = 0;
static bool link_degraded = true;
static bool cut_requested = false;
static uint32_t last_debug_ms = 0;

/** Last pose received, kept only to show that the payload survives the link. */
static float last_x = 0.0f;
static float last_y = 0.0f;
static float last_theta = 0.0f;

/** Synthetic counts, so the bench runs with no driver attached. */
static int32_t synthetic_counts = 0;

/**
 * @brief Compose and send one EncoderFrame for the tick just consumed.
 *
 * @param dt_us Measured interval since the previous tick.
 */
static void send_encoder_frame(uint32_t dt_us) {
    // A slow ramp: monotonic, so a decode error on the other side is obvious, and
    // small enough to stay plausible as a per-cycle delta.
    synthetic_counts += 87;  // about 1 mm per cycle at COUNTS_TO_MM

    teensy_link::EncoderFrame frame;
    frame.seq = encoder_seq;
    frame.dt_us = dt_us;
    frame.d1 = synthetic_counts;
    frame.d2 = -synthetic_counts;
    frame.d3 = 0;
    // No MOTORS_ENABLED and no ENCODERS_VALID: the counts are synthetic, and
    // claiming otherwise would be a lie the filter would act on.
    frame.flags = 0;

    teensy_link::seal(frame);

    const uint32_t now_us = micros();
    sent_at_us[encoder_seq] = now_us;
    LINK_SERIAL.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));

    ++encoder_seq;
    ++frames_sent;
}

/**
 * @brief Account for one received PoseFrame.
 *
 * echo_seq is what makes this measurable: it names the EncoderFrame this pose was
 * computed from, so we can look up when we sent it.
 */
static void handle_pose_frame(const teensy_link::PoseFrame &pose) {
    const uint32_t now_us = micros();

    rtt_us_last = now_us - sent_at_us[pose.echo_seq];
    if (rtt_us_last > rtt_us_max) {
        rtt_us_max = rtt_us_last;
    }

    decoder.counters().seq_gaps += pose_seq_tracker.observe(pose.seq);

    last_x = pose.x;
    last_y = pose.y;
    last_theta = pose.theta;

    last_pose_ms = millis();
    link_degraded = false;
    cut_requested = false;
}

/**
 * @brief Two-stage link watchdog.
 *
 * Stage 1 degrades: the production firmware falls back to encoder-only odometry.
 * Stage 2 requests the motor cut. This bench drives no motor, so it reports the
 * decision and its latency instead of performing it.
 */
static void check_link_watchdog() {
    const uint32_t silent_ms = millis() - last_pose_ms;

    if (!link_degraded && silent_ms > teensy_link::LINK_TIMEOUT_MS) {
        link_degraded = true;
        ++decoder.counters().timeouts;
        Serial.print("LINK DEGRADED after ");
        Serial.print(silent_ms);
        Serial.println(" ms without a PoseFrame");
    }

    if (!cut_requested && silent_ms > teensy_link::LINK_FATAL_MS) {
        cut_requested = true;
        Serial.print("MOTOR CUT REQUESTED after ");
        Serial.print(silent_ms);
        Serial.println(" ms (this bench drives no motor: decision only)");
    }
}

static void print_debug() {
    if (millis() - last_debug_ms < DEBUG_PERIOD_MS) {
        return;
    }
    last_debug_ms = millis();

    const teensy_link::Counters &c = decoder.counters();

    Serial.print(link_degraded ? "DEGRADED" : "OK      ");
    Serial.print(" tx=");
    Serial.print(frames_sent);
    Serial.print(" rx=");
    Serial.print(c.frames_received);
    Serial.print(" rtt_us=");
    Serial.print(rtt_us_last);
    Serial.print(" rtt_max=");
    Serial.print(rtt_us_max);
    Serial.print(" isr2send_max=");
    Serial.print(isr_to_send_us_max);
    Serial.print(" crc_err=");
    Serial.print(c.crc_errors);
    Serial.print(" dropped=");
    Serial.print(c.bytes_discarded);
    Serial.print(" gaps=");
    Serial.print(c.seq_gaps);
    Serial.print(" ticks_lost=");
    Serial.print(ticks_dropped);
    Serial.print(" pose=(");
    Serial.print(last_x, 1);
    Serial.print(", ");
    Serial.print(last_y, 1);
    Serial.print(", ");
    Serial.print(last_theta, 3);
    Serial.println(")");
}

void setup() {
    Serial.begin(115200);  // Never wait on the USB host.

    LINK_SERIAL.begin(LINK_BAUD);
    LINK_SERIAL.addMemoryForRead(link_rx_buffer, sizeof(link_rx_buffer));

    last_pose_ms = millis();
    last_debug_ms = millis();

    Serial.println("Teensy Moteur - link bench (synthetic counts)");

    since_last_tick = 0;
    tick_timer.begin(tick_isr, TICK_PERIOD_US);
}

void loop() {
    if (tick_pending) {
        // Copy the shared values with interrupts masked: they are multi-byte and
        // the ISR writes them.
        noInterrupts();
        const uint32_t dt_us = tick_dt_us;
        tick_pending = false;
        interrupts();

        const uint32_t isr_to_send_us = since_last_tick;
        if (isr_to_send_us > isr_to_send_us_max) {
            // How late loop() was in picking the tick up. It is part of the pose
            // age, and it is the first thing that grows when the RS485 reads move
            // back into this loop.
            isr_to_send_us_max = isr_to_send_us;
        }

        send_encoder_frame(dt_us);
    }

    while (LINK_SERIAL.available() > 0) {
        if (!decoder.push(static_cast<uint8_t>(LINK_SERIAL.read()))) {
            continue;
        }
        const teensy_link::PoseFrame *pose = decoder.as_pose_frame();
        if (pose != nullptr) {
            handle_pose_frame(*pose);
        }
    }

    check_link_watchdog();
    print_debug();
}
