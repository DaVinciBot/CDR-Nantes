/*
 * On-target timing of the EKF cycle. Runs on a bare Teensy 4.1 only: nothing
 * wired, no sensor, no motor.
 *
 * Run with: pio test -e teensy41   (from robot1/teensy_moteur/, board plugged in)
 *
 * WHY: the budget for one filter cycle is 80 us at 100 Hz, and the reason the
 * filter is hand-written rather than built on a generic matrix library is that
 * such a library, measured on this same MCU, already spends 86 us on a smaller
 * filter than ours. A budget that is argued but never measured is a guess; this
 * suite turns it into a number, on the real chip, with the real compiler flags.
 *
 * HOW: each operation is timed 10000 times with the ARM cycle counter
 * (DWT_CYCCNT, one tick per CPU cycle at 600 MHz). The inputs change at every
 * iteration so that the compiler cannot fold the work away, and every result
 * is written to a volatile sink. Min, mean and max are reported in ns. The max
 * includes whatever interrupt fired during the run (USB, systick): that is
 * deliberate, the filter will not run with interrupts masked either.
 *
 * The full cycle must fit the budget; the partial timings are informative.
 * Each model adds its own cycle here as it lands.
 */

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <unity.h>

#include "ekf3.h"
#include "ekf6.h"

using namespace kalman;

void setUp(void) {}
void tearDown(void) {}

namespace {

constexpr uint32_t ITERATIONS = 10000;

/** Budget for one full filter cycle, predict plus every update, ns. */
constexpr uint32_t BUDGET_NS = 80000;

const Geometry GEOMETRY = {160.0f, 60.0f * PI_F / 16384.0f};
const Ekf3Params EKF3_PARAMS = {10.0f, 0.02f, 0.01f};

/** Written after every timed call, so that the work cannot be optimised out. */
volatile float sink = 0.0f;

struct Timing {
    uint32_t min_cycles = UINT32_MAX;
    uint32_t max_cycles = 0;
    uint64_t total_cycles = 0;
};

uint32_t cycles_to_ns(uint64_t cycles) {
    return static_cast<uint32_t>(cycles * 1000u / (F_CPU_ACTUAL / 1000000u));
}

/**
 * @brief Times body(k) for k = 0 .. ITERATIONS-1.
 *
 * @param body Callable taking the iteration index.
 * @return Min, max and total cycle counts.
 */
template <typename Body>
Timing measure(Body body) {
    Timing timing;
    for (uint32_t k = 0; k < ITERATIONS; ++k) {
        const uint32_t start = ARM_DWT_CYCCNT;
        body(k);
        // Unsigned subtraction: correct across the 32-bit wrap (every 7 s).
        const uint32_t cycles = ARM_DWT_CYCCNT - start;
        if (cycles < timing.min_cycles) {
            timing.min_cycles = cycles;
        }
        if (cycles > timing.max_cycles) {
            timing.max_cycles = cycles;
        }
        timing.total_cycles += cycles;
    }
    return timing;
}

void report(const char *label, const Timing &timing) {
    // Integers only: float formatting in printf is not guaranteed on every core.
    char line[160];
    snprintf(line, sizeof(line), "%s: min %lu ns, mean %lu ns, max %lu ns (%lu runs, %lu MHz)",
             label, static_cast<unsigned long>(cycles_to_ns(timing.min_cycles)),
             static_cast<unsigned long>(cycles_to_ns(timing.total_cycles / ITERATIONS)),
             static_cast<unsigned long>(cycles_to_ns(timing.max_cycles)),
             static_cast<unsigned long>(ITERATIONS),
             static_cast<unsigned long>(F_CPU_ACTUAL / 1000000u));
    TEST_MESSAGE(line);
}

/** A plausible tick at about 200 mm/s, varying with k. */
OdometryTick tick_for(uint32_t k) {
    OdometryTick tick;
    tick.d1 = -4000 + static_cast<int32_t>(k % 97);
    tick.d2 = -3000 - static_cast<int32_t>(k % 89);
    tick.d3 = 8000 + static_cast<int32_t>(k % 83);
    tick.dt_us = 10000;
    tick.first_frame = false;
    tick.encoders_valid = true;
    return tick;
}

/** A heading measurement close to the estimate, varying with k. */
float heading_for(const Ekf3 &filter, uint32_t k) {
    return filter.state()[Ekf3::THETA] + 0.001f * (static_cast<float>(k % 7) - 3.0f);
}

}  // namespace

// === 3-STATE FILTER ===

void test_ekf3_predict_tick(void) {
    Ekf3 filter(GEOMETRY, EKF3_PARAMS);
    const Timing timing = measure([&](uint32_t k) {
        filter.predict_tick(tick_for(k));
        sink = filter.covariance()(0, 0);
    });
    report("ekf3 predict_tick", timing);
}

void test_ekf3_update_heading(void) {
    Ekf3 filter(GEOMETRY, EKF3_PARAMS);
    filter.reset(0.0f, 0.0f, 0.0f, 10.0f, 0.05f);
    const Timing timing = measure([&](uint32_t k) {
        filter.update_heading(heading_for(filter, k));
        sink = filter.covariance()(2, 2);
    });
    report("ekf3 update_heading", timing);
}

void test_ekf3_full_cycle_fits_the_budget(void) {
    Ekf3 filter(GEOMETRY, EKF3_PARAMS);
    const Timing timing = measure([&](uint32_t k) {
        filter.predict_tick(tick_for(k));
        filter.update_heading(heading_for(filter, k));
        sink = filter.state()[0];
    });
    report("ekf3 full cycle", timing);
    TEST_ASSERT_LESS_THAN_UINT32(BUDGET_NS, cycles_to_ns(timing.max_cycles));
}

// === 6-STATE FILTER (production) ===

/** Filter already moving, so that every update does real work. */
Ekf6 moving_ekf6() {
    Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    Vec<Ekf6::N> state = Vec<Ekf6::N>::zero();
    state[Ekf6::VX] = 200.0f;
    state[Ekf6::OMEGA] = 0.2f;
    Vec<Ekf6::N> variance;
    for (size_t i = 0; i < Ekf6::N; ++i) {
        variance[i] = 1.0f;
    }
    filter.reset(state, Mat<Ekf6::N>::diagonal(variance));
    return filter;
}

void test_ekf6_predict(void) {
    Ekf6 filter = moving_ekf6();
    const Timing timing = measure([&](uint32_t k) {
        filter.predict(0.01f + 1e-6f * static_cast<float>(k % 11));
        sink = filter.covariance()(0, 0);
    });
    report("ekf6 predict", timing);
}

void test_ekf6_full_cycle_fits_the_budget(void) {
    // One tick as it will run in production: prediction, three wheel updates,
    // two optical updates, one heading update.
    Ekf6 filter = moving_ekf6();
    const Timing timing = measure([&](uint32_t k) {
        filter.process_tick(tick_for(k));
        const float wobble = static_cast<float>(k % 13) - 6.0f;
        filter.update_optical(200.0f + wobble, 0.5f * wobble, (k + 1) * 10000u);
        filter.update_heading(filter.state()[Ekf6::THETA] + 0.001f * wobble, (k + 1) * 10000u);
        sink = filter.state()[0];
    });
    report("ekf6 full cycle", timing);
    TEST_ASSERT_LESS_THAN_UINT32(BUDGET_NS, cycles_to_ns(timing.max_cycles));
}

void setup() {
    // The cycle counter is already running on Teensy 4 (micros() relies on it);
    // enabling it again is harmless and makes the dependency explicit.
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;

    // Wait for the test runner to open the port, but never forever.
    while (!Serial && millis() < 4000) {
    }

    UNITY_BEGIN();

    RUN_TEST(test_ekf3_predict_tick);
    RUN_TEST(test_ekf3_update_heading);
    RUN_TEST(test_ekf3_full_cycle_fits_the_budget);

    RUN_TEST(test_ekf6_predict);
    RUN_TEST(test_ekf6_full_cycle_fits_the_budget);

    UNITY_END();
}

void loop() {}
