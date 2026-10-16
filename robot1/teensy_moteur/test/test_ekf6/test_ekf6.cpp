/*
 * Host tests for the 6-state EKF.
 *
 * Run with: pio test -e native   (from robot1/teensy_moteur/)
 *
 * MODEL: the sparse F P F^T against the dense product, F against finite
 * differences of the transition, the wheel rows against the inverse kinematics.
 *
 * TIME AND SAMPLES: first frame, bad dt, invalid encoders, stale samples.
 *
 * CONSISTENCY: a Monte-Carlo truth that follows exactly the filter's own process
 * and measurement models. Mean NIS per source near 1 and 2-sigma coverage near
 * 95 % are the proof that the covariance means what it says, which RMSE alone
 * cannot give.
 *
 * FAILURES SEEN IN THE FIELD: the SparkFun OTOS failures reported by FTC teams
 * are physical and silent (the sensor keeps returning plausible numbers): an
 * under-count of 10 to 30 % from dust, height or a cramped housing, and brief
 * losses of tracking. A drifting gyro is the other documented trap. Each test
 * below states what is detected AND what is not, so that the limit of the
 * filter is written down rather than discovered on the table.
 */

#include <math.h>
#include <random>
#include <unity.h>

#include "../support/dense_reference.h"
#include "ekf6.h"

using namespace kalman;

void setUp(void) {}
void tearDown(void) {}

// === FIXTURE ===

static constexpr float DEG = PI_F / 180.0f;
static constexpr float DT = 0.01f;
static constexpr uint32_t DT_US = 10000;

/** Geometry of the 2026 base: R = 160 mm, 60 mm wheels, 16384 counts/rev. */
static const Geometry GEOMETRY = {160.0f, 60.0f * PI_F / 16384.0f};

static std::mt19937 &rng() {
    static std::mt19937 generator(20260927u);
    return generator;
}

static double gaussian(double sigma) {
    std::normal_distribution<double> dist(0.0, sigma);
    return dist(rng());
}

static double uniform(double lo, double hi) {
    std::uniform_real_distribution<double> dist(lo, hi);
    return dist(rng());
}

static Vec<6> make_state(float x, float y, float theta, float vx, float vy, float omega) {
    Vec<6> s;
    s[Ekf6::X] = x;
    s[Ekf6::Y] = y;
    s[Ekf6::THETA] = theta;
    s[Ekf6::VX] = vx;
    s[Ekf6::VY] = vy;
    s[Ekf6::OMEGA] = omega;
    return s;
}

static Mat<6> make_diagonal(float xy, float theta, float v, float omega) {
    Vec<6> d = make_state(xy, xy, theta, v, v, omega);
    return Mat<6>::diagonal(d);
}

static OdometryTick make_tick(const int32_t counts[3], uint32_t dt_us) {
    OdometryTick tick;
    tick.d1 = counts[0];
    tick.d2 = counts[1];
    tick.d3 = counts[2];
    tick.dt_us = dt_us;
    tick.first_frame = false;
    tick.encoders_valid = true;
    return tick;
}

static float dot(const Vec<6> &a, const Vec<6> &b) {
    float sum = 0.0f;
    for (size_t i = 0; i < 6; ++i) {
        sum += a[i] * b[i];
    }
    return sum;
}

/**
 * Encoder tick for the true state, with the filter's own wheel noise model
 * when noisy, rounded to whole counts as the Moteur would.
 */
static OdometryTick wheel_tick(const Ekf6 &filter, const Ekf6Params &params,
                               const Vec<6> &truth, bool noisy) {
    const float floor_mm_s = params.wheel_sigma_counts * GEOMETRY.counts_to_mm / DT;
    int32_t counts[3];
    for (size_t w = 0; w < 3; ++w) {
        const double z = dot(filter.wheel_row(w), truth);
        const double slip = params.wheel_slip_ratio * z;
        const double noise = noisy ? gaussian(sqrt(floor_mm_s * floor_mm_s + slip * slip)) : 0.0;
        counts[w] = static_cast<int32_t>(lround((z + noise) * DT / GEOMETRY.counts_to_mm));
    }
    return make_tick(counts, DT_US);
}

// === MODEL ===

void test_wheel_rows_match_the_inverse_kinematics(void) {
    const Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    for (int trial = 0; trial < 100; ++trial) {
        const Vec<6> s = make_state(0.0f, 0.0f, 0.0f, static_cast<float>(uniform(-300, 300)),
                                    static_cast<float>(uniform(-300, 300)),
                                    static_cast<float>(uniform(-3, 3)));
        const BodyDelta delta = {s[Ekf6::VX] * DT, s[Ekf6::VY] * DT, s[Ekf6::OMEGA] * DT};
        float travel[3];
        inverse_kinematics(GEOMETRY, delta, travel);
        for (size_t w = 0; w < 3; ++w) {
            TEST_ASSERT_FLOAT_WITHIN(1e-2f, travel[w] / DT, dot(filter.wheel_row(w), s));
        }
    }
}

void test_jacobian_matches_finite_differences(void) {
    const Vec<6> s = make_state(10.0f, -20.0f, 0.7f, 150.0f, -80.0f, 0.9f);
    const float dt = 0.02f;
    Mat<6> F;
    Ekf6::jacobian(s, dt, F);

    // Step per variable: a small angle, a large velocity (x' is linear in it).
    const float step[6] = {1.0f, 1.0f, 1e-2f, 1.0f, 1.0f, 1e-2f};
    for (size_t col = 0; col < 6; ++col) {
        Vec<6> plus = s;
        Vec<6> minus = s;
        plus[col] += step[col];
        minus[col] -= step[col];
        const Vec<6> fp = Ekf6::transition(plus, dt);
        const Vec<6> fm = Ekf6::transition(minus, dt);
        for (size_t row = 0; row < 6; ++row) {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, F(row, col),
                                     (fp[row] - fm[row]) / (2.0f * step[col]));
        }
    }
}

void test_predicted_covariance_matches_dense_reference(void) {
    const Ekf6Params params = provisional_ekf6_params();
    for (int trial = 0; trial < 50; ++trial) {
        Mat<6> P = Mat<6>::zero();
        float a[6][6];
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                a[i][j] = static_cast<float>(uniform(-3.0, 3.0));
            }
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                for (int k = 0; k < 6; ++k) {
                    P(i, j) += a[i][k] * a[j][k];
                }
            }
        }
        const Vec<6> s = make_state(
            static_cast<float>(uniform(-1000, 1000)), static_cast<float>(uniform(-1000, 1000)),
            static_cast<float>(uniform(-3, 3)), static_cast<float>(uniform(-300, 300)),
            static_cast<float>(uniform(-300, 300)), static_cast<float>(uniform(-3, 3)));

        Ekf6 filter(GEOMETRY, params);
        Mat<6> F;
        Ekf6::jacobian(s, DT, F);
        const dense::DMat<6, 6> expected = dense::predict_covariance(
            dense::from(P), dense::from(F), dense::from(filter.process_noise(s, DT)));

        filter.reset(s, P);
        TEST_ASSERT_TRUE(filter.predict(DT));

        double scale = 1.0;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                scale = fmax(scale, fabs(expected(i, j)));
            }
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                TEST_ASSERT_FLOAT_WITHIN(1e-5 * scale, expected(i, j), filter.covariance()(i, j));
            }
        }
    }
}

void test_process_noise_is_positive_definite(void) {
    const Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    const Vec<6> s = make_state(0.0f, 0.0f, 1.0f, 200.0f, -50.0f, 0.5f);
    const dense::DMat<6, 6> Q = dense::from(filter.process_noise(s, DT));
    const dense::DMat<6, 6> L = dense::cholesky(Q);
    const dense::DMat<6, 6> back = dense::mul(L, dense::transpose(L));
    for (int i = 0; i < 6; ++i) {
        TEST_ASSERT_TRUE(L(i, i) > 0.0);
        for (int j = 0; j < 6; ++j) {
            TEST_ASSERT_FLOAT_WITHIN(1e-6 * (1.0 + fabs(Q(i, j))), Q(i, j), back(i, j));
        }
    }
}

// === MEAN PROPAGATION ===

void test_constant_body_velocity_goes_straight_along_the_heading(void) {
    Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    filter.reset(make_state(0.0f, 0.0f, 0.5f * PI_F, 200.0f, 0.0f, 0.0f), Mat<6>::zero());
    for (int k = 0; k < 100; ++k) {
        filter.predict(DT);
    }
    // One second at 200 mm/s, facing +y.
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.0f, filter.state()[Ekf6::X]);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 200.0f, filter.state()[Ekf6::Y]);
}

void test_constant_body_velocity_and_rate_draw_a_closed_circle(void) {
    // One turn in 10 s: radius 200 / (2 pi / 10).
    const float omega = TWO_PI_F / 10.0f;
    const float radius = 200.0f / omega;
    Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    filter.reset(make_state(0.0f, 0.0f, 0.0f, 200.0f, 0.0f, omega), Mat<6>::zero());

    for (int k = 0; k < 500; ++k) {
        filter.predict(DT);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, filter.state()[Ekf6::X]);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 2.0f * radius, filter.state()[Ekf6::Y]);

    for (int k = 500; k < 1000; ++k) {
        filter.predict(DT);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, filter.state()[Ekf6::X]);
    TEST_ASSERT_FLOAT_WITHIN(0.2f, 0.0f, filter.state()[Ekf6::Y]);
}

// === TIME AND SAMPLES ===

void test_first_frame_and_bad_dt_are_refused(void) {
    Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    filter.reset(make_state(0.0f, 0.0f, 0.0f, 200.0f, 0.0f, 0.0f), Mat<6>::zero());
    const int32_t counts[3] = {-87, -87, 174};

    OdometryTick tick = make_tick(counts, DT_US);
    tick.first_frame = true;
    TEST_ASSERT_FALSE(filter.process_tick(tick));

    TEST_ASSERT_FALSE(filter.predict(0.0f));
    TEST_ASSERT_FALSE(filter.predict(0.06f));  // longer than max_dt_s (50 ms)
    TEST_ASSERT_FALSE(filter.predict(NAN));
    TEST_ASSERT_FALSE(filter.process_tick(make_tick(counts, 60000)));

    TEST_ASSERT_EQUAL_UINT32(2, filter.ticks_skipped());
    TEST_ASSERT_EQUAL_UINT32(4, filter.predictions_refused());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, filter.state()[Ekf6::X]);
}

void test_invalid_encoders_still_predict_but_skip_the_wheels(void) {
    Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    filter.reset(make_state(0.0f, 0.0f, 0.0f, 200.0f, 0.0f, 0.0f), Mat<6>::zero());
    const int32_t counts[3] = {0, 0, 0};
    OdometryTick tick = make_tick(counts, DT_US);
    tick.encoders_valid = false;

    TEST_ASSERT_TRUE(filter.process_tick(tick));
    // The velocity model carried the pose: 200 mm/s for 10 ms.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, filter.state()[Ekf6::X]);
    // And the zero counts were NOT read as "the robot stopped".
    TEST_ASSERT_EQUAL_UINT32(0, filter.wheel_stats().accepted + filter.wheel_stats().rejected);
    TEST_ASSERT_EQUAL_FLOAT(200.0f, filter.state()[Ekf6::VX]);
}

void test_samples_not_newer_than_the_last_are_dropped(void) {
    Ekf6 filter(GEOMETRY, provisional_ekf6_params());
    filter.reset(0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 10.0f, 0.1f);

    TEST_ASSERT_FALSE(filter.update_optical(0.0f, 0.0f, 1000).stale);
    TEST_ASSERT_TRUE(filter.update_optical(0.0f, 0.0f, 1000).stale);  // same time
    TEST_ASSERT_TRUE(filter.update_optical(0.0f, 0.0f, 900).stale);   // older
    TEST_ASSERT_FALSE(filter.update_optical(0.0f, 0.0f, 1001).stale);

    // The microsecond clock wraps every 71 minutes: just after the wrap is later.
    filter.update_heading(0.0f, 0xFFFFFF00u);
    const uint32_t before = filter.stale_samples();
    filter.update_heading(0.0f, 0x00000100u);
    TEST_ASSERT_EQUAL_UINT32(before, filter.stale_samples());
    filter.update_heading(0.0f, 0xFFFFFF80u);
    TEST_ASSERT_EQUAL_UINT32(before + 1, filter.stale_samples());
}

// === MEASUREMENTS ===

void test_heading_update_crosses_pi_the_short_way(void) {
    Ekf6Params params = provisional_ekf6_params();
    params.heading_sigma_rad = 1.0f * DEG;
    Ekf6 filter(GEOMETRY, params);
    filter.reset(0.0f, 0.0f, 179.0f * DEG, 0.0f, 5.0f * DEG, 10.0f, 0.1f);

    const UpdateResult result = filter.update_heading(-179.0f * DEG, 1);
    TEST_ASSERT_TRUE(result.accepted);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f * DEG, result.innovation);
    TEST_ASSERT_TRUE(fabsf(filter.state()[Ekf6::THETA]) > 179.0f * DEG);
}

/** Pure rotation at 3 rad/s, optical sensor 15 mm off centre, noise-free. */
static Ekf6 spin_with_offset_sensor(float configured_lever_x_mm) {
    const float true_lever_x = 15.0f;
    const float omega = 3.0f;
    Ekf6Params params = provisional_ekf6_params();
    params.optical_lever_x_mm = configured_lever_x_mm;
    Ekf6 filter(GEOMETRY, params);
    Vec<6> truth = make_state(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, omega);
    filter.reset(truth, make_diagonal(1.0f, 1e-4f, 25.0f, 1e-2f));

    for (int k = 1; k <= 200; ++k) {
        truth = Ekf6::transition(truth, DT);
        filter.process_tick(wheel_tick(filter, params, truth, false));
        // The sensor sees omega x r: 3 rad/s * 15 mm = 45 mm/s sideways.
        filter.update_optical(0.0f, omega * true_lever_x, k * DT_US);
        filter.update_heading(truth[Ekf6::THETA], k * DT_US);
    }
    return filter;
}

void test_lever_arm_removes_the_phantom_velocity_of_a_spinning_robot(void) {
    // Correct lever arm: the 45 mm/s the sensor sees is explained by the
    // rotation, the centre does not move, every optical sample is accepted.
    const Ekf6 good = spin_with_offset_sensor(15.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, good.state()[Ekf6::VX]);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 0.0f, good.state()[Ekf6::VY]);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 3.0f, good.state()[Ekf6::OMEGA]);
    TEST_ASSERT_EQUAL_UINT32(0, good.optical_stats().rejected);

    // Lever arm left at zero: the filter either believes in a sideways velocity
    // that does not exist, or starts rejecting a perfectly healthy sensor.
    // Either way, 15 mm of error is not noise, it is a systematic bias.
    const Ekf6 bad = spin_with_offset_sensor(0.0f);
    TEST_ASSERT_TRUE(fabsf(bad.state()[Ekf6::VY]) > 5.0f || bad.optical_stats().rejected > 0);
}

// === COVARIANCE HEALTH ===

void test_health_ignores_position_growth_but_catches_velocity_blow_up(void) {
    const Ekf6Params params = provisional_ekf6_params();
    Ekf6 filter(GEOMETRY, params);
    Vec<6> truth = make_state(0.0f, 0.0f, 0.0f, 200.0f, 0.0f, 0.2f);
    filter.reset(truth, make_diagonal(0.0f, 1e-4f, 25.0f, 1e-3f));

    float last_xx = 0.0f;
    for (int k = 1; k <= 6000; ++k) {
        truth = Ekf6::transition(truth, DT);
        filter.process_tick(wheel_tick(filter, params, truth, false));
        filter.update_optical(truth[Ekf6::VX], truth[Ekf6::VY], k * DT_US);
        filter.update_heading(truth[Ekf6::THETA], k * DT_US);
        if (k % 1000 == 0) {
            // x keeps growing for a whole minute, and that is healthy.
            TEST_ASSERT_TRUE(filter.covariance()(Ekf6::X, Ekf6::X) > last_xx);
            TEST_ASSERT_TRUE(filter.covariance_healthy());
            last_xx = filter.covariance()(Ekf6::X, Ekf6::X);
        }
    }

    // Now every sensor goes silent: the velocities are no longer observed and
    // their variance must trip the health check within a couple of seconds.
    for (int k = 0; k < 200; ++k) {
        filter.predict(DT);
    }
    TEST_ASSERT_FALSE(filter.covariance_healthy());
}

// === CONSISTENCY ===

void test_monte_carlo_filter_is_consistent(void) {
    // The truth follows the filter's own model exactly: process noise drawn
    // from Q, measurements drawn from R. Then, and only then, the filter must
    // be consistent on every source and every state.
    const Ekf6Params params = provisional_ekf6_params();
    const int runs = 400;
    const int ticks = 300;

    NisStats wheels;
    NisStats optical;
    NisStats heading;
    int inside[6] = {0, 0, 0, 0, 0, 0};
    int per_tick_samples = 0;

    for (int run = 0; run < runs; ++run) {
        Ekf6 filter(GEOMETRY, params);
        Vec<6> truth = make_state(0.0f, 0.0f, 0.0f, 150.0f, 50.0f, 0.5f);
        filter.reset(truth, Mat<6>::zero());

        for (int k = 1; k <= ticks; ++k) {
            const dense::DMat<6, 6> L = dense::cholesky(dense::from(filter.process_noise(truth, DT)));
            double n[6];
            for (int i = 0; i < 6; ++i) {
                n[i] = gaussian(1.0);
            }
            truth = Ekf6::transition(truth, DT);
            for (int i = 0; i < 6; ++i) {
                double w = 0.0;
                for (int j = 0; j <= i; ++j) {
                    w += L(i, j) * n[j];
                }
                truth[i] += static_cast<float>(w);
            }
            truth[Ekf6::THETA] = wrap_pi(truth[Ekf6::THETA]);

            filter.process_tick(wheel_tick(filter, params, truth, true));
            filter.update_optical(
                truth[Ekf6::VX] + static_cast<float>(gaussian(params.optical_sigma_mm_s)),
                truth[Ekf6::VY] + static_cast<float>(gaussian(params.optical_sigma_mm_s)),
                k * DT_US);
            filter.update_heading(
                wrap_pi(truth[Ekf6::THETA] + static_cast<float>(gaussian(params.heading_sigma_rad))),
                k * DT_US);

            // Per-tick coverage for the observed states.
            for (size_t i = Ekf6::THETA; i < 6; ++i) {
                const float err = i == Ekf6::THETA
                                      ? angle_innovation(truth[i], filter.state()[i])
                                      : truth[i] - filter.state()[i];
                if (fabsf(err) <= 2.0f * sqrtf(filter.covariance()(i, i))) {
                    ++inside[i];
                }
            }
            ++per_tick_samples;
        }

        // Position errors are correlated along a run: one sample per run.
        for (size_t i = Ekf6::X; i <= Ekf6::Y; ++i) {
            if (fabsf(truth[i] - filter.state()[i]) <= 2.0f * sqrtf(filter.covariance()(i, i))) {
                ++inside[i];
            }
        }

        const NisStats *sources[3] = {&filter.wheel_stats(), &filter.optical_stats(),
                                      &filter.heading_stats()};
        NisStats *totals[3] = {&wheels, &optical, &heading};
        for (int s = 0; s < 3; ++s) {
            totals[s]->accepted += sources[s]->accepted;
            totals[s]->rejected += sources[s]->rejected;
            totals[s]->nis_sum += sources[s]->nis_sum;
        }
    }

    // 0.93 is the mean of a chi-square(1) truncated by the 99 % gate.
    TEST_ASSERT_FLOAT_WITHIN(0.07f, 0.93f, wheels.mean_nis());
    TEST_ASSERT_FLOAT_WITHIN(0.07f, 0.93f, optical.mean_nis());
    TEST_ASSERT_FLOAT_WITHIN(0.07f, 0.93f, heading.mean_nis());

    for (size_t i = Ekf6::THETA; i < 6; ++i) {
        TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.954f, static_cast<float>(inside[i]) / per_tick_samples);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.04f, 0.954f, static_cast<float>(inside[Ekf6::X]) / runs);
    TEST_ASSERT_FLOAT_WITHIN(0.04f, 0.954f, static_cast<float>(inside[Ekf6::Y]) / runs);
}

// === FAILURES SEEN IN THE FIELD ===

struct Scenario {
    /** Optical reading / true velocity: 0.9 is a 10 % under-count. */
    float optical_scale = 1.0f;
    /** Ticks during which the optical sensor reads zero (lost tracking). */
    int dropout_start = -1;
    int dropout_ticks = 0;
    /** Drift of the IMU heading, rad/s. */
    float heading_drift_rad_s = 0.0f;
};

struct Outcome {
    NisStats wheels;
    NisStats optical;
    NisStats heading;
    /**
     * Gate outcome on the FORWARD optical axis alone. The robot drives along x,
     * so an under-count shows there; optical_stats mixes both axes and would
     * halve the rate.
     */
    uint32_t forward_samples = 0;
    uint32_t forward_rejected = 0;
    uint32_t windows = 0;
    uint32_t windows_out = 0;
    bool optical_ok_at_dropout_end = true;
    uint32_t consecutive_after_recovery = 0;
    float max_vx_error_during_dropout = 0.0f;
    float final_theta_error = 0.0f;
};

/**
 * 10 s on a 1 m radius circle at 200 mm/s, measurements with the filter's own
 * noise, one defect injected. The truth itself is noise-free.
 */
static Outcome run_scenario(const Scenario &scenario) {
    const Ekf6Params params = provisional_ekf6_params();
    Ekf6 filter(GEOMETRY, params);
    Vec<6> truth = make_state(0.0f, 0.0f, 0.0f, 200.0f, 0.0f, 0.2f);
    filter.reset(truth, make_diagonal(1.0f, 1e-4f, 25.0f, 1e-3f));

    Outcome out;
    const int dropout_end = scenario.dropout_start + scenario.dropout_ticks;
    for (int k = 1; k <= 1000; ++k) {
        truth = Ekf6::transition(truth, DT);
        filter.process_tick(wheel_tick(filter, params, truth, true));

        const bool dropout = k >= scenario.dropout_start && k < dropout_end;
        const float sigma = params.optical_sigma_mm_s;
        const float ox = dropout ? 0.0f
                                 : scenario.optical_scale * truth[Ekf6::VX] +
                                       static_cast<float>(gaussian(sigma));
        const float oy = dropout ? 0.0f
                                 : scenario.optical_scale * truth[Ekf6::VY] +
                                       static_cast<float>(gaussian(sigma));
        const OpticalUpdate optical = filter.update_optical(ox, oy, k * DT_US);
        ++out.forward_samples;
        if (!optical.x.accepted) {
            ++out.forward_rejected;
        }

        const float drift = scenario.heading_drift_rad_s * k * DT;
        filter.update_heading(
            wrap_pi(truth[Ekf6::THETA] + drift +
                    static_cast<float>(gaussian(params.heading_sigma_rad))),
            k * DT_US);

        if (dropout) {
            out.max_vx_error_during_dropout =
                fmaxf(out.max_vx_error_during_dropout,
                      fabsf(truth[Ekf6::VX] - filter.state()[Ekf6::VX]));
            if (k == dropout_end - 1) {
                out.optical_ok_at_dropout_end = filter.optical_ok();
            }
        }
        if (k == dropout_end + 5) {
            out.consecutive_after_recovery = filter.optical_consecutive_rejections();
        }
    }

    out.wheels = filter.wheel_stats();
    out.optical = filter.optical_stats();
    out.heading = filter.heading_stats();
    out.windows = filter.optical_ratio_windows();
    out.windows_out = filter.optical_ratio_windows_out_of_band();
    out.final_theta_error = angle_innovation(filter.state()[Ekf6::THETA], truth[Ekf6::THETA]);
    return out;
}

static float forward_rejection_rate(const Outcome &out) {
    return static_cast<float>(out.forward_rejected) / static_cast<float>(out.forward_samples);
}

void test_healthy_optical_sensor_is_never_flagged(void) {
    const Outcome out = run_scenario(Scenario());
    TEST_ASSERT_EQUAL_UINT32(10, out.windows);
    TEST_ASSERT_EQUAL_UINT32(0, out.windows_out);
    TEST_ASSERT_TRUE(forward_rejection_rate(out) < 0.03f);
}

void test_five_percent_under_count_is_NOT_detected(void) {
    // The limit, written down: the ratio band is +-7 %, so a 5 % under-count
    // passes as normal and biases the velocity by a few percent.
    Scenario scenario;
    scenario.optical_scale = 0.95f;
    const Outcome out = run_scenario(scenario);
    TEST_ASSERT_TRUE(out.windows_out <= 1);
}

void test_ten_percent_under_count_passes_the_gate_but_not_the_ratio(void) {
    // The reason the ratio exists: at 10 % the samples are individually
    // plausible, and the gate lets almost all of them through.
    Scenario scenario;
    scenario.optical_scale = 0.90f;
    const Outcome out = run_scenario(scenario);
    TEST_ASSERT_TRUE(forward_rejection_rate(out) < 0.25f);
    TEST_ASSERT_TRUE(out.windows_out >= 9);
}

void test_twenty_percent_under_count_is_flagged_by_the_ratio(void) {
    Scenario scenario;
    scenario.optical_scale = 0.80f;
    const Outcome out = run_scenario(scenario);
    TEST_ASSERT_EQUAL_UINT32(out.windows, out.windows_out);
}

void test_thirty_percent_under_count_is_caught_by_both(void) {
    Scenario scenario;
    scenario.optical_scale = 0.70f;
    const Outcome out = run_scenario(scenario);
    TEST_ASSERT_TRUE(forward_rejection_rate(out) > 0.8f);
    TEST_ASSERT_EQUAL_UINT32(out.windows, out.windows_out);
}

void test_brief_loss_of_tracking_is_rejected_and_the_wheels_carry_on(void) {
    // 200 ms of an optical sensor reading zero while the robot drives at
    // 200 mm/s.
    Scenario scenario;
    scenario.dropout_start = 300;
    scenario.dropout_ticks = 20;
    const Outcome out = run_scenario(scenario);
    TEST_ASSERT_FALSE(out.optical_ok_at_dropout_end);
    TEST_ASSERT_TRUE(out.max_vx_error_during_dropout < 30.0f);
    // Recovered: at most the one ordinary 1 %-gate rejection, not a streak.
    TEST_ASSERT_TRUE(out.consecutive_after_recovery <= 1);
}

void test_slow_heading_drift_is_NOT_detected_and_the_pose_follows_it(void) {
    // The limit, written down: nothing else measures absolute heading, so a
    // gyro drift of 0.008 rad/s is invisible to every NIS and the heading
    // estimate follows the IMU. Only an absolute reference (the LiDAR, or a
    // border recalibration) can catch it.
    Scenario scenario;
    scenario.heading_drift_rad_s = 0.008f;
    const Outcome out = run_scenario(scenario);
    TEST_ASSERT_TRUE(out.heading.mean_nis() < 1.3f);
    TEST_ASSERT_TRUE(out.wheels.mean_nis() < 1.3f);
    TEST_ASSERT_TRUE(fabsf(out.final_theta_error) > 0.5f * 0.08f);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_wheel_rows_match_the_inverse_kinematics);
    RUN_TEST(test_jacobian_matches_finite_differences);
    RUN_TEST(test_predicted_covariance_matches_dense_reference);
    RUN_TEST(test_process_noise_is_positive_definite);

    RUN_TEST(test_constant_body_velocity_goes_straight_along_the_heading);
    RUN_TEST(test_constant_body_velocity_and_rate_draw_a_closed_circle);

    RUN_TEST(test_first_frame_and_bad_dt_are_refused);
    RUN_TEST(test_invalid_encoders_still_predict_but_skip_the_wheels);
    RUN_TEST(test_samples_not_newer_than_the_last_are_dropped);

    RUN_TEST(test_heading_update_crosses_pi_the_short_way);
    RUN_TEST(test_lever_arm_removes_the_phantom_velocity_of_a_spinning_robot);

    RUN_TEST(test_health_ignores_position_growth_but_catches_velocity_blow_up);

    RUN_TEST(test_monte_carlo_filter_is_consistent);

    RUN_TEST(test_healthy_optical_sensor_is_never_flagged);
    RUN_TEST(test_five_percent_under_count_is_NOT_detected);
    RUN_TEST(test_ten_percent_under_count_passes_the_gate_but_not_the_ratio);
    RUN_TEST(test_twenty_percent_under_count_is_flagged_by_the_ratio);
    RUN_TEST(test_thirty_percent_under_count_is_caught_by_both);
    RUN_TEST(test_brief_loss_of_tracking_is_rejected_and_the_wheels_carry_on);
    RUN_TEST(test_slow_heading_drift_is_NOT_detected_and_the_pose_follows_it);

    return UNITY_END();
}
