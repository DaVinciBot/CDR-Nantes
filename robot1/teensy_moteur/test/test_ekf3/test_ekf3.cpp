/*
 * Host tests for the kinematics and the 3-state EKF.
 *
 * Run with: pio test -e native   (from robot1/teensy_moteur/)
 *
 * Three layers, each checked against something independent of itself:
 *   - the kinematics against the motor board's convention and against its own
 *     inverse;
 *   - the hand-expanded prediction against the dense F P F^T + G Sigma G^T, and
 *     the Jacobians F and G against finite differences of the transition;
 *   - the whole filter against a Monte-Carlo truth: heading NIS near 1 and
 *     2-sigma coverage near 95 % on x, y and theta. That last check is what
 *     tells a consistent filter from one that merely tracks well.
 */

#include <math.h>
#include <random>
#include <unity.h>

#include "../support/dense_reference.h"
#include "ekf3.h"

using namespace kalman;

void setUp(void) {}
void tearDown(void) {}

// === FIXTURE ===

static constexpr float DEG = PI_F / 180.0f;

/** Geometry of the 2026 base: R = 160 mm, 60 mm wheels, 16384 counts/rev. */
static const Geometry GEOMETRY = {160.0f, 60.0f * PI_F / 16384.0f};

static const Ekf3Params PARAMS = {
    10.0f,  // wheel_sigma_counts
    0.02f,  // wheel_slip_ratio
    0.01f,  // heading_sigma_rad
};

static std::mt19937 &rng() {
    static std::mt19937 generator(20260926u);
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

/** Encoder counts a perfect robot would report for a displacement. */
static void exact_counts(const BodyDelta &u, int32_t counts[3]) {
    float travel[3];
    inverse_kinematics(GEOMETRY, u, travel);
    for (int k = 0; k < 3; ++k) {
        counts[k] = static_cast<int32_t>(lroundf(travel[k] / GEOMETRY.counts_to_mm));
    }
}

static OdometryTick make_tick(const int32_t counts[3]) {
    OdometryTick tick;
    tick.d1 = counts[0];
    tick.d2 = counts[1];
    tick.d3 = counts[2];
    tick.dt_us = 10000;
    tick.first_frame = false;
    tick.encoders_valid = true;
    return tick;
}

// === KINEMATICS ===

void test_pure_forward_motion_matches_the_motor_convention(void) {
    // holonomic_basis.cpp: w1 = -0.5 vx, w2 = -0.5 vx, w3 = vx.
    const BodyDelta u = {10.0f, 0.0f, 0.0f};
    float travel[3];
    inverse_kinematics(GEOMETRY, u, travel);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -5.0f, travel[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -5.0f, travel[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f, travel[2]);
}

void test_pure_rotation_turns_every_wheel_backwards(void) {
    // holonomic_basis.cpp: w_i = ... - omega R, so a positive rotation is
    // negative travel on all three wheels, and back.
    const BodyDelta u = {0.0f, 0.0f, 0.01f};
    float travel[3];
    inverse_kinematics(GEOMETRY, u, travel);
    for (int k = 0; k < 3; ++k) {
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.6f, travel[k]);
    }

    const int32_t n = 1000;
    const BodyDelta back = forward_kinematics(GEOMETRY, -n, -n, -n);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, back.dx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, back.dy);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, n * GEOMETRY.counts_to_mm / GEOMETRY.robot_radius_mm,
                             back.dtheta);
}

void test_forward_kinematics_inverts_inverse_kinematics(void) {
    for (int trial = 0; trial < 200; ++trial) {
        const BodyDelta u = {static_cast<float>(uniform(-5.0, 5.0)),
                             static_cast<float>(uniform(-5.0, 5.0)),
                             static_cast<float>(uniform(-0.05, 0.05))};
        int32_t counts[3];
        exact_counts(u, counts);
        const BodyDelta back = forward_kinematics(GEOMETRY, counts[0], counts[1], counts[2]);
        // Only the rounding to whole counts separates them: half a count per
        // wheel, about 0.006 mm.
        TEST_ASSERT_FLOAT_WITHIN(0.01f, u.dx, back.dx);
        TEST_ASSERT_FLOAT_WITHIN(0.01f, u.dy, back.dy);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, u.dtheta, back.dtheta);
    }
}

void test_body_delta_covariance_matches_dense_reference(void) {
    const float variance[3] = {0.04f, 0.09f, 0.01f};
    const double R = GEOMETRY.robot_radius_mm;
    const double s3 = sqrt(3.0);
    dense::DMat<3, 3> J;
    const double rows[3][3] = {
        {-1.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0},
        {1.0 / s3, -1.0 / s3, 0.0},
        {-1.0 / (3.0 * R), -1.0 / (3.0 * R), -1.0 / (3.0 * R)},
    };
    dense::DMat<3, 3> W = dense::DMat<3, 3>::zero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            J(i, j) = rows[i][j];
        }
        W(i, i) = variance[i];
    }
    const dense::DMat<3, 3> expected = dense::mul(dense::mul(J, W), dense::transpose(J));

    const Mat<3> actual = body_delta_covariance(GEOMETRY, variance);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            TEST_ASSERT_FLOAT_WITHIN(1e-6f * (1.0f + fabsf(static_cast<float>(expected(i, j)))),
                                     expected(i, j), actual(i, j));
        }
    }
}

// === MEAN PROPAGATION ===

void test_straight_line_follows_the_heading(void) {
    Ekf3 filter(GEOMETRY, PARAMS);
    filter.reset(0.0f, 0.0f, 0.5f * PI_F, 0.0f, 0.0f);
    const BodyDelta u = {2.0f, 0.0f, 0.0f};
    for (int k = 0; k < 100; ++k) {
        filter.predict(u, Mat<3>::zero());
    }
    // Facing +y, forward is +y.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, filter.state()[Ekf3::X]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 200.0f, filter.state()[Ekf3::Y]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f * PI_F, filter.state()[Ekf3::THETA]);
}

void test_circle_passes_the_far_side_and_closes(void) {
    // 1000 ticks of the same arc make one full turn of radius 2 / (2 pi / 1000).
    const int ticks = 1000;
    const float step = 2.0f;
    const float dtheta = TWO_PI_F / ticks;
    const float radius = step / dtheta;
    const BodyDelta u = {step, 0.0f, dtheta};

    Ekf3 filter(GEOMETRY, PARAMS);
    for (int k = 0; k < ticks / 2; ++k) {
        filter.predict(u, Mat<3>::zero());
    }
    // Starting at the origin facing +x and turning towards +y, the far side of
    // the circle is (0, 2r).
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, filter.state()[Ekf3::X]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.0f * radius, filter.state()[Ekf3::Y]);

    for (int k = ticks / 2; k < ticks; ++k) {
        filter.predict(u, Mat<3>::zero());
    }
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, filter.state()[Ekf3::X]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, filter.state()[Ekf3::Y]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, sinf(filter.state()[Ekf3::THETA]));
}

void test_pure_rotation_wraps_the_heading_without_moving(void) {
    Ekf3 filter(GEOMETRY, PARAMS);
    filter.reset(0.0f, 0.0f, 3.0f, 0.0f, 0.0f);
    const BodyDelta u = {0.0f, 0.0f, 0.1f};
    for (int k = 0; k < 40; ++k) {
        filter.predict(u, Mat<3>::zero());
        TEST_ASSERT_TRUE(filter.state()[Ekf3::THETA] >= -PI_F);
        TEST_ASSERT_TRUE(filter.state()[Ekf3::THETA] < PI_F);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, filter.state()[Ekf3::X]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, filter.state()[Ekf3::Y]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, wrap_pi(7.0f), filter.state()[Ekf3::THETA]);
}

// === LINEARISATION ===

void test_jacobians_match_finite_differences(void) {
    Vec<3> state;
    state[0] = 10.0f;
    state[1] = -20.0f;
    state[2] = 0.7f;
    const BodyDelta u = {5.0f, -3.0f, 0.2f};
    Mat<3> F;
    Mat<3> G;
    Ekf3::jacobians(state, u, F, G);

    const float h = 1e-2f;
    for (int col = 0; col < 3; ++col) {
        Vec<3> plus = state;
        Vec<3> minus = state;
        plus[col] += h;
        minus[col] -= h;
        const Vec<3> fp = Ekf3::transition(plus, u);
        const Vec<3> fm = Ekf3::transition(minus, u);
        for (int row = 0; row < 3; ++row) {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, F(row, col), (fp[row] - fm[row]) / (2.0f * h));
        }
    }

    for (int col = 0; col < 3; ++col) {
        BodyDelta up = u;
        BodyDelta um = u;
        float *pp = col == 0 ? &up.dx : col == 1 ? &up.dy : &up.dtheta;
        float *pm = col == 0 ? &um.dx : col == 1 ? &um.dy : &um.dtheta;
        *pp += h;
        *pm -= h;
        const Vec<3> fp = Ekf3::transition(state, up);
        const Vec<3> fm = Ekf3::transition(state, um);
        for (int row = 0; row < 3; ++row) {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, G(row, col), (fp[row] - fm[row]) / (2.0f * h));
        }
    }
}

void test_predicted_covariance_matches_dense_reference(void) {
    for (int trial = 0; trial < 50; ++trial) {
        // Random correlated covariance, and a random displacement noise.
        Mat<3> P = Mat<3>::zero();
        Mat<3> sigma_u = Mat<3>::zero();
        float a[3][3];
        float b[3][3];
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                a[i][j] = static_cast<float>(uniform(-3.0, 3.0));
                b[i][j] = static_cast<float>(uniform(-0.1, 0.1));
            }
        }
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                for (int k = 0; k < 3; ++k) {
                    P(i, j) += a[i][k] * a[j][k];
                    sigma_u(i, j) += b[i][k] * b[j][k];
                }
            }
        }
        Vec<3> state;
        state[0] = static_cast<float>(uniform(-1000.0, 1000.0));
        state[1] = static_cast<float>(uniform(-1000.0, 1000.0));
        state[2] = static_cast<float>(uniform(-3.0, 3.0));
        const BodyDelta u = {static_cast<float>(uniform(-5.0, 5.0)),
                             static_cast<float>(uniform(-5.0, 5.0)),
                             static_cast<float>(uniform(-0.1, 0.1))};

        Mat<3> F;
        Mat<3> G;
        Ekf3::jacobians(state, u, F, G);
        const dense::DMat<3, 3> dF = dense::from(F);
        const dense::DMat<3, 3> dG = dense::from(G);
        const dense::DMat<3, 3> Q =
            dense::mul(dense::mul(dG, dense::from(sigma_u)), dense::transpose(dG));
        const dense::DMat<3, 3> expected =
            dense::predict_covariance(dense::from(P), dF, Q);

        Ekf3 filter(GEOMETRY, PARAMS);
        filter.reset(state, P);
        filter.predict(u, sigma_u);

        // Tolerance relative to the largest element: an off-diagonal term near
        // zero can be the difference of large ones, and carries their rounding.
        double scale = 1.0;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                scale = fmax(scale, fabs(expected(i, j)));
            }
        }
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                TEST_ASSERT_FLOAT_WITHIN(1e-5 * scale, expected(i, j),
                                         filter.covariance()(i, j));
            }
        }
    }
}

// === MEASUREMENT ===

void test_heading_update_crosses_pi_the_short_way(void) {
    // Predicted +179 deg, measured -179 deg, the measurement much more precise:
    // the innovation is +2 deg, and the heading must move ACROSS +-pi towards
    // -179 deg, not swing back through 0.
    Ekf3 filter(GEOMETRY, {10.0f, 0.02f, 1.0f * DEG});
    filter.reset(0.0f, 0.0f, 179.0f * DEG, 0.0f, 5.0f * DEG);
    const UpdateResult result = filter.update_heading(-179.0f * DEG);

    TEST_ASSERT_TRUE(result.accepted);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f * DEG, result.innovation);
    TEST_ASSERT_TRUE(fabsf(filter.state()[Ekf3::THETA]) > 179.0f * DEG);
    TEST_ASSERT_TRUE(filter.covariance()(Ekf3::THETA, Ekf3::THETA) < (5.0f * DEG) * (5.0f * DEG));
}

// === TICKS ===

void test_first_frame_and_invalid_encoders_are_skipped(void) {
    Ekf3 filter(GEOMETRY, PARAMS);
    const int32_t counts[3] = {-4000, -4000, 8000};
    OdometryTick tick = make_tick(counts);

    tick.first_frame = true;
    TEST_ASSERT_FALSE(filter.predict_tick(tick));
    tick.first_frame = false;
    tick.encoders_valid = false;
    TEST_ASSERT_FALSE(filter.predict_tick(tick));

    TEST_ASSERT_EQUAL_UINT32(2, filter.ticks_skipped());
    TEST_ASSERT_EQUAL_UINT32(0, filter.ticks_used());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, filter.state()[Ekf3::X]);

    tick.encoders_valid = true;
    TEST_ASSERT_TRUE(filter.predict_tick(tick));
    TEST_ASSERT_TRUE(filter.state()[Ekf3::X] > 0.0f);
}

void test_tick_duration_does_not_change_the_result(void) {
    // The model is driven by displacement: a late tick must give the same pose.
    const int32_t counts[3] = {-4000, -3000, 8000};
    OdometryTick on_time = make_tick(counts);
    OdometryTick late = make_tick(counts);
    late.dt_us = 500000;

    Ekf3 a(GEOMETRY, PARAMS);
    Ekf3 b(GEOMETRY, PARAMS);
    a.predict_tick(on_time);
    b.predict_tick(late);
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(a.state()[i], b.state()[i]);
        TEST_ASSERT_EQUAL_FLOAT(a.covariance()(i, i), b.covariance()(i, i));
    }
}

void test_position_variance_keeps_growing_with_no_guard(void) {
    // No absolute position measurement: x and y are unobservable and their
    // variance must grow for the whole run, heading updates or not.
    Ekf3 filter(GEOMETRY, PARAMS);
    const BodyDelta u = {2.0f, 0.5f, 0.003f};
    int32_t counts[3];
    exact_counts(u, counts);
    const OdometryTick tick = make_tick(counts);

    float last_xx = 0.0f;
    float last_yy = 0.0f;
    for (int k = 1; k <= 10000; ++k) {
        filter.predict_tick(tick);
        filter.update_heading(filter.state()[Ekf3::THETA]);
        if (k % 1000 == 0) {
            const float xx = filter.covariance()(Ekf3::X, Ekf3::X);
            const float yy = filter.covariance()(Ekf3::Y, Ekf3::Y);
            TEST_ASSERT_TRUE(xx > last_xx);
            TEST_ASSERT_TRUE(yy > last_yy);
            last_xx = xx;
            last_yy = yy;
        }
    }
}

// === CONSISTENCY ===

void test_monte_carlo_filter_is_consistent(void) {
    // Truth moves with a known displacement per tick. The encoders report it
    // with exactly the noise the filter assumes, the IMU reports the heading
    // with exactly its assumed noise. The filter must then be consistent: heading
    // NIS near 1 (0.93 after the gate truncation), and the 2-sigma interval must
    // hold the truth about 95 % of the time on x, y and theta.
    const int runs = 400;
    const int ticks = 300;
    const float floor_mm = PARAMS.wheel_sigma_counts * GEOMETRY.counts_to_mm;

    int theta_inside = 0;
    int theta_samples = 0;
    int x_inside = 0;
    int y_inside = 0;
    NisStats heading;

    for (int run = 0; run < runs; ++run) {
        Ekf3 filter(GEOMETRY, PARAMS);
        Vec<3> truth = Vec<3>::zero();

        for (int k = 0; k < ticks; ++k) {
            const BodyDelta u = {2.0f, 0.5f, 0.004f * sinf(0.02f * k + run)};
            truth = Ekf3::transition(truth, u);

            float travel[3];
            inverse_kinematics(GEOMETRY, u, travel);
            int32_t counts[3];
            for (int w = 0; w < 3; ++w) {
                const double slip = PARAMS.wheel_slip_ratio * travel[w];
                const double sigma = sqrt(floor_mm * floor_mm + slip * slip);
                counts[w] = static_cast<int32_t>(
                    lround((travel[w] + gaussian(sigma)) / GEOMETRY.counts_to_mm));
            }
            filter.predict_tick(make_tick(counts));
            filter.update_heading(
                wrap_pi(truth[2] + static_cast<float>(gaussian(PARAMS.heading_sigma_rad))));

            const float err = angle_innovation(truth[2], filter.state()[Ekf3::THETA]);
            if (fabsf(err) <= 2.0f * sqrtf(filter.covariance()(Ekf3::THETA, Ekf3::THETA))) {
                ++theta_inside;
            }
            ++theta_samples;
        }

        // Position errors are strongly correlated along a run: sample them once
        // per run so that every sample is independent.
        const float ex = truth[0] - filter.state()[Ekf3::X];
        const float ey = truth[1] - filter.state()[Ekf3::Y];
        if (fabsf(ex) <= 2.0f * sqrtf(filter.covariance()(Ekf3::X, Ekf3::X))) {
            ++x_inside;
        }
        if (fabsf(ey) <= 2.0f * sqrtf(filter.covariance()(Ekf3::Y, Ekf3::Y))) {
            ++y_inside;
        }
        heading.accepted += filter.heading_stats().accepted;
        heading.rejected += filter.heading_stats().rejected;
        heading.nis_sum += filter.heading_stats().nis_sum;
    }

    TEST_ASSERT_FLOAT_WITHIN(0.07f, 0.93f, heading.mean_nis());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.954f, static_cast<float>(theta_inside) / theta_samples);
    TEST_ASSERT_FLOAT_WITHIN(0.04f, 0.954f, static_cast<float>(x_inside) / runs);
    TEST_ASSERT_FLOAT_WITHIN(0.04f, 0.954f, static_cast<float>(y_inside) / runs);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_pure_forward_motion_matches_the_motor_convention);
    RUN_TEST(test_pure_rotation_turns_every_wheel_backwards);
    RUN_TEST(test_forward_kinematics_inverts_inverse_kinematics);
    RUN_TEST(test_body_delta_covariance_matches_dense_reference);

    RUN_TEST(test_straight_line_follows_the_heading);
    RUN_TEST(test_circle_passes_the_far_side_and_closes);
    RUN_TEST(test_pure_rotation_wraps_the_heading_without_moving);

    RUN_TEST(test_jacobians_match_finite_differences);
    RUN_TEST(test_predicted_covariance_matches_dense_reference);

    RUN_TEST(test_heading_update_crosses_pi_the_short_way);

    RUN_TEST(test_first_frame_and_invalid_encoders_are_skipped);
    RUN_TEST(test_tick_duration_does_not_change_the_result);
    RUN_TEST(test_position_variance_keeps_growing_with_no_guard);

    RUN_TEST(test_monte_carlo_filter_is_consistent);

    return UNITY_END();
}
