/*
 * Host tests for the EKF core: angle wrapping, scalar update, gate, NIS.
 *
 * Run with: pio test -e native   (from robot1/teensy_capteur/)
 *
 * The scalar update is the only place where the filter touches its covariance on
 * a measurement, and it is hand-expanded to avoid a matrix inverse. These tests
 * pin it three ways: against the closed-form 1D Kalman filter, against the dense
 * textbook equations in double (test/support/dense_reference.h), and against a
 * joint multi-measurement update, which is what sequential scalar updates must
 * reproduce when R is diagonal. A Monte-Carlo run then checks that a filter built
 * on these primitives is consistent: mean NIS near 1, 2-sigma coverage near 95 %.
 */

#include <math.h>
#include <random>
#include <unity.h>

#include "../support/dense_reference.h"
#include "ekf.h"

using namespace kalman;

void setUp(void) {}
void tearDown(void) {}

// === HELPERS ===

static constexpr float DEG = PI_F / 180.0f;

/** Deterministic generator: a failing Monte-Carlo test must fail every time. */
static std::mt19937 &rng() {
    static std::mt19937 generator(20260925u);
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

/** Random symmetric positive-definite matrix: A A^T + eps I. */
template <size_t N>
static Mat<N> random_covariance() {
    float a[N][N];
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            a[i][j] = static_cast<float>(uniform(-2.0, 2.0));
        }
    }
    Mat<N> P = Mat<N>::zero();
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            float sum = 0.0f;
            for (size_t k = 0; k < N; ++k) {
                sum += a[i][k] * a[j][k];
            }
            P(i, j) = sum;
        }
        P(i, i) += 0.1f;
    }
    symmetrize(P);
    return P;
}

template <size_t N>
static Vec<N> random_vector(double lo, double hi) {
    Vec<N> v;
    for (size_t i = 0; i < N; ++i) {
        v[i] = static_cast<float>(uniform(lo, hi));
    }
    return v;
}

/** Asserts a float matrix against a double reference, relative to its scale. */
template <size_t N>
static void assert_matrix_near(const dense::DMat<N, N> &expected, const Mat<N> &actual,
                               double rel_tol) {
    double scale = 0.0;
    for (size_t i = 0; i < N; ++i) {
        scale = fmax(scale, fabs(expected(i, i)));
    }
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            TEST_ASSERT_FLOAT_WITHIN(rel_tol * scale, expected(i, j), actual(i, j));
        }
    }
}

// === ANGLES ===

void test_wrap_pi_keeps_every_angle_in_range(void) {
    const float angles[] = {0.0f,  1.0f,  -1.0f, 3.0f,  -3.0f,  PI_F,
                            -PI_F, 4.0f,  -4.0f, 10.0f, -10.0f, 1000.0f,
                            -1000.0f};
    for (float angle : angles) {
        const float wrapped = wrap_pi(angle);
        TEST_ASSERT_TRUE(wrapped >= -PI_F);
        TEST_ASSERT_TRUE(wrapped < PI_F);
        // Same direction: the difference is a whole number of turns.
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, sinf(wrapped) - sinf(angle));
        TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, cosf(wrapped) - cosf(angle));
    }
}

void test_wrap_pi_maps_plus_pi_to_minus_pi(void) {
    // The interval is half-open: +pi and -pi are the same heading, and only one
    // of them may come out, or two equal headings compare unequal.
    TEST_ASSERT_EQUAL_FLOAT(-PI_F, wrap_pi(PI_F));
    TEST_ASSERT_EQUAL_FLOAT(-PI_F, wrap_pi(-PI_F));
}

void test_wrap_pi_leaves_small_angles_alone(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.5f, wrap_pi(0.5f));
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, wrap_pi(-0.5f));
}

void test_angle_innovation_takes_the_short_way_across_pi(void) {
    // Measured +179 deg, predicted -179 deg: the robot is 2 deg clockwise of the
    // prediction, not 358 deg anticlockwise.
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -2.0f * DEG, angle_innovation(179.0f * DEG, -179.0f * DEG));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f * DEG, angle_innovation(-179.0f * DEG, 179.0f * DEG));
}

// === SCALAR UPDATE ===

void test_scalar_update_matches_closed_form_1d(void) {
    // P = 4, r = 1, innovation 2  ->  s = 5, K = 0.8, x = 1.6, P = 0.8, NIS = 0.8.
    Vec<1> x = Vec<1>::zero();
    Mat<1> P = Mat<1>::zero();
    P(0, 0) = 4.0f;
    Vec<1> h;
    h[0] = 1.0f;

    const UpdateResult result = scalar_update(x, P, h, 2.0f, 1.0f, CHI2_1DOF_99);

    TEST_ASSERT_TRUE(result.accepted);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 5.0f, result.variance);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, result.nis);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.6f, x[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.8f, P(0, 0));
}

void test_scalar_update_matches_dense_reference(void) {
    constexpr size_t N = 6;
    for (int trial = 0; trial < 50; ++trial) {
        Vec<N> x = random_vector<N>(-100.0, 100.0);
        Mat<N> P = random_covariance<N>();
        const Vec<N> h = random_vector<N>(-1.0, 1.0);
        const float innovation = static_cast<float>(uniform(-1.0, 1.0));
        const float r = static_cast<float>(uniform(0.1, 2.0));

        dense::DMat<N, 1> x_ref = dense::from(x);
        dense::DMat<N, N> P_ref = dense::from(P);
        dense::DMat<1, N> H_ref;
        for (size_t j = 0; j < N; ++j) {
            H_ref(0, j) = h[j];
        }
        dense::DMat<1, 1> y_ref;
        y_ref(0, 0) = innovation;
        dense::DMat<1, 1> R_ref;
        R_ref(0, 0) = r;

        const UpdateResult result = scalar_update(x, P, h, innovation, r, INFINITY);
        dense::joint_update(x_ref, P_ref, H_ref, y_ref, R_ref);

        TEST_ASSERT_TRUE(result.accepted);
        for (size_t i = 0; i < N; ++i) {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, x_ref(i, 0), x[i]);
        }
        assert_matrix_near(P_ref, P, 1e-4);
    }
}

void test_sequential_scalar_updates_equal_one_joint_update(void) {
    // This is what licenses scalar updates at all: with a diagonal R, applying
    // the measurements one after the other gives the joint update exactly, and
    // the joint one needs an M x M inverse that the production code never does.
    constexpr size_t N = 6;
    constexpr size_t M = 3;
    for (int trial = 0; trial < 50; ++trial) {
        Vec<N> x = random_vector<N>(-10.0, 10.0);
        Mat<N> P = random_covariance<N>();
        Vec<N> rows[M];
        float z[M];
        float r[M];
        for (size_t k = 0; k < M; ++k) {
            rows[k] = random_vector<N>(-1.0, 1.0);
            z[k] = static_cast<float>(uniform(-5.0, 5.0));
            r[k] = static_cast<float>(uniform(0.1, 2.0));
        }

        dense::DMat<N, 1> x_ref = dense::from(x);
        dense::DMat<N, N> P_ref = dense::from(P);
        dense::DMat<M, N> H_ref;
        dense::DMat<M, 1> y_ref;
        dense::DMat<M, M> R_ref = dense::DMat<M, M>::zero();
        for (size_t k = 0; k < M; ++k) {
            double hx = 0.0;
            for (size_t j = 0; j < N; ++j) {
                H_ref(k, j) = rows[k][j];
                hx += rows[k][j] * x_ref(j, 0);
            }
            y_ref(k, 0) = z[k] - hx;
            R_ref(k, k) = r[k];
        }
        dense::joint_update(x_ref, P_ref, H_ref, y_ref, R_ref);

        for (size_t k = 0; k < M; ++k) {
            // The innovation of each scalar update is taken against the state
            // as updated by the previous ones.
            float hx = 0.0f;
            for (size_t j = 0; j < N; ++j) {
                hx += rows[k][j] * x[j];
            }
            scalar_update(x, P, rows[k], z[k] - hx, r[k], INFINITY);
        }

        for (size_t i = 0; i < N; ++i) {
            TEST_ASSERT_FLOAT_WITHIN(1e-3f, x_ref(i, 0), x[i]);
        }
        assert_matrix_near(P_ref, P, 1e-4);
    }
}

void test_covariance_stays_symmetric_and_positive_over_many_updates(void) {
    constexpr size_t N = 6;
    Vec<N> x = Vec<N>::zero();
    Mat<N> P = random_covariance<N>();
    for (int step = 0; step < 5000; ++step) {
        const Vec<N> h = random_vector<N>(-1.0, 1.0);
        scalar_update(x, P, h, static_cast<float>(gaussian(1.0)), 0.5f, INFINITY);
        // Re-inflate a little, as a prediction would, so P does not collapse to 0.
        for (size_t i = 0; i < N; ++i) {
            P(i, i) += 0.01f;
        }
    }
    for (size_t i = 0; i < N; ++i) {
        TEST_ASSERT_TRUE(P(i, i) > 0.0f);
        for (size_t j = 0; j < N; ++j) {
            TEST_ASSERT_EQUAL_FLOAT(P(i, j), P(j, i));
            // Cauchy-Schwarz, a necessary condition for positive semi-definiteness.
            TEST_ASSERT_TRUE(P(i, j) * P(i, j) <= P(i, i) * P(j, j) * 1.0001f);
        }
    }
}

// === GATE ===

void test_gate_rejects_an_outlier_and_leaves_the_state_untouched(void) {
    constexpr size_t N = 3;
    Vec<N> x = random_vector<N>(-1.0, 1.0);
    Mat<N> P = random_covariance<N>();
    const Vec<N> x_before = x;
    const Mat<N> P_before = P;
    Vec<N> h = Vec<N>::zero();
    h[0] = 1.0f;

    // s = P00 + r; an innovation of 10 sigma has NIS 100, far beyond 6.635.
    const float sigma = sqrtf(P(0, 0) + 1.0f);
    const UpdateResult result = scalar_update(x, P, h, 10.0f * sigma, 1.0f, CHI2_1DOF_99);

    TEST_ASSERT_FALSE(result.accepted);
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 100.0f, result.nis);
    for (size_t i = 0; i < N; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(x_before[i], x[i]);
        for (size_t j = 0; j < N; ++j) {
            TEST_ASSERT_EQUAL_FLOAT(P_before(i, j), P(i, j));
        }
    }
}

void test_gate_accepts_just_inside_the_threshold(void) {
    Vec<1> x = Vec<1>::zero();
    Mat<1> P = Mat<1>::zero();
    P(0, 0) = 1.0f;
    Vec<1> h;
    h[0] = 1.0f;
    // s = 2, NIS = y^2 / 2 = 6.6 < 6.635.
    const UpdateResult result = scalar_update(x, P, h, sqrtf(13.2f), 1.0f, CHI2_1DOF_99);
    TEST_ASSERT_TRUE(result.accepted);
}

void test_nan_innovation_is_rejected_not_accepted(void) {
    Vec<1> x = Vec<1>::zero();
    Mat<1> P = Mat<1>::identity();
    Vec<1> h;
    h[0] = 1.0f;
    const UpdateResult result = scalar_update(x, P, h, NAN, 1.0f, CHI2_1DOF_99);
    TEST_ASSERT_FALSE(result.accepted);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, x[0]);
    TEST_ASSERT_EQUAL_FLOAT(1.0f, P(0, 0));
}

void test_non_positive_variance_is_rejected(void) {
    // A zero covariance and a zero r would divide by zero.
    Vec<1> x = Vec<1>::zero();
    Mat<1> P = Mat<1>::zero();
    Vec<1> h;
    h[0] = 1.0f;
    const UpdateResult result = scalar_update(x, P, h, 1.0f, 0.0f, INFINITY);
    TEST_ASSERT_FALSE(result.accepted);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, x[0]);
}

// === NIS STATISTICS ===

void test_nis_stats_average_accepted_and_count_rejected(void) {
    NisStats stats;
    TEST_ASSERT_EQUAL_FLOAT(0.0f, stats.mean_nis());

    UpdateResult result = {0.0f, 1.0f, 0.5f, true};
    stats.record(result);
    result.nis = 1.5f;
    stats.record(result);
    result.nis = 50.0f;
    result.accepted = false;
    stats.record(result);

    TEST_ASSERT_EQUAL_UINT32(2, stats.accepted);
    TEST_ASSERT_EQUAL_UINT32(1, stats.rejected);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, stats.mean_nis());

    stats.reset();
    TEST_ASSERT_EQUAL_UINT32(0, stats.accepted);
    TEST_ASSERT_EQUAL_UINT32(0, stats.rejected);
}

// === CONSISTENCY ===

void test_consistent_random_walk_filter_has_nis_near_one(void) {
    // Truth is a 1D random walk observed with noise; the filter uses the true q
    // and r. Its NIS must then average about 1 (0.93 after the 99 % gate
    // truncation) and its 2-sigma interval must contain the truth about 95 % of
    // the time. If the primitives were wrong, a filter could still track the
    // truth well while failing both, which is exactly what RMSE cannot see.
    const double q = 0.04;  // process noise variance per step
    const double r = 1.0;   // measurement noise variance
    const int steps = 20000;

    double truth = 0.0;
    Vec<1> x = Vec<1>::zero();
    Mat<1> P = Mat<1>::identity();
    Vec<1> h;
    h[0] = 1.0f;
    NisStats stats;
    int inside_two_sigma = 0;

    for (int k = 0; k < steps; ++k) {
        truth += gaussian(sqrt(q));
        P(0, 0) += static_cast<float>(q);

        const double z = truth + gaussian(sqrt(r));
        stats.record(scalar_update(x, P, h, static_cast<float>(z - x[0]),
                                   static_cast<float>(r), CHI2_1DOF_99));

        if (fabs(x[0] - truth) <= 2.0 * sqrt(P(0, 0))) {
            ++inside_two_sigma;
        }
    }

    const float coverage = static_cast<float>(inside_two_sigma) / steps;
    TEST_ASSERT_FLOAT_WITHIN(0.07f, 0.93f, stats.mean_nis());
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.954f, coverage);
    // One good measurement in a hundred is rejected by construction.
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.01f, static_cast<float>(stats.rejected) / steps);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_wrap_pi_keeps_every_angle_in_range);
    RUN_TEST(test_wrap_pi_maps_plus_pi_to_minus_pi);
    RUN_TEST(test_wrap_pi_leaves_small_angles_alone);
    RUN_TEST(test_angle_innovation_takes_the_short_way_across_pi);

    RUN_TEST(test_scalar_update_matches_closed_form_1d);
    RUN_TEST(test_scalar_update_matches_dense_reference);
    RUN_TEST(test_sequential_scalar_updates_equal_one_joint_update);
    RUN_TEST(test_covariance_stays_symmetric_and_positive_over_many_updates);

    RUN_TEST(test_gate_rejects_an_outlier_and_leaves_the_state_untouched);
    RUN_TEST(test_gate_accepts_just_inside_the_threshold);
    RUN_TEST(test_nan_innovation_is_rejected_not_accepted);
    RUN_TEST(test_non_positive_variance_is_rejected);

    RUN_TEST(test_nis_stats_average_accepted_and_count_rejected);

    RUN_TEST(test_consistent_random_walk_filter_has_nis_near_one);

    return UNITY_END();
}
