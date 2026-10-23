/*
 * Host tests for lib/pose_estimator: the complementary filter, the EKF wrapper,
 * and the hand-over between the two.
 *
 * Run with: pio test -e native   (from robot1/teensy_moteur/)
 *
 * COMPLEMENTARY: the 20/80 filter on the common contract (robot-frame optical
 * step, thresholds as speeds). The 2026 weaknesses (optical blind below
 * 0.2 m/s, outlier bound at full speed, mean heading wrong across +-pi) are
 * gone, and the tests below say so.
 *
 * EKF WRAPPER: the mapping from EstimatorInput to Ekf6 (fresh encoders, IMU,
 * flags, velocity in the table frame). The filter itself is covered by
 * test_ekf6.
 *
 * HAND-OVER: an estimator selected at run time starts from the pose it is given.
 *
 * The tick is the motor board's 15 ms.
 */

#include <math.h>
#include <unity.h>

#include "complementary_estimator.h"
#include "ekf_estimator.h"
#include "kinematics.h"

using namespace pose;

void setUp(void) {}
void tearDown(void) {}

// === FIXTURE ===

static constexpr float PI_F = 3.14159265358979f;
static constexpr uint32_t DT_US = 15000;
static constexpr float DT_S = 0.015f;

/** Geometry of the 2026 base: R = 160 mm, 60 mm wheels, 16384 counts/rev. */
static constexpr float RADIUS_MM = 160.0f;
static constexpr float COUNTS_TO_MM = 60.0f * PI_F / 16384.0f;

static ComplementaryParams complementary_params() {
    ComplementaryParams p;
    p.robot_radius_mm = RADIUS_MM;
    p.counts_to_mm = COUNTS_TO_MM;
    return p;
}

static kalman::Geometry geometry() {
    kalman::Geometry g;
    g.robot_radius_mm = RADIUS_MM;
    g.counts_to_mm = COUNTS_TO_MM;
    return g;
}

/** A first tick: the references are taken, nothing moves. */
static EstimatorInput first_tick() {
    EstimatorInput in = empty_input();
    in.first_frame = true;
    return in;
}

/**
 * A regular tick for a robot-frame displacement (dx, dy, dtheta), encoders
 * fresh, counts rounded as the drivers would give them.
 */
static EstimatorInput motion_tick(float dx, float dy, float dtheta) {
    kalman::BodyDelta delta;
    delta.dx = dx;
    delta.dy = dy;
    delta.dtheta = dtheta;
    float travel[3];
    kalman::inverse_kinematics(geometry(), delta, travel);

    EstimatorInput in = empty_input();
    in.d1 = static_cast<int32_t>(lroundf(travel[0] / COUNTS_TO_MM));
    in.d2 = static_cast<int32_t>(lroundf(travel[1] / COUNTS_TO_MM));
    in.d3 = static_cast<int32_t>(lroundf(travel[2] / COUNTS_TO_MM));
    in.dt_us = DT_US;
    in.encoders_fresh = true;
    in.encoder_age_us = 2000;
    return in;
}

// === COMPLEMENTARY ===

static void test_complementary_first_tick_integrates_nothing(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, 0.0f);

    EstimatorInput in = motion_tick(50.0f, 0.0f, 0.0f);
    in.first_frame = true;
    est.step(in);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.pose().y_mm);
    TEST_ASSERT_EQUAL_UINT8(0, est.health_flags() & FILTER_READY);
}

static void test_complementary_straight_line_on_encoders(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());
    for (int i = 0; i < 100; ++i) {
        est.step(motion_tick(2.0f, 0.0f, 0.0f));
    }
    // Count rounding: under 0.5 count per wheel per tick.
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, est.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, est.pose().y_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.0f, est.pose().theta_rad);
    TEST_ASSERT_TRUE(est.health_flags() & FILTER_READY);
}

static void test_complementary_follows_the_heading(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, PI_F / 2.0f);
    est.step(first_tick());
    for (int i = 0; i < 100; ++i) {
        EstimatorInput in = motion_tick(2.0f, 0.0f, 0.0f);
        in.imu_valid = true;
        in.heading_rad = PI_F / 2.0f;
        est.step(in);
    }
    // Forward in the robot frame is +y in the table at 90 deg.
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, est.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 200.0f, est.pose().y_mm);
}

static void test_complementary_imu_heading_overrides_the_encoders(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());

    EstimatorInput in = motion_tick(0.0f, 0.0f, 0.05f);
    in.imu_valid = true;
    in.heading_rad = 0.5f;
    est.step(in);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.5f, est.pose().theta_rad);
}

static void test_complementary_encoder_rotation_without_imu(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());
    for (int i = 0; i < 50; ++i) {
        est.step(motion_tick(0.0f, 0.0f, 0.01f));
    }
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.5f, est.pose().theta_rad);
}

static void test_complementary_ignores_deltas_under_the_noise_floor(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());

    EstimatorInput in = empty_input();
    in.d1 = 15;
    in.d2 = -15;
    in.d3 = 15;
    in.dt_us = DT_US;
    in.encoders_fresh = true;
    for (int i = 0; i < 100; ++i) {
        est.step(in);
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.pose().y_mm);
}

/** The test firmware runs with use_encoders = false: without optical, the
 *  position does not move. Pinned so that nobody is surprised by it. */
static void test_complementary_without_use_encoders_position_is_frozen(void) {
    ComplementaryParams p = complementary_params();
    p.use_encoders = false;
    ComplementaryEstimator est(p);
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());
    for (int i = 0; i < 100; ++i) {
        est.step(motion_tick(2.0f, 0.0f, 0.01f));
    }
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.pose().y_mm);
    // The heading still integrates.
    TEST_ASSERT_TRUE(est.pose().theta_rad > 0.9f);
}

/** A tick with a new optical sample: robot-frame step over step_dt_us. */
static EstimatorInput with_optical(EstimatorInput in, float dx_mm, float dy_mm,
                                   uint32_t step_dt_us = DT_US) {
    in.optical_step_valid = true;
    in.optical_dx_mm = dx_mm;
    in.optical_dy_mm = dy_mm;
    in.optical_step_dt_us = step_dt_us;
    return in;
}

/**
 * Runs a straight line with optical steps of optical_step_mm per tick and
 * encoder steps of encoder_step_mm, the encoders alone not integrated.
 */
static float fused_x_after(float optical_step_mm, float encoder_step_mm,
                           bool encoders_fresh, int ticks) {
    ComplementaryParams p = complementary_params();
    p.use_encoders = false;
    ComplementaryEstimator est(p);
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());
    for (int i = 0; i < ticks; ++i) {
        EstimatorInput in = motion_tick(encoder_step_mm, 0.0f, 0.0f);
        in.encoders_fresh = encoders_fresh;
        est.step(with_optical(in, optical_step_mm, 0.0f));
    }
    return est.pose().x_mm;
}

static void test_complementary_fuses_20_percent_encoders_80_percent_optical(void) {
    // 0.2 * 2 + 0.8 * 5 = 4.4 mm per tick.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 44.0f, fused_x_after(5.0f, 2.0f, true, 10));
}

static void test_complementary_missed_encoder_read_trusts_the_optical_alone(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, fused_x_after(5.0f, 2.0f, false, 10));
}

static void test_complementary_optical_sees_slow_motion(void) {
    // 1.5 mm per 15 ms tick = 100 mm/s: the 2026 floor (2 mm per tick) zeroed
    // it, the robot was optically blind below 0.2 m/s.
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 15.0f, fused_x_after(1.5f, 1.5f, true, 10));
}

static void test_complementary_optical_under_the_speed_floor_is_noise(void) {
    // 0.03 mm per tick = 2 mm/s, under the 5 mm/s floor; encoders alone off.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, fused_x_after(0.03f, 0.0f, true, 10));
}

static void test_complementary_full_speed_is_not_an_outlier(void) {
    // 1 m/s at 15 ms = 15 mm per tick, which sat right on the 2026 bound.
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 150.0f, fused_x_after(15.0f, 15.0f, true, 10));
}

static void test_complementary_optical_outlier_is_rejected_and_counted(void) {
    ComplementaryParams p = complementary_params();
    p.use_encoders = false;
    ComplementaryEstimator est(p);
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());

    // 30 mm in 15 ms = 2 m/s, above 1.5 m/s.
    EstimatorInput in = empty_input();
    in.dt_us = DT_US;
    est.step(with_optical(in, 30.0f, 0.0f));

    TEST_ASSERT_EQUAL_UINT32(1, est.optical_outlier_count());
    TEST_ASSERT_TRUE(est.health_flags() & MEAS_REJECTED);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, est.pose().x_mm);
}

static void test_complementary_missed_optical_read_is_rescaled_to_the_tick(void) {
    ComplementaryParams p = complementary_params();
    p.use_encoders = false;
    p.alpha_encoders = 0.0;  // optical only, to read its share directly
    ComplementaryEstimator est(p);
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());

    // One tick without an optical sample, then one step covering two ticks:
    // 10 mm in 30 ms is 333 mm/s, i.e. 5 mm over this tick.
    est.step(motion_tick(5.0f, 0.0f, 0.0f));
    est.step(with_optical(motion_tick(5.0f, 0.0f, 0.0f), 10.0f, 0.0f, 2 * DT_US));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 5.0f, est.pose().x_mm);
}

static void test_complementary_pure_rotation_drops_the_optical(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, 0.0f);
    est.step(first_tick());

    for (int i = 1; i <= 10; ++i) {
        EstimatorInput in = empty_input();
        in.d1 = 200;
        in.d2 = 200;
        in.d3 = 200;
        in.dt_us = DT_US;
        in.encoders_fresh = true;
        est.step(with_optical(in, 5.0f, 0.0f));
        TEST_ASSERT_EQUAL_UINT8(0, est.health_flags() & OPTICAL_OK);
    }
    // Equal wheel deltas: no translation from the encoders, optical dropped.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, est.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, est.pose().y_mm);
}

static void test_complementary_mean_heading_across_pi(void) {
    // Heading crosses +-pi during the tick: the robot faces -x all along. The
    // 2026 plain average gave a mean heading of 0, i.e. motion towards +x.
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, PI_F - 0.01f);
    est.step(first_tick());

    EstimatorInput in = motion_tick(10.0f, 0.0f, 0.0f);
    in.imu_valid = true;
    in.heading_rad = -PI_F + 0.01f;
    est.step(in);

    TEST_ASSERT_FLOAT_WITHIN(0.1f, -10.0f, est.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, est.pose().y_mm);
}

static void test_complementary_reports_its_velocity_in_the_table_frame(void) {
    ComplementaryEstimator est(complementary_params());
    est.reset(0.0f, 0.0f, PI_F / 2.0f);
    est.step(first_tick());
    for (int i = 0; i < 10; ++i) {
        EstimatorInput in = motion_tick(3.0f, 0.0f, 0.0f);  // 200 mm/s forward
        in.imu_valid = true;
        in.heading_rad = PI_F / 2.0f;
        est.step(in);
    }
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 0.0f, est.pose().vx_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 200.0f, est.pose().vy_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, est.pose().omega_rad_s);
}

// === EKF WRAPPER ===

/**
 * Straight line from rest, ramped at 1 m/s^2 up to 200 mm/s. A ramp, not a
 * step: 0 to 200 mm/s in one tick is over 10 m/s^2, far outside the process noise,
 * and the gate rightly rejects the wheels until the velocity variance has grown
 * (13 mm of lag measured). The reference is the pose integrated from the very
 * counts fed in, so count rounding does not enter the tolerance.
 */
static void test_ekf_straight_line_on_encoders(void) {
    EkfEstimator ekf(geometry(), kalman::provisional_ekf6_params(DT_S));
    ekf.reset(0.0f, 0.0f, 0.0f);
    ekf.step(first_tick());

    // The rounded counts carry a small constant rotation per tick: the
    // reference integrates the full pose they describe, not a straight line.
    double truth_x = 0.0, truth_y = 0.0, truth_theta = 0.0;
    for (int i = 0; i < 200; ++i) {
        const float speed_mm_s = fminf(200.0f, 15.0f * static_cast<float>(i + 1));
        const EstimatorInput in = motion_tick(speed_mm_s * DT_S, 0.0f, 0.0f);
        ekf.step(in);
        const kalman::BodyDelta d = kalman::forward_kinematics(geometry(), in.d1, in.d2, in.d3);
        const double mid = truth_theta + 0.5 * d.dtheta;
        truth_x += d.dx * cos(mid) - d.dy * sin(mid);
        truth_y += d.dx * sin(mid) + d.dy * cos(mid);
        truth_theta += d.dtheta;
    }
    // About 1 mm of lag: the constant-velocity model trails a ramp.
    TEST_ASSERT_FLOAT_WITHIN(2.0f, static_cast<float>(truth_x), ekf.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, static_cast<float>(truth_y), ekf.pose().y_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, static_cast<float>(truth_theta), ekf.pose().theta_rad);
    TEST_ASSERT_EQUAL_UINT32(0, ekf.filter().wheel_stats().rejected);
    TEST_ASSERT_TRUE(ekf.health_flags() & FILTER_READY);
}

static void test_ekf_stale_encoders_predict_without_wheel_updates(void) {
    EkfEstimator ekf(geometry(), kalman::provisional_ekf6_params(DT_S));
    ekf.reset(0.0f, 0.0f, 0.0f);
    ekf.step(first_tick());
    ekf.step(motion_tick(2.0f, 0.0f, 0.0f));

    const uint32_t accepted = ekf.filter().wheel_stats().accepted;
    const uint32_t used = ekf.filter().ticks_used();

    EstimatorInput in = motion_tick(4.0f, 0.0f, 0.0f);  // two ticks of motion
    in.encoders_fresh = false;
    ekf.step(in);

    TEST_ASSERT_EQUAL_UINT32(accepted, ekf.filter().wheel_stats().accepted);
    TEST_ASSERT_EQUAL_UINT32(used + 1, ekf.filter().ticks_used());
}

static void test_ekf_takes_the_imu_heading(void) {
    EkfEstimator ekf(geometry(), kalman::provisional_ekf6_params(DT_S));
    ekf.reset(0.0f, 0.0f, 0.0f);
    ekf.step(first_tick());
    for (int i = 1; i <= 50; ++i) {
        EstimatorInput in = motion_tick(0.0f, 0.0f, 0.0f);
        in.imu_valid = true;
        in.heading_rad = 0.05f;
        in.imu_timestamp_us = static_cast<uint32_t>(i) * DT_US;
        ekf.step(in);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.05f, ekf.pose().theta_rad);
}

static void test_ekf_optical_ok_needs_an_optical_sample(void) {
    EkfEstimator ekf(geometry(), kalman::provisional_ekf6_params(DT_S));
    ekf.reset(0.0f, 0.0f, 0.0f);
    ekf.step(first_tick());
    ekf.step(motion_tick(2.0f, 0.0f, 0.0f));
    TEST_ASSERT_EQUAL_UINT8(0, ekf.health_flags() & OPTICAL_OK);

    EstimatorInput in = motion_tick(2.0f, 0.0f, 0.0f);
    in.optical_velocity_valid = true;
    in.optical_vx_mm_s = 200.0f;
    in.optical_timestamp_us = 1000;
    ekf.step(in);
    TEST_ASSERT_TRUE(ekf.health_flags() & OPTICAL_OK);
}

static void test_ekf_reports_its_velocity_in_the_table_frame(void) {
    EkfEstimator ekf(geometry(), kalman::provisional_ekf6_params(DT_S));
    ekf.reset(0.0f, 0.0f, PI_F / 2.0f);
    ekf.step(first_tick());
    for (int i = 0; i < 200; ++i) {
        const float speed_mm_s = fminf(200.0f, 15.0f * static_cast<float>(i + 1));
        EstimatorInput in = motion_tick(speed_mm_s * DT_S, 0.0f, 0.0f);
        in.imu_valid = true;
        in.heading_rad = PI_F / 2.0f;
        in.imu_timestamp_us = static_cast<uint32_t>(i + 1) * DT_US;
        ekf.step(in);
    }
    // Forward in the robot frame is +y in the table at 90 deg.
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 0.0f, ekf.pose().vx_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 200.0f, ekf.pose().vy_mm_s);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, ekf.pose().omega_rad_s);
}

// === HAND-OVER ===

static void hand_over(const PoseEstimator &from, PoseEstimator &to) {
    const PoseEstimate p = from.pose();
    to.reset(p.x_mm, p.y_mm, p.theta_rad);
}

static void test_hand_over_keeps_the_pose_both_ways(void) {
    ComplementaryEstimator comp(complementary_params());
    EkfEstimator ekf(geometry(), kalman::provisional_ekf6_params(DT_S));

    comp.reset(1000.0f, 500.0f, 1.0f);
    comp.step(first_tick());
    for (int i = 0; i < 50; ++i) {
        comp.step(motion_tick(2.0f, 1.0f, 0.0f));
    }

    hand_over(comp, ekf);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, comp.pose().x_mm, ekf.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, comp.pose().y_mm, ekf.pose().y_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, comp.pose().theta_rad, ekf.pose().theta_rad);

    // At rest, the new estimator holds the pose it was given.
    for (int i = 0; i < 20; ++i) {
        ekf.step(motion_tick(0.0f, 0.0f, 0.0f));
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, comp.pose().x_mm, ekf.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, comp.pose().y_mm, ekf.pose().y_mm);

    hand_over(ekf, comp);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, ekf.pose().x_mm, comp.pose().x_mm);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, ekf.pose().y_mm, comp.pose().y_mm);
}

static void test_reset_wraps_the_heading(void) {
    ComplementaryEstimator comp(complementary_params());
    EkfEstimator ekf(geometry(), kalman::provisional_ekf6_params(DT_S));
    comp.reset(0.0f, 0.0f, 4.0f);
    ekf.reset(0.0f, 0.0f, 4.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f - 2.0f * PI_F, comp.pose().theta_rad);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 4.0f - 2.0f * PI_F, ekf.pose().theta_rad);
}

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_complementary_first_tick_integrates_nothing);
    RUN_TEST(test_complementary_straight_line_on_encoders);
    RUN_TEST(test_complementary_follows_the_heading);
    RUN_TEST(test_complementary_imu_heading_overrides_the_encoders);
    RUN_TEST(test_complementary_encoder_rotation_without_imu);
    RUN_TEST(test_complementary_ignores_deltas_under_the_noise_floor);
    RUN_TEST(test_complementary_without_use_encoders_position_is_frozen);
    RUN_TEST(test_complementary_fuses_20_percent_encoders_80_percent_optical);
    RUN_TEST(test_complementary_missed_encoder_read_trusts_the_optical_alone);
    RUN_TEST(test_complementary_optical_sees_slow_motion);
    RUN_TEST(test_complementary_optical_under_the_speed_floor_is_noise);
    RUN_TEST(test_complementary_full_speed_is_not_an_outlier);
    RUN_TEST(test_complementary_optical_outlier_is_rejected_and_counted);
    RUN_TEST(test_complementary_missed_optical_read_is_rescaled_to_the_tick);
    RUN_TEST(test_complementary_pure_rotation_drops_the_optical);
    RUN_TEST(test_complementary_mean_heading_across_pi);
    RUN_TEST(test_complementary_reports_its_velocity_in_the_table_frame);

    RUN_TEST(test_ekf_straight_line_on_encoders);
    RUN_TEST(test_ekf_stale_encoders_predict_without_wheel_updates);
    RUN_TEST(test_ekf_takes_the_imu_heading);
    RUN_TEST(test_ekf_optical_ok_needs_an_optical_sample);
    RUN_TEST(test_ekf_reports_its_velocity_in_the_table_frame);

    RUN_TEST(test_hand_over_keeps_the_pose_both_ways);
    RUN_TEST(test_reset_wraps_the_heading);

    return UNITY_END();
}
