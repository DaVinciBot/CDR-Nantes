/*
 * Host tests for lib/motion: from a robot velocity command to the wheel speeds.
 *
 * Run with: pio test -e native   (from robot1/teensy_moteur/)
 *
 * The limits are those of the 2027 base: 350 rpm, 320 rpm/s per wheel, braking
 * at 1 m/s^2, 60 mm wheels, R = 160 mm.
 */

#include <math.h>
#include <unity.h>

#include "kinematics.h"
#include "wheel_command_shaper.h"

using namespace motion;

// Unity is built without double support: compare doubles by hand.
#define ASSERT_NEAR(tol, expected, actual)                                         TEST_ASSERT_TRUE_MESSAGE(fabs((double)(expected) - (double)(actual)) <= (tol),                              #actual " not within " #tol " of " #expected)

void setUp(void) {}
void tearDown(void) {}

// === FIXTURE ===

static constexpr double PI_D = 3.14159265358979323846;
static constexpr double RADIUS_MM = 160.0;
static constexpr double WHEEL_MM = 60.0;
static constexpr double MAX_RPM = 350.0;
static constexpr double ACCEL_RPM_S = 320.0;
static constexpr double ACCEL_MM_S2 = 1000.0;

static MotionParams params_2027() {
    MotionParams p;
    p.robot_radius_mm = RADIUS_MM;
    p.wheel_diameter_mm = WHEEL_MM;
    p.max_wheel_rpm = MAX_RPM;
    p.max_wheel_accel_rpm_s = ACCEL_RPM_S;
    p.max_accel_mm_s2 = ACCEL_MM_S2;
    p.max_alpha_rad_s2 = ACCEL_MM_S2 / RADIUS_MM;
    return p;
}

static const VelocityCommand ZERO = {0.0, 0.0, 0.0};
/** Far enough that the braking cap never binds. */
static constexpr double FAR_MM = 1.0e6;
static constexpr double FAR_RAD = 1.0e6;

static double rpm_to_mm_s(double rpm) { return rpm * PI_D * WHEEL_MM / 60.0; }

/** Robot-frame speed from wheel speeds: the forward kinematics, as speeds. */
static void body_speed(const double wheel_rpm[3], double &vx, double &vy, double &omega) {
    const double w1 = rpm_to_mm_s(wheel_rpm[0]);
    const double w2 = rpm_to_mm_s(wheel_rpm[1]);
    const double w3 = rpm_to_mm_s(wheel_rpm[2]);
    vx = (2.0 * w3 - w1 - w2) / 3.0;
    vy = (w1 - w2) / sqrt(3.0);
    omega = -(w1 + w2 + w3) / (3.0 * RADIUS_MM);
}

// === BRAKING SPEED ===

void test_braking_speed_is_sqrt_2_a_d(void) {
    ASSERT_NEAR(1e-9, 1000.0, braking_speed(500.0, 1000.0));
    ASSERT_NEAR(1e-9, 0.0, braking_speed(0.0, 1000.0));
    ASSERT_NEAR(1e-9, 0.0, braking_speed(-3.0, 1000.0));
    ASSERT_NEAR(1e-9, 0.0, braking_speed(100.0, 0.0));
}

void test_tick_braking_speed_stops_exactly_on_the_distance(void) {
    const double a = 1000.0, d = 50.0, dt = 0.015;
    const double v = braking_speed(d, a, dt);
    // One tick at v, then a stop at a: exactly d.
    ASSERT_NEAR(1e-9, d, v * dt + v * v / (2.0 * a));
    TEST_ASSERT_TRUE(v < braking_speed(d, a));
    // No tick: the continuous formula.
    ASSERT_NEAR(1e-9, braking_speed(d, a), braking_speed(d, a, 0.0));
    ASSERT_NEAR(1e-9, 0.0, braking_speed(0.0, a, dt));
}

// === ACCELERATION LIMIT ===

/** Fastest wheel climbs by exactly ACCEL_RPM_S * dt per tick until it arrives. */
static void check_ramp_slope(double dt_s) {
    WheelCommandShaper shaper(params_2027());
    // Along the axis of wheel 3: it carries the whole command, ~300 rpm target.
    const VelocityCommand v = {rpm_to_mm_s(300.0), 0.0, 0.0};
    const double step_rpm = ACCEL_RPM_S * dt_s;

    double previous = 0.0;
    double target_rpm = 0.0;
    int ticks = 0;
    while (ticks < 1000) {
        const ShapedCommand &c = shaper.step(v, ZERO, 0.0, FAR_MM, FAR_RAD, dt_s);
        ++ticks;
        target_rpm = c.requested_rpm[2];
        const double now = c.wheel_rpm[2];
        if (!c.accel_limited) {
            // Last tick: the remainder, no more than one step.
            ASSERT_NEAR(1e-9, target_rpm, now);
            TEST_ASSERT_TRUE(now - previous <= step_rpm + 1e-9);
            break;
        }
        ASSERT_NEAR(1e-9, step_rpm, now - previous);
        previous = now;
    }
    ASSERT_NEAR(1e-3, 300.0, target_rpm);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ceil(target_rpm / step_rpm)), ticks);
}

void test_ramp_slope_is_320_rpm_s_at_15_ms(void) { check_ramp_slope(0.015); }

void test_ramp_slope_is_320_rpm_s_at_10_ms(void) { check_ramp_slope(0.010); }

void test_ramp_keeps_the_direction(void) {
    WheelCommandShaper shaper(params_2027());
    // Diagonal with some rotation: the three wheels have different targets.
    const VelocityCommand v = {600.0, 350.0, 0.8};
    for (int tick = 0; tick < 200; ++tick) {
        const ShapedCommand &c = shaper.step(v, ZERO, 0.3, FAR_MM, FAR_RAD, 0.015);
        // Every wheel is at the same fraction of its own target.
        const double fraction = c.wheel_rpm[0] / c.requested_rpm[0];
        ASSERT_NEAR(1e-9, fraction, c.wheel_rpm[1] / c.requested_rpm[1]);
        ASSERT_NEAR(1e-9, fraction, c.wheel_rpm[2] / c.requested_rpm[2]);
    }
    // And the target is reached in the end.
    const ShapedCommand &c = shaper.last();
    TEST_ASSERT_FALSE(c.accel_limited);
    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(1e-9, c.requested_rpm[i], c.wheel_rpm[i]);
    }
}

void test_deceleration_is_limited_too(void) {
    WheelCommandShaper shaper(params_2027());
    const VelocityCommand v = {rpm_to_mm_s(300.0), 0.0, 0.0};
    for (int tick = 0; tick < 200; ++tick) shaper.step(v, ZERO, 0.0, FAR_MM, FAR_RAD, 0.015);
    const double cruising = shaper.last().wheel_rpm[2];
    const ShapedCommand &c = shaper.step(ZERO, ZERO, 0.0, FAR_MM, FAR_RAD, 0.015);
    TEST_ASSERT_TRUE(c.accel_limited);
    ASSERT_NEAR(1e-9, cruising - ACCEL_RPM_S * 0.015, c.wheel_rpm[2]);
}

// === NORMALISATION ===

void test_normalisation_caps_the_fastest_wheel_and_keeps_the_direction(void) {
    MotionParams p = params_2027();
    p.max_wheel_accel_rpm_s = 1.0e9;  // isolate the normalisation
    WheelCommandShaper shaper(p);
    const VelocityCommand v = {2000.0, 1500.0, 0.0};
    const ShapedCommand &c = shaper.step(v, ZERO, 0.0, FAR_MM, FAR_RAD, 0.015);

    TEST_ASSERT_TRUE(c.speed_saturated);
    double fastest = 0.0;
    for (int i = 0; i < 3; ++i) fastest = fmax(fastest, fabs(c.wheel_rpm[i]));
    ASSERT_NEAR(1e-9, MAX_RPM, fastest);

    double vx, vy, omega;
    body_speed(c.wheel_rpm, vx, vy, omega);
    ASSERT_NEAR(1e-3, 1500.0 / 2000.0, vy / vx);
    ASSERT_NEAR(1e-6, 0.0, omega);
}

// === FRAMES AND KINEMATICS ===

void test_round_trip_through_forward_kinematics(void) {
    MotionParams p = params_2027();
    p.max_wheel_accel_rpm_s = 1.0e9;
    WheelCommandShaper shaper(p);
    const VelocityCommand v = {300.0, -200.0, 0.5};
    const ShapedCommand &c = shaper.step(v, ZERO, 0.0, FAR_MM, FAR_RAD, 0.015);

    // One second of this command, as encoder counts.
    const double counts_per_rev = 16384.0;
    kinematics::Geometry g;
    g.robot_radius_mm = static_cast<float>(RADIUS_MM);
    g.counts_to_mm = static_cast<float>(PI_D * WHEEL_MM / counts_per_rev);
    int32_t counts[3];
    for (int i = 0; i < 3; ++i) {
        counts[i] = static_cast<int32_t>(lround(c.wheel_rpm[i] / 60.0 * counts_per_rev));
    }
    const kinematics::BodyDelta back =
        kinematics::forward_kinematics(g, counts[0], counts[1], counts[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 300.0f, back.dx);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -200.0f, back.dy);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, back.dtheta);
}

void test_table_command_is_turned_into_the_robot_frame(void) {
    MotionParams p = params_2027();
    p.max_wheel_accel_rpm_s = 1.0e9;
    WheelCommandShaper shaper(p);
    // Heading +90 deg: table +x is robot -y.
    const VelocityCommand v = {500.0, 0.0, 0.0};
    const ShapedCommand &c = shaper.step(v, ZERO, PI_D / 2.0, FAR_MM, FAR_RAD, 0.015);
    double vx, vy, omega;
    body_speed(c.wheel_rpm, vx, vy, omega);
    ASSERT_NEAR(1e-2, 0.0, vx);
    ASSERT_NEAR(1e-2, -500.0, vy);
}

// === BRAKING ===

/**
 * A 1D approach, as on the robot: P loop on the remaining distance, clamped at
 * 1 m/s, robot moving at the speed of its wheels. It must reach the target
 * without passing it.
 */
static void check_approach(double distance_mm, double dt_s, double &peak_mm_s,
                           double &final_error_mm, double &max_overshoot_mm) {
    WheelCommandShaper shaper(params_2027());
    const double kp = 6.0;  // 1/s
    double x = 0.0;
    peak_mm_s = 0.0;
    max_overshoot_mm = 0.0;
    for (int tick = 0; tick < 2000; ++tick) {
        const double error = distance_mm - x;
        double v = kp * error;
        v = fmax(-1000.0, fmin(1000.0, v));
        const VelocityCommand pid = {v, 0.0, 0.0};
        const ShapedCommand &c = shaper.step(pid, ZERO, 0.0, fabs(error), 0.0, dt_s);
        double vx, vy, omega;
        body_speed(c.wheel_rpm, vx, vy, omega);
        x += vx * dt_s;
        peak_mm_s = fmax(peak_mm_s, vx);
        max_overshoot_mm = fmax(max_overshoot_mm, x - distance_mm);
    }
    final_error_mm = distance_mm - x;
}

void test_long_approach_reaches_1_m_s_and_stops_without_overshoot(void) {
    double peak, final_error, overshoot;
    check_approach(2000.0, 0.015, peak, final_error, overshoot);
    ASSERT_NEAR(5.0, 1000.0, peak);
    ASSERT_NEAR(0.5, 0.0, final_error);
    TEST_ASSERT_TRUE(overshoot < 0.5);
}

void test_short_approach_stops_without_overshoot(void) {
    double peak, final_error, overshoot;
    check_approach(150.0, 0.015, peak, final_error, overshoot);
    TEST_ASSERT_TRUE(peak < 1000.0);
    ASSERT_NEAR(0.5, 0.0, final_error);
    TEST_ASSERT_TRUE(overshoot < 0.5);
}

void test_braking_caps_the_position_loop_only(void) {
    MotionParams p = params_2027();
    p.max_wheel_accel_rpm_s = 1.0e9;
    WheelCommandShaper shaper(p);

    // 50 mm left: the position loop is capped at the tick braking speed
    // (~302 mm/s, against 316 for the continuous sqrt(2 a d)).
    const VelocityCommand pid = {800.0, 0.0, 0.0};
    const ShapedCommand &c = shaper.step(pid, ZERO, 0.0, 50.0, FAR_RAD, 0.015);
    TEST_ASSERT_TRUE(c.braking);
    double vx, vy, omega;
    body_speed(c.wheel_rpm, vx, vy, omega);
    ASSERT_NEAR(0.5, braking_speed(50.0, ACCEL_MM_S2, 0.015), vx);

    // A moving target (feed-forward) is not braked by the distance to it.
    shaper.reset();
    const VelocityCommand ff = {600.0, 0.0, 0.0};
    const ShapedCommand &d = shaper.step(ZERO, ff, 0.0, 0.0, 0.0, 0.015);
    TEST_ASSERT_FALSE(d.braking);
    body_speed(d.wheel_rpm, vx, vy, omega);
    ASSERT_NEAR(0.5, 600.0, vx);
}

void test_rotation_is_braked_on_the_heading_error(void) {
    MotionParams p = params_2027();
    p.max_wheel_accel_rpm_s = 1.0e9;
    WheelCommandShaper shaper(p);
    const double alpha = ACCEL_MM_S2 / RADIUS_MM;
    const VelocityCommand pid = {0.0, 0.0, -3.0};
    const ShapedCommand &c = shaper.step(pid, ZERO, 0.0, FAR_MM, 0.1, 0.015);
    TEST_ASSERT_TRUE(c.braking);
    double vx, vy, omega;
    body_speed(c.wheel_rpm, vx, vy, omega);
    ASSERT_NEAR(1e-3, -braking_speed(0.1, alpha, 0.015), omega);
}

// === STATE ===

void test_reset_and_bad_dt(void) {
    WheelCommandShaper shaper(params_2027());
    const VelocityCommand v = {500.0, 0.0, 0.0};
    shaper.step(v, ZERO, 0.0, FAR_MM, FAR_RAD, 0.015);
    const double before = shaper.last().wheel_rpm[2];
    TEST_ASSERT_TRUE(before > 0.0);

    // A non-positive dt changes nothing.
    shaper.step(v, ZERO, 0.0, FAR_MM, FAR_RAD, 0.0);
    ASSERT_NEAR(1e-12, before, shaper.last().wheel_rpm[2]);

    shaper.reset();
    for (int i = 0; i < 3; ++i) {
        ASSERT_NEAR(1e-12, 0.0, shaper.last().wheel_rpm[i]);
    }
    TEST_ASSERT_FALSE(shaper.last().accel_limited);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_braking_speed_is_sqrt_2_a_d);
    RUN_TEST(test_tick_braking_speed_stops_exactly_on_the_distance);
    RUN_TEST(test_ramp_slope_is_320_rpm_s_at_15_ms);
    RUN_TEST(test_ramp_slope_is_320_rpm_s_at_10_ms);
    RUN_TEST(test_ramp_keeps_the_direction);
    RUN_TEST(test_deceleration_is_limited_too);
    RUN_TEST(test_normalisation_caps_the_fastest_wheel_and_keeps_the_direction);
    RUN_TEST(test_round_trip_through_forward_kinematics);
    RUN_TEST(test_table_command_is_turned_into_the_robot_frame);
    RUN_TEST(test_long_approach_reaches_1_m_s_and_stops_without_overshoot);
    RUN_TEST(test_short_approach_stops_without_overshoot);
    RUN_TEST(test_braking_caps_the_position_loop_only);
    RUN_TEST(test_rotation_is_braked_on_the_heading_error);
    RUN_TEST(test_reset_and_bad_dt);
    return UNITY_END();
}
