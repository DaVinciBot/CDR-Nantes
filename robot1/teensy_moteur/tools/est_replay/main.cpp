/*
 * est_replay: runs a pose estimator of the board on the host, fed with the
 * inputs logged by LOG_TICK, so that a run can be replayed off the robot with
 * the very code that ran on it (lib/pose_estimator, lib/kalman,
 * lib/kinematics).
 *
 * Built by the "replay" environment of platformio.ini:
 *
 *     pio run -e replay        ->  .pio/build/replay/program(.exe)
 *
 * Driven by robot1/replay/estimators (Python) through stdin / stdout, one
 * command per line, tokens separated by spaces. Floats may be written in
 * decimal or in C99 hexadecimal ("0x1.8p+0", Python float.hex()): hexadecimal
 * carries a float without any rounding.
 *
 *   P <parameters>
 *       A new estimator with these parameters. Must come first.
 *       complementary (11 values):
 *         <robot_radius_mm> <counts_to_mm> <encoder_noise_counts>
 *         <imu_max_age_us> <optical_outlier_mm_s> <optical_noise_mm_s>
 *         <pure_rotation_min_rad> <pure_rotation_min_counts>
 *         <pure_rotation_tolerance> <alpha_encoders> <use_encoders>
 *       ekf6 (23 values): geometry, then kalman::Ekf6Params in declaration
 *       order, then pose::EkfInitialSigmas:
 *         <robot_radius_mm> <counts_to_mm>
 *         <accel_noise> <alpha_noise> <wheel_sigma_counts> <wheel_slip_ratio>
 *         <optical_sigma_mm_s> <heading_sigma_rad> <optical_lever_x_mm>
 *         <optical_lever_y_mm> <max_dt_s> <max_sigma_theta_rad>
 *         <max_sigma_v_mm_s> <max_sigma_omega_rad_s>
 *         <optical_max_consecutive_rejections> <optical_ratio_min>
 *         <optical_ratio_max> <optical_ratio_min_speed_mm_s>
 *         <optical_ratio_window>
 *         <sigma_xy_mm> <sigma_theta_rad> <sigma_v_mm_s> <sigma_omega_rad_s>
 *   R <x_mm> <y_mm> <theta_rad>
 *       reset(), as SET_ODOMETRIE or an estimator switch does on the board.
 *   S <tick> <d1> <d2> <d3> <dt_us> <encoder_age_us> <first_frame>
 *     <encoders_fresh> <imu_valid> <heading_rad> <imu_timestamp_us>
 *     <imu_age_us> <optical_timestamp_us> <optical_step_valid> <optical_dx_mm>
 *     <optical_dy_mm> <optical_step_dt_us> <optical_velocity_valid>
 *     <optical_vx_mm_s> <optical_vy_mm_s>
 *       One step(). Answers one line:
 *       <tick> <x> <y> <theta> <vx> <vy> <omega> <health_flags> <step_flags>
 *       floats with 9 significant digits, which gives a float back exactly.
 *       ekf6 appends, on the same line:
 *         <Pxx> <Pyy> <Ptt> <Pxy> <Pxt> <Pyt>      position / heading covariance
 *         <covariance_healthy> <optical_ok> <optical_consecutive_rejections>
 *         <optical_ratio> <optical_ratio_windows> <optical_ratio_windows_out>
 *         <n> then n times <source> <innovation> <variance> <nis> <accepted>
 *       the n scalar updates of the tick (kalman::UpdateSource order:
 *       0-2 wheels, 3-4 optical x / y, 5 heading).
 *   K   keep a copy of the estimator state;
 *   B   back to that copy. The replay uses the pair to try both places of a
 *       reset whose tick is ambiguous by one.
 *
 * Only S answers, and every answer is flushed: the caller may run in lock
 * step. A malformed line ends the program with "E <reason>" on stdout and
 * exit code 2.
 *
 * Usage: est_replay [--estimator complementary|ekf6]
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <complementary_estimator.h>
#include <ekf_estimator.h>

namespace {

constexpr int MAX_LINE = 1024;
constexpr int MAX_TOKENS = 32;

[[noreturn]] void fail(const char *reason, long line_number) {
    printf("E line %ld: %s\n", line_number, reason);
    fflush(stdout);
    exit(2);
}

/** Splits a line in place on spaces and tabs; returns the token count. */
int tokenize(char *line, char *tokens[MAX_TOKENS]) {
    int count = 0;
    char *p = line;
    while (*p != '\0') {
        while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') *p++ = '\0';
        if (*p == '\0') break;
        if (count == MAX_TOKENS) return -1;
        tokens[count++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') ++p;
    }
    return count;
}

double to_double(const char *token, long line_number) {
    char *end = nullptr;
    const double value = strtod(token, &end);
    if (end == token || *end != '\0') fail("bad number", line_number);
    return value;
}

long long to_integer(const char *token, long line_number) {
    char *end = nullptr;
    const long long value = strtoll(token, &end, 10);
    if (end == token || *end != '\0') fail("bad integer", line_number);
    return value;
}

float to_float(const char *token, long line_number) {
    return static_cast<float>(to_double(token, line_number));
}

uint32_t to_u32(const char *token, long line_number) {
    return static_cast<uint32_t>(to_integer(token, line_number));
}

bool to_bool(const char *token, long line_number) {
    return to_integer(token, line_number) != 0;
}

pose::ComplementaryParams parse_params(char *tokens[], long n) {
    pose::ComplementaryParams p;
    p.robot_radius_mm = to_double(tokens[1], n);
    p.counts_to_mm = to_double(tokens[2], n);
    p.encoder_noise_counts = to_double(tokens[3], n);
    p.imu_max_age_us = to_u32(tokens[4], n);
    p.optical_outlier_mm_s = to_double(tokens[5], n);
    p.optical_noise_mm_s = to_double(tokens[6], n);
    p.pure_rotation_min_rad = to_double(tokens[7], n);
    p.pure_rotation_min_counts = to_double(tokens[8], n);
    p.pure_rotation_tolerance = to_double(tokens[9], n);
    p.alpha_encoders = to_double(tokens[10], n);
    p.use_encoders = to_bool(tokens[11], n);
    return p;
}

/** @brief What the ekf6 P line builds. */
struct EkfSetup {
    kalman::Geometry geometry;
    kalman::Ekf6Params params;
    pose::EkfInitialSigmas sigmas;
};

EkfSetup parse_ekf_params(char *tokens[], long n) {
    EkfSetup s;
    s.geometry.robot_radius_mm = to_float(tokens[1], n);
    s.geometry.counts_to_mm = to_float(tokens[2], n);
    kalman::Ekf6Params &p = s.params;
    p.accel_noise = to_float(tokens[3], n);
    p.alpha_noise = to_float(tokens[4], n);
    p.wheel_sigma_counts = to_float(tokens[5], n);
    p.wheel_slip_ratio = to_float(tokens[6], n);
    p.optical_sigma_mm_s = to_float(tokens[7], n);
    p.heading_sigma_rad = to_float(tokens[8], n);
    p.optical_lever_x_mm = to_float(tokens[9], n);
    p.optical_lever_y_mm = to_float(tokens[10], n);
    p.max_dt_s = to_float(tokens[11], n);
    p.max_sigma_theta_rad = to_float(tokens[12], n);
    p.max_sigma_v_mm_s = to_float(tokens[13], n);
    p.max_sigma_omega_rad_s = to_float(tokens[14], n);
    p.optical_max_consecutive_rejections = to_u32(tokens[15], n);
    p.optical_ratio_min = to_float(tokens[16], n);
    p.optical_ratio_max = to_float(tokens[17], n);
    p.optical_ratio_min_speed_mm_s = to_float(tokens[18], n);
    p.optical_ratio_window = to_u32(tokens[19], n);
    s.sigmas.xy_mm = to_float(tokens[20], n);
    s.sigmas.theta_rad = to_float(tokens[21], n);
    s.sigmas.v_mm_s = to_float(tokens[22], n);
    s.sigmas.omega_rad_s = to_float(tokens[23], n);
    return s;
}

pose::EstimatorInput parse_input(char *tokens[], long n) {
    pose::EstimatorInput in = pose::empty_input();
    in.d1 = static_cast<int32_t>(to_integer(tokens[2], n));
    in.d2 = static_cast<int32_t>(to_integer(tokens[3], n));
    in.d3 = static_cast<int32_t>(to_integer(tokens[4], n));
    in.dt_us = to_u32(tokens[5], n);
    in.encoder_age_us = to_u32(tokens[6], n);
    in.first_frame = to_bool(tokens[7], n);
    in.encoders_fresh = to_bool(tokens[8], n);
    in.imu_valid = to_bool(tokens[9], n);
    in.heading_rad = to_float(tokens[10], n);
    in.imu_timestamp_us = to_u32(tokens[11], n);
    in.imu_age_us = to_u32(tokens[12], n);
    in.optical_timestamp_us = to_u32(tokens[13], n);
    in.optical_step_valid = to_bool(tokens[14], n);
    in.optical_dx_mm = to_float(tokens[15], n);
    in.optical_dy_mm = to_float(tokens[16], n);
    in.optical_step_dt_us = to_u32(tokens[17], n);
    in.optical_velocity_valid = to_bool(tokens[18], n);
    in.optical_vx_mm_s = to_float(tokens[19], n);
    in.optical_vy_mm_s = to_float(tokens[20], n);
    return in;
}

// ===== One estimator type behind the protocol =====

struct Complementary {
    using Estimator = pose::ComplementaryEstimator;
    static constexpr int P_VALUES = 11;
    static Estimator *make(char *tokens[], long n) {
        return new Estimator(parse_params(tokens, n));
    }
    static void print_extra(const Estimator &) {}
};

struct Ekf {
    using Estimator = pose::EkfEstimator;
    static constexpr int P_VALUES = 23;
    static Estimator *make(char *tokens[], long n) {
        const EkfSetup s = parse_ekf_params(tokens, n);
        return new Estimator(s.geometry, s.params, s.sigmas);
    }
    static void print_extra(const Estimator &estimator) {
        const kalman::Ekf6 &f = estimator.filter();
        const kalman::Mat<kalman::Ekf6::N> &P = f.covariance();
        constexpr size_t X = kalman::Ekf6::X;
        constexpr size_t Y = kalman::Ekf6::Y;
        constexpr size_t T = kalman::Ekf6::THETA;
        printf(" %.9g %.9g %.9g %.9g %.9g %.9g", static_cast<double>(P(X, X)),
               static_cast<double>(P(Y, Y)), static_cast<double>(P(T, T)),
               static_cast<double>(P(X, Y)), static_cast<double>(P(X, T)),
               static_cast<double>(P(Y, T)));
        printf(" %d %d %lu %.9g %lu %lu", f.covariance_healthy() ? 1 : 0,
               f.optical_ok() ? 1 : 0,
               static_cast<unsigned long>(f.optical_consecutive_rejections()),
               static_cast<double>(f.optical_ratio()),
               static_cast<unsigned long>(f.optical_ratio_windows()),
               static_cast<unsigned long>(f.optical_ratio_windows_out_of_band()));
        printf(" %u", static_cast<unsigned>(f.tick_update_count()));
        for (size_t i = 0; i < f.tick_update_count(); ++i) {
            const kalman::TickUpdate &u = f.tick_update(i);
            printf(" %u %.9g %.9g %.9g %d", static_cast<unsigned>(u.source),
                   static_cast<double>(u.result.innovation),
                   static_cast<double>(u.result.variance),
                   static_cast<double>(u.result.nis), u.result.accepted ? 1 : 0);
        }
    }
};

template <class Kind>
int run() {
    using Estimator = typename Kind::Estimator;
    Estimator *estimator = nullptr;
    Estimator *kept = nullptr;
    char line[MAX_LINE];
    char *tokens[MAX_TOKENS];
    long line_number = 0;

    while (fgets(line, sizeof(line), stdin) != nullptr) {
        ++line_number;
        if (strchr(line, '\n') == nullptr && !feof(stdin)) fail("line too long", line_number);
        const int count = tokenize(line, tokens);
        if (count < 0) fail("too many tokens", line_number);
        if (count == 0) continue;
        const char command = tokens[0][0];
        if (tokens[0][1] != '\0') fail("unknown command", line_number);

        if (command == 'P') {
            if (count != Kind::P_VALUES + 1) fail("wrong number of P values", line_number);
            delete estimator;
            delete kept;
            kept = nullptr;
            estimator = Kind::make(tokens, line_number);
            continue;
        }
        if (estimator == nullptr) fail("no P line yet", line_number);

        if (command == 'R') {
            if (count != 4) fail("R takes 3 values", line_number);
            estimator->reset(to_float(tokens[1], line_number), to_float(tokens[2], line_number),
                             to_float(tokens[3], line_number));
        } else if (command == 'S') {
            if (count != 21) fail("S takes 20 values", line_number);
            estimator->step(parse_input(tokens, line_number));
            const pose::PoseEstimate p = estimator->pose();
            printf("%s %.9g %.9g %.9g %.9g %.9g %.9g %u %u", tokens[1],
                   static_cast<double>(p.x_mm), static_cast<double>(p.y_mm),
                   static_cast<double>(p.theta_rad), static_cast<double>(p.vx_mm_s),
                   static_cast<double>(p.vy_mm_s), static_cast<double>(p.omega_rad_s),
                   static_cast<unsigned>(estimator->health_flags()),
                   static_cast<unsigned>(estimator->step_flags()));
            Kind::print_extra(*estimator);
            printf("\n");
            fflush(stdout);
        } else if (command == 'K') {
            if (kept == nullptr) {
                kept = new Estimator(*estimator);
            } else {
                *kept = *estimator;
            }
        } else if (command == 'B') {
            if (kept == nullptr) fail("B without K", line_number);
            *estimator = *kept;
        } else {
            fail("unknown command", line_number);
        }
    }
    delete estimator;
    delete kept;
    return 0;
}

}  // namespace

int main(int argc, char **argv) {
    bool ekf = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--estimator") == 0 && i + 1 < argc) {
            if (strcmp(argv[i + 1], "ekf6") == 0) {
                ekf = true;
            } else if (strcmp(argv[i + 1], "complementary") != 0) {
                printf("E unknown estimator: %s\n", argv[i + 1]);
                return 2;
            }
            ++i;
        } else {
            printf("E usage: est_replay [--estimator complementary|ekf6]\n");
            return 2;
        }
    }
    return ekf ? run<Ekf>() : run<Complementary>();
}
