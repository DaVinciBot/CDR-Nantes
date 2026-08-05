/**
 * test_logique_complete.cpp
 */

#include <Arduino.h>
#include <holonomic_basis.h>
#include <config.h>

// ===== PID =====
PID x_pid    (KP_X,     KI_X,     KD_X,     -MAX_SPEED_RPM, MAX_SPEED_RPM, 5.0);
PID y_pid    (KP_Y,     KI_Y,     KD_Y,     -MAX_SPEED_RPM, MAX_SPEED_RPM, 5.0);
PID theta_pid(KP_THETA, KI_THETA, KD_THETA, -MAX_SPEED_RPM, MAX_SPEED_RPM, 2.0);

Holonomic_Basis* hb = new Holonomic_Basis(
    ROBOT_RADIUS, WHEEL_DIAMETER, MAX_SPEED_RPM,
    x_pid, y_pid, theta_pid
);

enum TestState {
    TEST_INIT,
    TEST_Y_100MM,
    TEST_DONE
};

TestState state = TEST_INIT;
uint32_t state_start_time = 0;
Point target(0, 0, 0);
int64_t enc1_start = 0, enc2_start = 0, enc3_start = 0;

IntervalTimer compute_timer;

void compute_loop() {
    hb->update_odometry();
    hb->handle(target, nullptr);
    hb->need_send_movement = true;
    hb->need_read_encoders = true;
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n╔══════════════════════════════════════════════════╗");
    Serial.println("║       TEST Y- 100mm  ENC+OPT+PID                ║");
    Serial.println("╚══════════════════════════════════════════════════╝\n");

    hb->define_wheel1(W1_SERIAL, W1_ADDR);
    hb->define_wheel2(W2_SERIAL, W2_ADDR);
    hb->define_wheel3(W3_SERIAL, W3_ADDR);
    hb->init_motors();
    hb->init_holonomic_basis(0.0, 0.0, 0.0);
    hb->init_sensors();

    hb->use_encoders     = false;
    hb->use_optical_flow = false;
    hb->use_imu          = false;
    hb->use_pid_control  = true;

    Serial.println("╔══════════════════════════════════════════════════╗");
    Serial.println("║              CONFIGURATION                       ║");
    Serial.println("╠══════════════════════════════════════════════════╣");
    Serial.println("║  Mode      : PID                                 ║");
    Serial.println("║  Odométrie : Encodeurs RS485 + Optique PAA5100   ║");
    Serial.println("║  Fréquence : 100 Hz                              ║");
    Serial.println("║                                                  ║");
    Serial.println("║  TEST : Translation Y- = 100 mm                  ║");
    Serial.println("║    → Cible  : target.y = -100 mm                ║");
    Serial.println("║    → Attendu: ligne droite en -Y, X stable       ║");
    Serial.println("╚══════════════════════════════════════════════════╝\n");

    // ╔══════════════ TEST TIRETTE (temporaire) ══════════════╗
    tirette_init();
    Serial.println("[TEST TIRETTE] Lecture GPIO 31 pendant 8s...");
    Serial.println("[TEST TIRETTE] Insere/retire le cable pour voir l'etat changer.");
    for (int i = 0; i < 40; i++) {
        Serial.printf("[TEST TIRETTE] GPIO 34 = %s\n",
            tirette_is_inserted() ? "HIGH  (cable en place)" : "LOW   (cable retire)");
        delay(200);
    }
    Serial.println("[TEST TIRETTE] Fin du test. Attente retrait tirette pour demarrage...");
    tirette_wait_for_start();
    // ╚══════════════════════════════════════════════════════╝

    hb->calibrate_imu_origin();
    Serial.println("[ZERO] Angle 0 defini sur l'orientation actuelle (X+)");

    hb->enable_motors();
    delay(500);

    // Pre-fill encoder buffers with real motor positions before the ISR starts.
    // buffered_enc initializes to 0, but MKS servos retain absolute position.
    // Without this, the first ISR call captures last_enc=0, the second sees the
    // full absolute position as a delta → phantom position jump of hundreds of mm.
    hb->read_encoders_nonblocking();
    delay(50);  // ensure buffers are written before ISR first fires

    compute_timer.begin(compute_loop, ASSERVISSEMENT_FREQUENCY);
    state_start_time = millis();
}

// ===== AFFICHAGE =====
static uint32_t last_display_ms = 0;
void display_odometry() {
    uint32_t now = millis();
    if (now - last_display_ms < 500) return;
    last_display_ms = now;

    Point pos = hb->get_current_position();
    double w1 = hb->filtered_wheel1_rpm;
    double w2 = hb->filtered_wheel2_rpm;
    double w3 = hb->filtered_wheel3_rpm;

    double opt_x, opt_y;
    uint32_t opt_valid, opt_outliers;
    hb->get_optical_data(opt_x, opt_y, opt_valid, opt_outliers);

    double dist = fabs(pos.y + 100.0);

    Serial.println("─────────────────────────────────────────────────────────────");
    Serial.printf("[POS]  X=%+7.1fmm  Y=%+7.1fmm  θ=%+6.3frad  |  Reste=%.1fmm\n",
                  pos.x, pos.y, pos.theta, dist);
    Serial.printf("[OPT]  accX=%+7.1fmm  accY=%+7.1fmm  valid=%-5u  outliers=%u\n",
                  opt_x, opt_y, opt_valid, opt_outliers);
    Serial.printf("[CMD]  W1=%+6.0fRPM  W2=%+6.0fRPM  W3=%+6.0fRPM\n",
                  w1, w2, w3);
}

void loop() {
    uint32_t now = millis();

    if (hb->need_read_encoders) {
        hb->read_encoders_nonblocking();
        hb->need_read_encoders = false;
    }

    if (hb->need_send_movement) {
        hb->send_movement_commands_nonblocking();
        hb->need_send_movement = false;
    }

    display_odometry();

    uint32_t elapsed = now - state_start_time;

    switch (state) {

        // ──────────────────────────────────────────────────────────────────
        case TEST_INIT:
            if (elapsed > 1000) {
                Serial.println("\n╔══════════════════════════════════════════════════╗");
                Serial.println("║         TEST : Translation Y- 100mm               ║");
                Serial.println("╠══════════════════════════════════════════════════╣");
                Serial.println("║  Cible : target.y = -100mm  target.theta = 0     ║");
                Serial.println("║  Attendu : ligne droite en -Y, X et θ stables    ║");
                Serial.println("╠══════════════════════════════════════════════════╣");
                
                // Capturer les encodeurs de départ
                hb->get_raw_encoders(enc1_start, enc2_start, enc3_start);
                Serial.printf("║  [DÉPART] Enc1=%lld  Enc2=%lld  Enc3=%lld          ║\n", 
                              enc1_start, enc2_start, enc3_start);
                Serial.println("╚══════════════════════════════════════════════════╝\n");

                target.x     = 0.0;
                target.y     = -600.0;
                target.theta = 0.0;

                state = TEST_Y_100MM;
                state_start_time = now;
            }
            break;

        // ──────────────────────────────────────────────────────────────────
        case TEST_Y_100MM: {
            Point pos = hb->get_current_position();
            double dist_to_target = fabs(pos.y + 100.0);  // Distance restante vers Y=-100mm

            // Arrêt quand la cible est atteinte
            if (dist_to_target <= 10.0 || elapsed > 15000) {  // 15s timeout
                Serial.println("\n[TEST TERMINÉ] Arrêt robot...\n");
                
                // Capturer les encodeurs finaux
                int64_t enc1_end, enc2_end, enc3_end;
                hb->get_raw_encoders(enc1_end, enc2_end, enc3_end);
                
                Serial.println("╔══════════════════════════════════════════════════╗");
                Serial.println("║          RÉSULTATS : COMPTAGE D'ENCODEURS        ║");
                Serial.println("╠══════════════════════════════════════════════════╣");
                Serial.printf("║  DÉPART : E1=%lld  E2=%lld  E3=%lld\n", enc1_start, enc2_start, enc3_start);
                Serial.printf("║  FIN    : E1=%lld  E2=%lld  E3=%lld\n", enc1_end, enc2_end, enc3_end);
                Serial.println("╠══════════════════════════════════════════════════╣");
                Serial.printf("║  ΔE1 = %lld counts\n", enc1_end - enc1_start);
                Serial.printf("║  ΔE2 = %lld counts\n", enc2_end - enc2_start);
                Serial.printf("║  ΔE3 = %lld counts\n", enc3_end - enc3_start);
                Serial.println("╠══════════════════════════════════════════════════╣");
                Serial.println("║  Position finale (odométrie) :");
                Serial.printf("║    X = %.1f mm (attendu :    0 mm)\n", pos.x);
                Serial.printf("║    Y = %.1f mm (attendu : -100 mm)\n", pos.y);
                Serial.printf("║    Θ = %.3f rad (attendu : 0 rad)\n", pos.theta);
                Serial.println("╚══════════════════════════════════════════════════╝\n");
                
                target = {0, 0, 0};
                delay(3000);  // Pause pour qu'il s'immobilise
                
                hb->emergency_stop();
                hb->disable_motors();
                
                state = TEST_DONE;
            }
            break;
        }

        // ──────────────────────────────────────────────────────────────────
        case TEST_DONE:
            break;
    }
}