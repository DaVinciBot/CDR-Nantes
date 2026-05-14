#include <Arduino.h>
#include <SPI.h>
#include <Bitcraze_PMW3901.h>

// ── PAA5100JE ─────────────────────────────────────────────────────────────────
Bitcraze_PMW3901 paa5100(10);   // CS=10, SPI0
bool paa_ok = false;

// ── SPI Teensy 4.1 — FORCÉ à 2MHz max (PAA5100JE ne supporte pas plus) ────────
#define SPI_FREQ 2000000   // 2 MHz

// ── Calibration ───────────────────────────────────────────────────────────────
const float CALIB_DISTANCES_MM[] = { 500.0f, 500.0f, 500.0f, 500.0f };
const uint8_t NB_CALIB_STEPS = 4;
float SENSOR_HEIGHT_MM = 19.0f;
float scale_x_results[NB_CALIB_STEPS];
float scale_y_results[NB_CALIB_STEPS];
uint8_t valid_results = 0;
long long acc_counts_x = 0;
long long acc_counts_y = 0;

enum CalibState { IDLE, WAITING_START, RECORDING, WAITING_RESULT };
CalibState state = IDLE;
uint8_t current_step = 0;
char current_axis = 'X';

// ── Wrapper SPI sécurisé pour Teensy 4.1 ─────────────────────────────────────
void spi_write_reg(uint8_t reg, uint8_t val) {
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));
    digitalWriteFast(10, LOW);
    delayMicroseconds(1);
    SPI.transfer(reg | 0x80);
    SPI.transfer(val);
    delayMicroseconds(1);
    digitalWriteFast(10, HIGH);
    SPI.endTransaction();
    delayMicroseconds(20);
}

uint8_t spi_read_reg(uint8_t reg) {
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));
    digitalWriteFast(10, LOW);
    delayMicroseconds(1);
    SPI.transfer(reg & 0x7F);
    delayMicroseconds(2);    // tSRAD = 2µs min
    uint8_t val = SPI.transfer(0x00);
    delayMicroseconds(1);
    digitalWriteFast(10, HIGH);
    SPI.endTransaction();
    delayMicroseconds(20);
    return val;
}

// ── Utilitaires ───────────────────────────────────────────────────────────────
void printSeparator() {
    Serial.println(F("════════════════════════════════════════════════════"));
}

void printMenu() {
    printSeparator();
    Serial.println(F("  CALIBRATION PAA5100JE — Teensy 4.1"));
    printSeparator();
    Serial.println(F("  [1] Calibration axe X"));
    Serial.println(F("  [2] Calibration axe Y"));
    Serial.println(F("  [3] Calibration complete X puis Y"));
    Serial.println(F("  [4] Afficher facteurs calcules"));
    Serial.println(F("  [5] Test libre avec facteur actuel"));
    Serial.println(F("  [6] Validation — comparer capteur vs realite"));
    Serial.println(F("  [d] Diagnostic SPI complet"));
    Serial.println(F("  [h] Modifier la hauteur capteur"));
    Serial.println(F("  [r] Reset / retour menu"));
    printSeparator();
    Serial.printf("  SPI : %d Hz  |  Hauteur : %.1f mm\n", SPI_FREQ, SENSOR_HEIGHT_MM);
    printSeparator();
}

void printCurrentScale() {
    Serial.println(F("\n── Facteur SCALE actuel ──────────────────────────────"));
    if (valid_results == 0) {
        Serial.println(F("  Aucune calibration effectuee."));
        Serial.println(F("  Valeur par defaut : 0.0423 mm/count"));
    } else {
        float sum_x = 0, sum_y = 0;
        uint8_t cnt_x = 0, cnt_y = 0;
        for (uint8_t i = 0; i < NB_CALIB_STEPS; i++) {
            if (scale_x_results[i] > 0) { sum_x += scale_x_results[i]; cnt_x++; }
            if (scale_y_results[i] > 0) { sum_y += scale_y_results[i]; cnt_y++; }
        }
        if (cnt_x > 0) Serial.printf("  SCALE_X moyen : %.6f mm/count (%u mesures)\n", sum_x/cnt_x, cnt_x);
        if (cnt_y > 0) Serial.printf("  SCALE_Y moyen : %.6f mm/count (%u mesures)\n", sum_y/cnt_y, cnt_y);
        Serial.println(F("\n  ── Copiez dans votre code principal ─────────────"));
        if (cnt_x > 0 && cnt_y > 0 && fabsf((sum_x/cnt_x)-(sum_y/cnt_y)) < 0.005f) {
            Serial.printf("  const float SCALE = %.6ff;\n", (sum_x/cnt_x + sum_y/cnt_y)/2.0f);
        } else {
            if (cnt_x > 0) Serial.printf("  const float SCALE_X = %.6ff;\n", sum_x/cnt_x);
            if (cnt_y > 0) Serial.printf("  const float SCALE_Y = %.6ff;\n", sum_y/cnt_y);
        }
    }
    Serial.println();
}

// ── Récupère le meilleur SCALE X disponible ───────────────────────────────────
float getBestScaleX() {
    float sum_x = 0;
    uint8_t cnt_x = 0;
    for (uint8_t i = 0; i < NB_CALIB_STEPS; i++) {
        if (scale_x_results[i] > 0) { sum_x += scale_x_results[i]; cnt_x++; }
    }
    if (cnt_x > 0) return sum_x / cnt_x;
    return 0.006034f;   // valeur mesurée lors de la session précédente
}

// ── Diagnostic SPI ────────────────────────────────────────────────────────────
void runSpiDiagnostic() {
    printSeparator();
    Serial.println(F("  DIAGNOSTIC SPI — Teensy 4.1"));
    printSeparator();

    uint8_t pid = spi_read_reg(0x00);
    Serial.printf("  Product ID      : 0x%02X (attendu 0x49) %s\n",
                  pid, pid == 0x49 ? "OK" : "MAUVAIS — prob SPI ou cablage");

    uint8_t rev = spi_read_reg(0x01);
    Serial.printf("  Revision ID     : 0x%02X\n", rev);

    uint8_t squal = spi_read_reg(0x07);
    Serial.printf("  SQUAL (qualite) : %u/255  ", squal);
    if      (squal > 100) Serial.println(F("*** Excellent"));
    else if (squal > 50)  Serial.println(F("**  Correct"));
    else if (squal > 20)  Serial.println(F("*   Faible — surface inadaptee"));
    else                  Serial.println(F("    Tres faible — verif hauteur/surface"));

    uint8_t shutH = spi_read_reg(0x0C);
    uint8_t shutL = spi_read_reg(0x0B);
    uint16_t shutter = (shutH << 8) | shutL;
    Serial.printf("  Shutter         : %u  ", shutter);
    if (shutter > 1000) Serial.println(F("(exposition longue — surface sombre ou trop loin)"));
    else                Serial.println(F("(normal)"));

    uint8_t raw = spi_read_reg(0x08);
    Serial.printf("  RawData_Sum     : %u/255\n", raw);

    uint8_t maxraw = spi_read_reg(0x09);
    Serial.printf("  Max_RawData     : %u/255  ", maxraw);
    if (maxraw == 255) Serial.println(F("SATURATION — trop de lumiere ou surface trop reflechissante"));
    else               Serial.println(F("(normal)"));

    printSeparator();
    Serial.printf("  Frequence SPI   : %d Hz\n", SPI_FREQ);
    Serial.println(F("  Pins Teensy 4.1 : CS=10  MOSI=11  MISO=12  SCK=13"));
    printSeparator();
}

// ── Polling capteur via Motion Burst ─────────────────────────────────────────
void pollSensor() {
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));
    digitalWriteFast(10, LOW);
    delayMicroseconds(1);
    SPI.transfer(0x16 & 0x7F);   // Motion_Burst
    delayMicroseconds(2);        // tSRAD obligatoire
    uint8_t motion = SPI.transfer(0x00);
    uint8_t dxL    = SPI.transfer(0x00);
    uint8_t dxH    = SPI.transfer(0x00);
    uint8_t dyL    = SPI.transfer(0x00);
    uint8_t dyH    = SPI.transfer(0x00);
    SPI.transfer(0x00);  // squal (ignoré ici)
    delayMicroseconds(1);
    digitalWriteFast(10, HIGH);
    SPI.endTransaction();
    delayMicroseconds(20);

    // Bit 7 du registre Motion = mouvement détecté
    if (motion & 0x80) {
        int16_t dx = (int16_t)((dxH << 8) | dxL);
        int16_t dy = (int16_t)((dyH << 8) | dyL);
        acc_counts_x += dx;
        acc_counts_y += dy;
    }
}

// ── Polling simple (hors calibration) ────────────────────────────────────────
void pollSimple(int16_t &dx, int16_t &dy) {
    SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE3));
    digitalWriteFast(10, LOW);
    delayMicroseconds(1);
    SPI.transfer(0x16 & 0x7F);
    delayMicroseconds(2);
    uint8_t motion = SPI.transfer(0x00);
    uint8_t dxL    = SPI.transfer(0x00);
    uint8_t dxH    = SPI.transfer(0x00);
    uint8_t dyL    = SPI.transfer(0x00);
    uint8_t dyH    = SPI.transfer(0x00);
    SPI.transfer(0x00);
    delayMicroseconds(1);
    digitalWriteFast(10, HIGH);
    SPI.endTransaction();
    delayMicroseconds(20);

    if (motion & 0x80) {
        dx = (int16_t)((dxH << 8) | dxL);
        dy = (int16_t)((dyH << 8) | dyL);
    } else {
        dx = 0;
        dy = 0;
    }
}

// ── Mode validation [6] ───────────────────────────────────────────────────────
void modeValidation() {
    if (!paa_ok) { Serial.println(F("[ERREUR] Capteur non detecte !")); return; }

    float scale = getBestScaleX();

    printSeparator();
    Serial.println(F("  MODE VALIDATION — comparer capteur vs realite"));
    printSeparator();
    Serial.printf("  SCALE utilise : %.6f mm/count\n", scale);
    Serial.println(F("  [ESPACE] = demarre / arrete une mesure"));
    Serial.println(F("  [r]      = quitter"));
    printSeparator();

    long long cx       = 0;
    long long cx_start = 0;
    bool      recording = false;
    uint8_t   run       = 0;
    uint32_t  t_last    = millis();
    uint32_t  t_print   = millis();

    while (true) {

        // ── Lecture Serial ────────────────────────────────────────────────────
        if (Serial.available()) {
            char q = Serial.read();
            while (Serial.available()) Serial.read();   // vider buffer

            if (q == '\n' || q == '\r') {
                // ignorer résidus terminal
            } else if (q == 'r' || q == 'R') {
                break;

            } else if (q == ' ') {
                if (!recording) {
                    // ── Démarrage d'un run ────────────────────────────────────
                    cx_start  = cx;
                    recording = true;
                    run++;
                    Serial.printf("\n  Demarre Run %u — deplacez le robot sur axe X...\n", run);
                    Serial.println(F("  [ESPACE] pour arreter et afficher le resultat\n"));

                } else {
                    // ── Fin d'un run — affichage résultat ─────────────────────
                    recording = false;
                    long long delta   = cx - cx_start;
                    float     mesured = (float)abs(delta) * scale;

                    Serial.println();
                    printSeparator();
                    Serial.printf("  Run %u — RESULTAT\n", run);
                    printSeparator();
                    Serial.printf("  Counts mesures   : %lld\n",  abs(delta));
                    Serial.printf("  Distance capteur : %.2f mm\n", mesured);
                    Serial.println(F("  Distance reelle  : ??? mm  <- mesurez avec un metre ruban"));
                    Serial.println();

                    // Calcul d'erreur si l'utilisateur entre la distance réelle
                    Serial.println(F("  Entrez la distance reelle en mm + ENTREE (ou ESPACE pour nouveau run) :"));

                    String input  = "";
                    bool   got_input = false;
                    uint32_t t_inp = millis();
                    while (millis() - t_inp < 15000) {
                        if (Serial.available()) {
                            char ch = Serial.read();
                            if (ch == ' ') {
                                // L'utilisateur veut démarrer un nouveau run directement
                                cx_start  = cx;
                                recording = true;
                                run++;
                                Serial.printf("\n  Demarre Run %u — deplacez le robot...\n", run);
                                got_input = true;
                                break;
                            } else if (ch == '\n' || ch == '\r') {
                                if (input.length() > 0) { got_input = true; break; }
                            } else if (ch == 'r' || ch == 'R') {
                                Serial.println();
                                printMenu();
                                return;
                            } else {
                                input += ch;
                                Serial.print(ch);
                            }
                        }
                    }

                    if (got_input && input.length() > 0) {
                        float real_mm = input.toFloat();
                        if (real_mm > 0) {
                            float erreur    = mesured - real_mm;
                            float erreur_pc = (erreur / real_mm) * 100.0f;
                            float scale_reel = real_mm / (float)abs(delta);
                            Serial.println();
                            printSeparator();
                            Serial.printf("  Distance reelle  : %.1f mm\n",  real_mm);
                            Serial.printf("  Distance capteur : %.2f mm\n",  mesured);
                            Serial.printf("  Erreur           : %+.2f mm  (%+.1f%%)\n", erreur, erreur_pc);
                            Serial.printf("  SCALE reel       : %.6f mm/count\n", scale_reel);
                            if (fabsf(erreur_pc) < 2.0f)
                                Serial.println(F("  Qualite          : *** Excellent (<2%)"));
                            else if (fabsf(erreur_pc) < 5.0f)
                                Serial.println(F("  Qualite          : **  Bon (<5%)"));
                            else if (fabsf(erreur_pc) < 10.0f)
                                Serial.println(F("  Qualite          : *   Acceptable (<10%)"));
                            else
                                Serial.println(F("  Qualite          :     Mauvais (>10%) — recalibrer"));
                            printSeparator();
                            Serial.println(F("  [ESPACE] nouveau run  |  [r] quitter"));
                        }
                    }
                }
            }
        }

        // ── Polling capteur 100Hz ─────────────────────────────────────────────
        if (millis() - t_last >= 10) {
            t_last = millis();
            int16_t dx = 0, dy = 0;
            pollSimple(dx, dy);
            cx += dx;
        }

        // ── Affichage live pendant recording ─────────────────────────────────
        if (recording && millis() - t_print >= 100) {
            t_print = millis();
            long long delta   = cx - cx_start;
            float     mesured = (float)abs(delta) * scale;
            uint8_t   sq      = spi_read_reg(0x07);
            Serial.printf("  Distance capteur : %+8.2f mm  (counts: %+lld)  SQUAL: %3u\r",
                          mesured, delta, sq);
        }
    }

    Serial.println();
    printMenu();
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    pinMode(10, OUTPUT);
    digitalWriteFast(10, HIGH);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(115200);
    delay(1500);

    Serial.println(F("\n\n"));
    printSeparator();
    Serial.println(F("  OUTIL DE CALIBRATION PAA5100JE"));
    Serial.println(F("  Teensy 4.1 — SPI force 2MHz"));
    printSeparator();

    SPI.begin();

    Serial.print(F("[PAA5100] Initialisation... "));
    uint32_t t0 = millis();
    while (millis() - t0 < 500) {
        if (paa5100.begin()) { paa_ok = true; break; }
        delay(10);
    }
    if (paa_ok) {
        Serial.println(F("OK"));
    } else {
        Serial.println(F("ECHEC"));
        Serial.println(F("  CS=10  MOSI=11  MISO=12  SCK=13  VCC=3.3V"));
    }

    memset(scale_x_results, 0, sizeof(scale_x_results));
    memset(scale_y_results, 0, sizeof(scale_y_results));

    digitalWrite(LED_BUILTIN, LOW);

    // Diagnostic rapide au démarrage
    Serial.println(F("\n── Diagnostic demarrage ─────────────────────────────"));
    uint8_t pid = spi_read_reg(0x00);
    Serial.printf("  Product ID : 0x%02X (attendu 0x49) %s\n",
                  pid, pid == 0x49 ? "OK" : "MAUVAIS");
    uint8_t squal = spi_read_reg(0x07);
    Serial.printf("  SQUAL      : %u/255\n", squal);

    uint16_t non_null = 0;
    for (int i = 0; i < 100; i++) {
        int16_t dx, dy;
        paa5100.readMotionCount(&dx, &dy);
        if (dx != 0 || dy != 0) non_null++;
        delay(10);
    }
    Serial.printf("  Repos      : %u/100 lectures non-nulles", non_null);
    if      (non_null < 5)  Serial.println(F(" — Stable"));
    else if (non_null < 20) Serial.println(F(" — Legerement bruyant"));
    else                    Serial.println(F(" — Tres bruyant — prob alim ou surface"));
    Serial.println();

    printMenu();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {

    // ── Polling calibration 100Hz ─────────────────────────────────────────────
    static uint32_t last_poll = 0;
    if (millis() - last_poll >= 10) {
        last_poll = millis();
        if (paa_ok && state == RECORDING) pollSensor();
    }

    // ── Affichage counts pendant calibration (500ms) ──────────────────────────
    static uint32_t last_display = 0;
    if (state == RECORDING && millis() - last_display >= 500) {
        last_display = millis();
        long long counts = (current_axis == 'X') ? acc_counts_x : acc_counts_y;
        Serial.printf("  Counts accumules axe %c : %lld\n", current_axis, counts);
    }

    // ── Lecture commandes Serial ──────────────────────────────────────────────
    if (!Serial.available()) return;

    char c = Serial.read();
    while (Serial.available()) Serial.read();   // vider buffer

    // Reset global
    if (c == 'r' || c == 'R') {
        state         = IDLE;
        current_step  = 0;
        acc_counts_x  = 0;
        acc_counts_y  = 0;
        Serial.println(F("\n[RESET]"));
        printMenu();
        return;
    }

    switch (state) {

        // ══════════════════════════════════════════════════════════════════════
        case IDLE:

            if (c == '1' || c == '2' || c == '3') {
                if (!paa_ok) { Serial.println(F("[ERREUR] Capteur non detecte !")); return; }
                current_axis  = (c == '2') ? 'Y' : 'X';
                current_step  = 0;
                valid_results = 0;
                memset(scale_x_results, 0, sizeof(scale_x_results));
                memset(scale_y_results, 0, sizeof(scale_y_results));
                printSeparator();
                Serial.printf("  CALIBRATION AXE %c\n", current_axis);
                printSeparator();
                Serial.println(F("  1. Capteur face au sol a la hauteur configuree"));
                Serial.println(F("  2. Marquez le point de depart"));
                Serial.println(F("  3. Deplacez LENTEMENT (~50 mm/s) EN LIGNE DROITE"));
                Serial.println(F("  4. Appuyez sur une touche quand le deplacement est termine"));
                printSeparator();
                state = WAITING_START;
                Serial.printf("\n  Etape 1/%u — Distance cible : %.0f mm\n",
                              NB_CALIB_STEPS, CALIB_DISTANCES_MM[0]);
                Serial.println(F("  -> Point de depart pret ? Appuyez sur une touche..."));

            } else if (c == '4') {
                printCurrentScale();

            } else if (c == '5') {
                // ── Mode test libre ───────────────────────────────────────────
                if (!paa_ok) { Serial.println(F("[ERREUR] Capteur non detecte !")); return; }
                float scale = getBestScaleX();
                printSeparator();
                Serial.println(F("  MODE TEST LIBRE — deplacez le capteur (r pour quitter)"));
                Serial.printf("  SCALE : %.6f mm/count\n", scale);
                Serial.println(F("  SQUAL affiche en temps reel (>50 = surface OK)"));
                printSeparator();
                float    px = 0, py = 0;
                uint32_t t_last  = millis();
                uint32_t t_print = millis();
                while (true) {
                    if (Serial.available()) {
                        char q = Serial.read();
                        if (q == 'r' || q == 'R') break;
                    }
                    if (millis() - t_last >= 10) {
                        t_last = millis();
                        int16_t dx = 0, dy = 0;
                        pollSimple(dx, dy);
                        px += dx * scale;
                        py += dy * scale;
                    }
                    if (millis() - t_print >= 100) {
                        t_print = millis();
                        uint8_t sq = spi_read_reg(0x07);
                        Serial.printf("  PosX: %+8.2f mm  PosY: %+8.2f mm  SQUAL: %3u\r", px, py, sq);
                    }
                }
                Serial.println();
                printMenu();

            } else if (c == '6') {
                modeValidation();

            } else if (c == 'd' || c == 'D') {
                runSpiDiagnostic();

            } else if (c == 'h' || c == 'H') {
                Serial.println(F("\n  Nouvelle hauteur en mm + ENTREE :"));
                String   input  = "";
                uint32_t t_wait = millis();
                while (millis() - t_wait < 10000) {
                    if (Serial.available()) {
                        char ch = Serial.read();
                        if (ch == '\n' || ch == '\r') { if (input.length() > 0) break; }
                        else { input += ch; Serial.print(ch); }
                    }
                }
                float h = input.toFloat();
                if (h >= 10.0f && h <= 100.0f) {
                    SENSOR_HEIGHT_MM = h;
                    Serial.printf("\n  Hauteur : %.1f mm\n", SENSOR_HEIGHT_MM);
                } else {
                    Serial.println(F("\n  Invalide (10-100 mm)"));
                }
                printMenu();
            }
            break;

        // ══════════════════════════════════════════════════════════════════════
        case WAITING_START:
            if (c == '\n' || c == '\r') break;   // ignorer résidus terminal
            acc_counts_x = 0;
            acc_counts_y = 0;
            state = RECORDING;
            Serial.printf("\n  ENREGISTREMENT — deplacez de %.0f mm axe %c...\n",
                          CALIB_DISTANCES_MM[current_step], current_axis);
            Serial.println(F("  -> Appuyez sur une touche quand termine"));
            break;

        // ══════════════════════════════════════════════════════════════════════
        case RECORDING: {
            if (c == '\n' || c == '\r') break;   // ignorer résidus terminal

            long long counts = (current_axis == 'X') ? acc_counts_x : acc_counts_y;
            Serial.printf("\n  STOP — Counts : %lld\n", counts);

            if (abs(counts) < 10) {
                Serial.println(F("  [!] Trop peu de counts — bougez plus lentement"));
                Serial.println(F("  -> Appuyez pour reessayer ce step"));
                state = WAITING_START;
                break;
            }

            float distance   = CALIB_DISTANCES_MM[current_step];
            float scale_calc = distance / (float)abs(counts);
            Serial.printf("  Distance  : %.1f mm\n",      distance);
            Serial.printf("  Counts    : %lld\n",         abs(counts));
            Serial.printf("  SCALE     : %.6f mm/count\n", scale_calc);

            if (scale_calc < 0.001f || scale_calc > 0.200f)
                Serial.println(F("  [!] Hors plage normale — verif hauteur/surface"));

            if (current_axis == 'X') scale_x_results[current_step] = scale_calc;
            else                     scale_y_results[current_step] = scale_calc;
            valid_results++;
            current_step++;

            if (current_step >= NB_CALIB_STEPS) {
                // ── Calibration terminée ──────────────────────────────────────
                printSeparator();
                Serial.printf("  CALIBRATION AXE %c TERMINEE\n", current_axis);
                printSeparator();
                float sum = 0;
                for (uint8_t i = 0; i < NB_CALIB_STEPS; i++) {
                    float v = (current_axis == 'X') ? scale_x_results[i] : scale_y_results[i];
                    Serial.printf("  Step %u (%.0f mm) : %.6f\n",
                                  i+1, CALIB_DISTANCES_MM[i], v);
                    sum += v;
                }
                float avg = sum / NB_CALIB_STEPS;
                Serial.printf("\n  MOYENNE : %.6f mm/count\n", avg);

                float variance = 0;
                for (uint8_t i = 0; i < NB_CALIB_STEPS; i++) {
                    float v = (current_axis == 'X') ? scale_x_results[i] : scale_y_results[i];
                    variance += (v - avg) * (v - avg);
                }
                float stddev = sqrtf(variance / NB_CALIB_STEPS);
                float cv     = (avg > 0) ? (stddev / avg * 100.0f) : 0.0f;
                Serial.printf("  CV : %.1f%%  ", cv);
                if      (cv < 3.0f) Serial.println(F("*** Excellente"));
                else if (cv < 7.0f) Serial.println(F("**  Bonne"));
                else                Serial.println(F("*   Mediocre — refaire la calibration"));

                state = IDLE;
                printCurrentScale();
                printMenu();

            } else {
                // ── Étape suivante ────────────────────────────────────────────
                Serial.printf("\n  Etape %u/%u — %.0f mm\n",
                              current_step+1, NB_CALIB_STEPS,
                              CALIB_DISTANCES_MM[current_step]);
                Serial.println(F("  -> Retournez au point de depart, appuyez sur une touche..."));
                state = WAITING_START;
            }
            break;
        }

        default: break;
    }
}