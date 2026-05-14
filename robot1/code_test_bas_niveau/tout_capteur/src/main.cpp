#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <Bitcraze_PMW3901.h>
#include "config.h"

// ── IMU BNO085 ────────────────────────────────────────────────────────────────
Adafruit_BNO08x   bno08x(BNO085_RESET_PIN);
sh2_SensorValue_t sensorValue;

// ── Capteur optique PAA5100JE ─────────────────────────────────────────────────
// Utilise le bus SPI standard. CS est défini dans config.h (ex: pin 10)
Bitcraze_PMW3901  paa5100(PAA5100_CS_PIN);

// ── Flags et Variables de Calibration ─────────────────────────────────────────
bool bno085_ok  = false;
bool paa5100_ok = false;
long totalX = 0;
long totalY = 0;

// ── Timers ────────────────────────────────────────────────────────────────────
uint32_t g_last_sensor  = 0;
uint32_t g_last_encoder = 0;
uint32_t g_last_test    = 0;

// =============================================================================
// ── Moteurs — Utilitaires ────────────────────────────────────────────────────
// =============================================================================

uint8_t computeCRC(uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return sum & 0xFF;
}

void sendPacket(HardwareSerial& serial, uint8_t* pkt, uint8_t len) {
    pkt[len - 1] = computeCRC(pkt, len - 1);
    while (serial.available()) serial.read(); 
    serial.write(pkt, len);
    serial.flush();
}

void enableMotor(HardwareSerial& serial, uint8_t motor_num) {
    uint8_t pkt[5] = {250, MKS_ADDR, 243, 1, 0};
    sendPacket(serial, pkt, 5);
}

void spinMotor(HardwareSerial& serial, uint8_t motor_num, uint8_t dir, uint16_t rpm, uint8_t accel) {
    uint8_t byte4 = (dir << 7) | ((rpm >> 8) & 0x0F);
    uint8_t byte5 = (rpm & 0xFF);
    uint8_t pkt[7] = {250, MKS_ADDR, 246, byte4, byte5, accel, 0};
    sendPacket(serial, pkt, 7);
}

void stopMotor(HardwareSerial& serial, uint8_t motor_num) {
    uint8_t pkt[7] = {250, MKS_ADDR, 246, 0, 0, 0, 0};
    sendPacket(serial, pkt, 7);
}

int64_t readEncoder(HardwareSerial& serial, uint8_t motor_num) {
    uint8_t pkt[4] = {250, MKS_ADDR, 49, 0};
    sendPacket(serial, pkt, 4);
    uint8_t resp[10];
    uint8_t n = 0;
    uint32_t t = millis();
    while (n < 10 && millis() - t < 50) {
        if (serial.available()) { resp[n++] = serial.read(); t = millis(); }
    }
    if (n >= 9 && resp[0] == 0xFB && resp[1] == MKS_ADDR && resp[2] == 0x31) {
        int64_t val = 0;
        for (int i = 0; i < 6; i++) val = (val << 8) | (int64_t)resp[3 + i];
        if (val & (1LL << 47)) val |= (int64_t)0xFFFF000000000000LL;
        return val;
    }
    return INT64_MIN;
}

// =============================================================================
// ── Setup ─────────────────────────────────────────────────────────────────────
// =============================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(BAUDRATE);
    delay(1000);
    Serial.println("\n=== INITIALISATION SYSTEME ===");

    // 1. I2C — BNO085
    Wire.setSDA(PIN_SDA);
    Wire.setSCL(PIN_SCL);
    Wire.begin();
    Wire.setClock(400000);
    if (!bno08x.begin_I2C()) {
        Serial.println("[ERREUR] BNO085 non detecte");
    } else {
        bno08x.enableReport(SH2_ROTATION_VECTOR, 10000);
        bno085_ok = true;
        Serial.println("[BNO085] OK");
    }

    // 2. SPI — PAA5100JE (Force l'allumage via sequence d'init)
    Serial.println("[SPI] Init PAA5100JE...");
    pinMode(PAA5100_CS_PIN, OUTPUT);
    digitalWrite(PAA5100_CS_PIN, HIGH); // S'assurer que CS est HIGH au repos
    
    if (!paa5100.begin()) {
        Serial.println("[ERREUR] PAA5100JE non detecte (Verif câblage/alimentation)");
    } else {
        paa5100_ok = true;
        Serial.println("[PAA5100JE] OK - Lumieres activees");
    }

    // 3. UART — Moteurs
    W1_SERIAL.begin(MKS_BAUDRATE);
    W2_SERIAL.begin(MKS_BAUDRATE);
    W3_SERIAL.begin(MKS_BAUDRATE);
    Serial.println("[UART] Moteurs initialises");

    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("=== Systeme pret. Appuyez sur 'r' pour reset distance optique ===");
}

// =============================================================================
// ── Loop ──────────────────────────────────────────────────────────────────────
// =============================================================================
void loop() {
    uint32_t now = millis();

    // --- Gestion Reset via Console ---
    if (Serial.available()) {
        if (Serial.read() == 'r') {
            totalX = 0; totalY = 0;
            Serial.println("\n[RESET] Distances optiques remises a zero\n");
        }
    }

    // --- Lecture Capteurs @ 50Hz ---
    if (now - g_last_sensor >= SENSOR_INTERVAL_MS) {
        g_last_sensor = now;

        // BNO085 - Orientation
        if (bno085_ok && bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
                float qr = sensorValue.un.rotationVector.real;
                float qi = sensorValue.un.rotationVector.i;
                float qj = sensorValue.un.rotationVector.j;
                float qk = sensorValue.un.rotationVector.k;
                float yaw = atan2(2.0f*(qr*qk + qi*qj), 1.0f-2.0f*(qj*qj + qk*qk)) * 180.0f / PI;
                Serial.printf("[IMU] Yaw: %.2f°  ", yaw);
            }
        }

        // PAA5100JE - Calibration optique
        if (paa5100_ok) {
            int16_t dx, dy;
            paa5100.readMotionCount(&dx, &dy);
            totalX += dx;
            totalY += dy;
            
            // On affiche seulement si mouvement pour ne pas polluer la console
            if (dx != 0 || dy != 0) {
                Serial.printf("[OPTIC] dX:%d dY:%d | CumulX:%ld CumulY:%ld", dx, dy, totalX, totalY);
            }
        }
        Serial.println(); // Nouvelle ligne pour la lisibilité
    }

    // --- Lecture Encodeurs Moteurs @ 2Hz ---
    if (now - g_last_encoder >= ENCODER_INTERVAL_MS) {
        g_last_encoder = now;
        int64_t e1 = readEncoder(W1_SERIAL, 1);
        int64_t e2 = readEncoder(W2_SERIAL, 2);
        int64_t e3 = readEncoder(W3_SERIAL, 3);
        Serial.printf("[ENC] W1:%lld W2:%lld W3:%lld\n", e1, e2, e3);
    }
}