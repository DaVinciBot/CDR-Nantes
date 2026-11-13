#include "mks_servo.h"

#include <math.h>

namespace {
constexpr uint8_t kDownlinkHead = 0xFA;
constexpr uint8_t kUplinkHead   = 0xFB;
}

MKSServo::MKSServo(HardwareSerial& serial, uint8_t addr, uint8_t mstep)
    : serial(serial), addr(addr), mstep(mstep) {}

void MKSServo::begin(uint32_t baudrate) {
    serial.begin(baudrate);
    delay(10);
}

uint8_t MKSServo::computeCRC(const uint8_t* data, size_t len) const {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

bool MKSServo::sendPacket(uint8_t cmd, const uint8_t* payload, size_t payloadLen) {
    const size_t frameLen = 3 + payloadLen + 1;
    if (frameLen > 32) return false;

    uint8_t frame[32];
    frame[0] = kDownlinkHead;
    frame[1] = addr;
    frame[2] = cmd;
    for (size_t i = 0; i < payloadLen; ++i) {
        frame[3 + i] = payload[i];
    }
    frame[3 + payloadLen] = computeCRC(frame, 3 + payloadLen);

    // Drop whatever is left in RX before a request, so that its response is the
    // first frame read back. Safe for the encoder burst too: each wheel has its
    // own UART, so sending to one driver never purges another one's response.
    while (serial.available() > 0) {
        (void)serial.read();
    }

    serial.write(frame, frameLen);
    serial.flush();
    return true;
}

// A response is [0xFB][addr][cmd][payload][checksum], checksum = sum of the
// bytes before it, modulo 256. Its length is fixed by the command, so the frame
// is cut at that length and only then checked. Testing the checksum at every
// byte instead (the end of the frame guessed from a sum that happens to match)
// cut encoder frames short whenever a payload byte equalled the running sum:
// about 1.6 % of the 0x31 reads per wheel, and a whole 256-count window every
// 65536 counts where every read of that wheel failed.
bool MKSServo::readResponse(uint8_t expectedCmd,
                            size_t expectedPayloadLen,
                            uint8_t* payload,
                            size_t payloadCapacity,
                            size_t& payloadLen,
                            uint32_t timeoutMs) {
    payloadLen = 0;
    uint8_t frame[32];
    const size_t frameLen = 4 + expectedPayloadLen;
    if (frameLen > sizeof(frame) || expectedPayloadLen > payloadCapacity) return false;

    const uint32_t start = millis();
    size_t received = 0;

    while ((millis() - start) < timeoutMs) {
        while (serial.available() > 0) {
            const int value = serial.read();
            if (value < 0) continue;
            const uint8_t b = static_cast<uint8_t>(value);

            // Wait for a header before storing anything.
            if (received == 0 && b != kUplinkHead) continue;
            frame[received++] = b;
            if (received < frameLen) continue;

            const bool valid = computeCRC(frame, frameLen - 1) == frame[frameLen - 1] &&
                               frame[1] == addr && frame[2] == expectedCmd;
            if (valid) {
                for (size_t i = 0; i < expectedPayloadLen; ++i) {
                    payload[i] = frame[3 + i];
                }
                payloadLen = expectedPayloadLen;
                return true;
            }

            // Not our frame (a corrupted byte, or a header that was a payload
            // byte): restart from the next header among the bytes already read
            // rather than dropping them all.
            size_t next = 1;
            while (next < frameLen && frame[next] != kUplinkHead) ++next;
            received = frameLen - next;
            for (size_t i = 0; i < received; ++i) {
                frame[i] = frame[next + i];
            }
        }
    }
    return false;
}

// ─── ENABLE / DISABLE ────────────────────────────────────────────────────────

bool MKSServo::enable() {
    const uint8_t payload[1] = {0x01};
    return sendPacket(0xF3, payload, sizeof(payload));
    // Fire & forget: pas d'attente de réponse 0xF3
}

bool MKSServo::disable() {
    const uint8_t payload[1] = {0x00};
    return sendPacket(0xF3, payload, sizeof(payload));
    // Fire & forget: pas d'attente de réponse 0xF3
}

// ─── SET SPEED ───────────────────────────────────────────────────────────────
// Réponses F6 :
//   0x01 → commande acceptée / moteur en accélération
//   0x02 → moteur à vitesse de croisière
//   0x00 → commande stop (speed=0) acquittée : moteur arrêté. SUCCÈS, pas un refus.
//
// Timeout 10ms : réponse 4 bytes ≈ 350µs à 115200 bauds + latence RS485 ~1ms.
// Si UartRSP=Disable → timeout systématique → return true (commande envoyée, moteur tourne).
// Si UartRSP=Enable  → réponse reçue, debug disponible.

bool MKSServo::setSpeed(double rpm, uint8_t acc) {
    const bool ccw = rpm < 0.0;
    // Rounded, not truncated: truncation biases every wheel towards zero by a
    // different amount (up to 1 rpm = 3.1 mm/s) and skews the direction at low
    // speed. The speed field is 12 bits.
    long rounded = lround(fabs(rpm));
    if (rounded > 0x0FFF) rounded = 0x0FFF;
    const uint16_t speed = static_cast<uint16_t>(rounded);

    // Format F6 : [dir(7)|speed_high(3..0)] [speed_low] [acc 0-255]
    uint8_t payload[3];
    payload[0] = static_cast<uint8_t>((ccw ? 0x80 : 0x00) | ((speed >> 8) & 0x0F));
    payload[1] = static_cast<uint8_t>(speed & 0xFF);
    payload[2] = acc;  // V1.0.9 : 0-255, limite V1.0 à 32 supprimée

    return sendPacket(0xF6, payload, sizeof(payload));
    // Fire & forget: pas d'attente de réponse 0xF6 (UartRSP=Disable au niveau HW)
}

// ─── STOP / EMERGENCY STOP ───────────────────────────────────────────────────
// stop(acc)       → controlled deceleration on the driver ramp (the caller
//                   passes MKS_ACC: at the old acc = 5, ~0.25 m/s^2, stopping
//                   from 1 m/s took 4 s and 2 m)
// emergencyStop() → coupure immédiate acc=0, réservé aux cas de sécurité

bool MKSServo::stop(uint8_t acc) {
    return setSpeed(0.0, acc);
}

bool MKSServo::emergencyStop() {
    return setSpeed(0.0, 0);   // arrêt immédiat sans rampe
}

// ─── READ ENCODER ASYNCHRONE ─────────────────────────────────────────────────────
// Protocole asynchrone en 2 phases pour rafale synchronisée :
//   1. sendReadRequest()    → envoie requête 0x31 sans attendre
//   2. readEncoderResponse() → lit réponse 0x31 avec timeout
//
// Permet à mksGroup->readAllEncodersSynced() d'envoyer 3 requêtes en rafale,
// puis de lire 3 réponses (les servos répondent en parallèle).

bool MKSServo::sendReadRequest() {
    return sendPacket(0x31, nullptr, 0);
    // Fire & forget: envoie la requête, ne bloque pas
}

bool MKSServo::readEncoderResponse(int64_t& encoderCount, uint32_t timeoutMs) {
    encoderCount = 0;
    // 0x31 answers a signed 48-bit count: FB addr 31 [6 bytes] checksum.
    uint8_t payload[6];
    size_t payloadLen = 0;
    if (!readResponse(0x31, sizeof(payload), payload, sizeof(payload), payloadLen,
                      timeoutMs)) {
        return false;
    }

    int64_t value = 0;
    for (size_t i = 0; i < 6; ++i) {
        value = (value << 8) | payload[i];
    }

    // Extension de signe 48 bits → int64_t
    if (value & (1LL << 47)) {
        value |= ~((1LL << 48) - 1);
    }

    encoderCount = value;
    return true;
}

// ─── READ ENCODER (synchrone, wrapper) ───────────────────────────────────────────
// Interface synchrone existante : envoie requête + attend réponse dans le même appel.
// Utilise les 2 primitives asynchrones.

bool MKSServo::readEncoder(int64_t& encoderCount) {
    if (!sendReadRequest()) return false;
    return readEncoderResponse(encoderCount, 50);
}

// ─── CALIBRATION ─────────────────────────────────────────────────────────────

bool MKSServo::calibrate(uint32_t timeoutMs) {
    const uint8_t payload[1] = {0x00};
    if (!sendPacket(0x80, payload, sizeof(payload))) return false;

    const uint32_t start = millis();
    while ((millis() - start) < timeoutMs) {
        // FB addr 80 [status] checksum
        uint8_t rsp[1];
        size_t rspLen = 0;
        if (readResponse(0x80, sizeof(rsp), rsp, sizeof(rsp), rspLen, 100)) {
            if (rspLen >= 1) {
                if (rsp[0] == 1) return true;
                if (rsp[0] == 2) return false;
            }
        }
        delay(10);
    }
    return false;
}