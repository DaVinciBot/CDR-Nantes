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

    // Purge RX uniquement pour les commandes de contrôle (pas pour 0x31 readEncoder)
    // En mode rafale, la réponse du moteur peut arriver pendant l'envoi suivant
    if (cmd != 0x31) {
        while (serial.available() > 0) {
            (void)serial.read();
        }
    }

    serial.write(frame, frameLen);
    serial.flush();
    return true;
}

bool MKSServo::readResponse(uint8_t expectedCmd,
                            uint8_t* payload,
                            size_t payloadCapacity,
                            size_t& payloadLen,
                            uint32_t timeoutMs) {
    payloadLen = 0;
    const uint32_t start = millis();
    uint8_t frame[32];
    size_t frameLen = 0;
    bool receiving = false;

    while ((millis() - start) < timeoutMs) {
        while (serial.available() > 0) {
            const int value = serial.read();
            if (value < 0) continue;

            const uint8_t b = static_cast<uint8_t>(value);

            if (!receiving) {
                if (b == kUplinkHead) {
                    frame[0] = b;
                    frameLen = 1;
                    receiving = true;
                }
                continue;
            }

            if (frameLen < sizeof(frame)) {
                frame[frameLen++] = b;
            } else {
                receiving = false;
                frameLen = 0;
                continue;
            }

            if (frameLen >= 4) {
                const uint8_t crc = computeCRC(frame, frameLen - 1);
                if (crc == frame[frameLen - 1]) {
                    if (frame[0] != kUplinkHead || frame[1] != addr || frame[2] != expectedCmd) {
                        receiving = false;
                        frameLen = 0;
                        continue;
                    }
                    const size_t inPayloadLen = frameLen - 4;
                    if (inPayloadLen > payloadCapacity) return false;
                    for (size_t i = 0; i < inPayloadLen; ++i) {
                        payload[i] = frame[3 + i];
                    }
                    payloadLen = inPayloadLen;
                    return true;
                }
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
    const bool ccw       = rpm < 0.0;
    const uint16_t speed = static_cast<uint16_t>(fabs(rpm));

    // Format F6 : [dir(7)|speed_high(3..0)] [speed_low] [acc 0-255]
    uint8_t payload[3];
    payload[0] = static_cast<uint8_t>((ccw ? 0x80 : 0x00) | ((speed >> 8) & 0x0F));
    payload[1] = static_cast<uint8_t>(speed & 0xFF);
    payload[2] = acc;  // V1.0.9 : 0-255, limite V1.0 à 32 supprimée

    return sendPacket(0xF6, payload, sizeof(payload));
    // Fire & forget: pas d'attente de réponse 0xF6 (UartRSP=Disable au niveau HW)
}

// ─── STOP / EMERGENCY STOP ───────────────────────────────────────────────────
// stop()          → décélération contrôlée acc=5, fin de trajectoire normale
// emergencyStop() → coupure immédiate acc=0, réservé aux cas de sécurité

bool MKSServo::stop() {
    return setSpeed(0.0, 5);   // rampe douce, évite les à-coups mécaniques
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
    uint8_t payload[8];
    size_t payloadLen = 0;
    if (!readResponse(0x31, payload, sizeof(payload), payloadLen, timeoutMs)) {
        return false;
    }

    if (payloadLen < 6) return false;

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
        uint8_t rsp[4];
        size_t rspLen = 0;
        if (readResponse(0x80, rsp, sizeof(rsp), rspLen, 100)) {
            if (rspLen >= 1) {
                if (rsp[0] == 1) return true;
                if (rsp[0] == 2) return false;
            }
        }
        delay(10);
    }
    return false;
}