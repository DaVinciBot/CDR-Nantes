#pragma once

#include <Arduino.h>

class MKSServo {
   public:
    MKSServo(HardwareSerial& serial, uint8_t addr, uint8_t mstep);

    void begin(uint32_t baudrate);
    bool enable();
    bool disable();
    bool setSpeed(double rpm, uint8_t acc);
    bool stop(uint8_t acc);  // controlled deceleration on the driver ramp
    bool emergencyStop();  // coupure immédiate acc=0
    bool readEncoder(int64_t& encoderCount);
    
    // Lecture encodeur asynchrone (rafale synchronisée)
    bool sendReadRequest();  // Envoie 0x31 sans attendre réponse
    bool readEncoderResponse(int64_t& encoderCount, uint32_t timeoutMs = 50);
    
    bool calibrate(uint32_t timeoutMs = 15000);

   private:
    HardwareSerial& serial;
    uint8_t addr;
    uint8_t mstep;

    uint8_t computeCRC(const uint8_t* data, size_t len) const;
    bool sendPacket(uint8_t cmd, const uint8_t* payload, size_t payloadLen);

    // Reads one response of a known length: expectedPayloadLen bytes between
    // [0xFB][addr][cmd] and the checksum (6 for 0x31, 1 for 0x80).
    // Timeouts:
    //   0x31 readEncoder → 50 ms by default (10-byte response, 0.9 ms at 115200)
    //   0x80 calibrate   → 100 ms per poll
    bool readResponse(uint8_t expectedCmd,
                      size_t expectedPayloadLen,
                      uint8_t* payload,
                      size_t payloadCapacity,
                      size_t& payloadLen,
                      uint32_t timeoutMs = 10);
};