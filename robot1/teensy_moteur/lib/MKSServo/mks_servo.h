#pragma once

#include <Arduino.h>

class MKSServo {
   public:
    MKSServo(HardwareSerial& serial, uint8_t addr, uint8_t mstep);

    void begin(uint32_t baudrate);
    bool enable();
    bool disable();
    bool setSpeed(double rpm, uint8_t acc);
    bool stop();           // décélération douce acc=5
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

    // Timeouts :
    //   F3 enable/disable → 10ms (réponse 4 bytes ≈ 350µs + latence)
    //   F6 setSpeed       → 10ms (idem)
    //   0x31 readEncoder  → 50ms (réponse 9 bytes, lecture critique)
    bool readResponse(uint8_t expectedCmd,
                      uint8_t* payload,
                      size_t payloadCapacity,
                      size_t& payloadLen,
                      uint32_t timeoutMs = 10);
};