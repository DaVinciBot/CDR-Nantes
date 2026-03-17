#pragma once

#include <Arduino.h>

class MKSServo {
   public:
    MKSServo(HardwareSerial& serial, uint8_t dePin, uint8_t addr, uint8_t mstep);

    void begin(uint32_t baudrate);
    bool enable();
    bool disable();
    bool setSpeed(double rpm, uint8_t acc);
    bool stop();
    bool emergencyStop();
    bool readEncoder(int64_t& encoderCount);
    bool calibrate(uint32_t timeoutMs = 15000);

   private:
    HardwareSerial& serial;
    uint8_t dePin;
    uint8_t addr;
    uint8_t mstep;

    uint8_t computeCRC(const uint8_t* data, size_t len) const;
    bool sendPacket(uint8_t cmd, const uint8_t* payload, size_t payloadLen);
    bool readResponse(uint8_t expectedCmd,
                      uint8_t* payload,
                      size_t payloadCapacity,
                      size_t& payloadLen,
                      uint32_t timeoutMs = 20);
    uint16_t rpmToMKSSpeed(double rpm) const;

    bool sendSimpleCommand(uint8_t cmd, bool waitAck, uint8_t expectedStatus = 1);
};
