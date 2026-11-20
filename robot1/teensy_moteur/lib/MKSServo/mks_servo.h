#pragma once

#include <Arduino.h>

// Why a read failed, as reported to the logs. Same values as the RS485_*
// constants of the USB protocol (messages.h), which carries them per wheel.
enum class ReadResult : uint8_t {
    OK = 0,
    NO_ANSWER = 1,     // nothing that looked like a frame before the timeout
    BAD_CHECKSUM = 2,  // a whole frame, wrong checksum
    SHORT_FRAME = 3,   // the timeout cut a frame in the middle
    BAD_ADDRESS = 4,   // a valid frame, from another address or command
};

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
    ReadResult readEncoderResponse(int64_t& encoderCount, uint32_t timeoutMs = 50);
    
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
    // Returns OK, or why no valid frame was read before the timeout.
    ReadResult readResponse(uint8_t expectedCmd,
                      size_t expectedPayloadLen,
                      uint8_t* payload,
                      size_t payloadCapacity,
                      size_t& payloadLen,
                      uint32_t timeoutMs = 10);
};