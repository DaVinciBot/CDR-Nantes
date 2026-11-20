#pragma once

#include <Arduino.h>

#include "mks_servo.h"

class MKSGroup {
   public:
    MKSGroup(MKSServo* wheel1, MKSServo* wheel2, MKSServo* wheel3);

    void normalize(double& w1, double& w2, double& w3, double maxAbsRpm) const;
    // False if a command could not be sent to at least one driver.
    bool setSpeedsSynced(double w1, double w2, double w3, uint8_t acc);
    void stopAll();
    void emergencyStopAll();
    bool readAllEncoders(int64_t& enc1, int64_t& enc2, int64_t& enc3);
    // True only if the three wheels answered. `fail`, when given, receives
    // why each wheel did not (ReadResult values, OK for a wheel that answered).
    bool readAllEncodersSynced(int64_t& enc1, int64_t& enc2, int64_t& enc3,
                               uint8_t* fail = nullptr);

   private:
    MKSServo* wheel1;
    MKSServo* wheel2;
    MKSServo* wheel3;
};
