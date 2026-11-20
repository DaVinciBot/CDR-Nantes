#include "mks_group.h"

#include <math.h>

#include <config.h>

MKSGroup::MKSGroup(MKSServo* wheel1, MKSServo* wheel2, MKSServo* wheel3)
    : wheel1(wheel1), wheel2(wheel2), wheel3(wheel3) {}

void MKSGroup::normalize(double& w1, double& w2, double& w3, double maxAbsRpm) const {
    const double maxWheel = fmax(fmax(fabs(w1), fabs(w2)), fabs(w3));
    if (maxWheel > maxAbsRpm && maxWheel > 0.0) {
        const double scale = maxAbsRpm / maxWheel;
        w1 *= scale;
        w2 *= scale;
        w3 *= scale;
    }
}

bool MKSGroup::setSpeedsSynced(double w1, double w2, double w3, uint8_t acc) {
    normalize(w1, w2, w3, MAX_SPEED_RPM);

    bool ok = true;
    if (wheel1) {
        ok = wheel1->setSpeed(w1, acc) && ok;
    }
    if (wheel2) {
        ok = wheel2->setSpeed(w2, acc) && ok;
    }
    if (wheel3) {
        ok = wheel3->setSpeed(w3, acc) && ok;
    }
    return ok;
}

void MKSGroup::stopAll() {
    if (wheel1) {
        wheel1->stop(MKS_ACC);
    }
    if (wheel2) {
        wheel2->stop(MKS_ACC);
    }
    if (wheel3) {
        wheel3->stop(MKS_ACC);
    }
}

void MKSGroup::emergencyStopAll() {
    if (wheel1) {
        wheel1->emergencyStop();
    }
    if (wheel2) {
        wheel2->emergencyStop();
    }
    if (wheel3) {
        wheel3->emergencyStop();
    }
}

bool MKSGroup::readAllEncoders(int64_t& enc1, int64_t& enc2, int64_t& enc3) {
    bool ok = true;
    if (wheel1) {
        ok = wheel1->readEncoder(enc1) && ok;
    } else {
        enc1 = 0;
    }

    if (wheel2) {
        ok = wheel2->readEncoder(enc2) && ok;
    } else {
        enc2 = 0;
    }

    if (wheel3) {
        ok = wheel3->readEncoder(enc3) && ok;
    } else {
        enc3 = 0;
    }

    return ok;
}

// ─── READ ALL ENCODERS SYNCED ─────────────────────────────────────────────────────
// Envoi en rafale ~0.3ms de décalage total, puis lectures en parallèle
// Au lieu de : send(0x31 wheel1) → wait response → send(0x31 wheel2) → wait → etc
// On fait : send(0x31 wheel1) → send(0x31 wheel2) → send(0x31 wheel3) → 
//           read response wheel1 → read response wheel2 → read response wheel3
// Les servos répondent en parallèle, gain de ~30-40ms en pratique

bool MKSGroup::readAllEncodersSynced(int64_t& enc1, int64_t& enc2, int64_t& enc3,
                                     uint8_t* fail) {
    enc1 = enc2 = enc3 = 0;
    MKSServo* wheels[3] = {wheel1, wheel2, wheel3};
    int64_t* counts[3] = {&enc1, &enc2, &enc3};
    ReadResult result[3] = {ReadResult::OK, ReadResult::OK, ReadResult::OK};

    // Phase 1: the three 0x31 requests in a burst (~0.3 ms apart)
    bool sent[3] = {false, false, false};
    for (int i = 0; i < 3; ++i) {
        if (wheels[i]) sent[i] = wheels[i]->sendReadRequest();
    }

    // Phase 2: the three answers (50 ms timeout each; the drivers answer in
    // parallel). Every wheel is read, so a failure is reported per wheel.
    bool ok = true;
    for (int i = 0; i < 3; ++i) {
        if (!wheels[i]) continue;
        result[i] = sent[i] ? wheels[i]->readEncoderResponse(*counts[i], 50)
                            : ReadResult::NO_ANSWER;
        if (result[i] != ReadResult::OK) ok = false;
    }

    if (fail) {
        for (int i = 0; i < 3; ++i) fail[i] = static_cast<uint8_t>(result[i]);
    }
    return ok;
}

