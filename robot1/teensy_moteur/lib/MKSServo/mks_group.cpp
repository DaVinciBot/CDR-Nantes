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

void MKSGroup::setSpeedsSynced(double w1, double w2, double w3, uint8_t acc) {
    normalize(w1, w2, w3, MAX_SPEED_RPM);

    if (wheel1) {
        wheel1->setSpeed(w1, acc);
    }
    if (wheel2) {
        wheel2->setSpeed(w2, acc);
    }
    if (wheel3) {
        wheel3->setSpeed(w3, acc);
    }
}

void MKSGroup::stopAll() {
    if (wheel1) {
        wheel1->stop();
    }
    if (wheel2) {
        wheel2->stop();
    }
    if (wheel3) {
        wheel3->stop();
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

bool MKSGroup::readAllEncodersSynced(int64_t& enc1, int64_t& enc2, int64_t& enc3) {
    enc1 = enc2 = enc3 = 0;
    
    // Phase 1 : Envoyer 3 requêtes 0x31 en rafale (~0.3ms de décalage)
    bool send_ok = true;
    if (wheel1) {
        send_ok = wheel1->sendReadRequest() && send_ok;
    }
    if (wheel2) {
        send_ok = wheel2->sendReadRequest() && send_ok;
    }
    if (wheel3) {
        send_ok = wheel3->sendReadRequest() && send_ok;
    }
    
    if (!send_ok) return false;
    
    // Phase 2 : Lire 3 réponses (timeout = 50ms par servo, mais réponses en //è)
    bool ok = true;
    if (wheel1) {
        ok = wheel1->readEncoderResponse(enc1, 50) && ok;
    }
    if (wheel2) {
        ok = wheel2->readEncoderResponse(enc2, 50) && ok;
    }
    if (wheel3) {
        ok = wheel3->readEncoderResponse(enc3, 50) && ok;
    }
    
    return ok;
}
