/*
 * EKF core: angle wrapping and NIS statistics. The scalar update is a template
 * and lives in include/ekf.h. See that header for the model and the traps.
 *
 * No Arduino dependency: this file must build in the "native" environment.
 */

#include "ekf.h"

#include <math.h>

namespace kalman {

float wrap_pi(float angle) {
    float wrapped = angle - TWO_PI_F * floorf((angle + PI_F) / TWO_PI_F);
    // Float rounding can land exactly on +pi (e.g. from just below -pi); the
    // interval is half-open, so fold it back.
    if (wrapped >= PI_F) {
        wrapped -= TWO_PI_F;
    }
    return wrapped;
}

void NisStats::record(const UpdateResult &result) {
    if (result.accepted) {
        ++accepted;
        nis_sum += result.nis;
    } else {
        ++rejected;
    }
}

float NisStats::mean_nis() const {
    return accepted > 0 ? nis_sum / static_cast<float>(accepted) : 0.0f;
}

void NisStats::reset() {
    accepted = 0;
    rejected = 0;
    nis_sum = 0.0f;
}

}  // namespace kalman
