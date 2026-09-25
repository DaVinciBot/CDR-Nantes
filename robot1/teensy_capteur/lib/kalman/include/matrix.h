/*
 * Static matrix algebra for the EKF. PLACEHOLDER — the first EKF step.
 *
 * Planned content: a Mat<R, C> template with its storage inline (no heap, no
 * exceptions, no std::vector), in float, with the operations the filter needs:
 * product, transpose, sum, difference, identity, and a small symmetric inverse
 * for the innovation covariance S.
 *
 * WHY NOT REUSE THE 2026 PROTOTYPE (archive/main-2026-04, kalman/matrix (1).hpp):
 * it stores its data as vector<vector<float>> and throws std::invalid_argument
 * on a dimension mismatch. Heap allocation and exceptions in a 100 Hz real-time
 * path are not a style question. Its Joseph-form update and its overall class
 * shape are worth keeping as a reference; its implementation is not.
 *
 * This header must stay free of any Arduino include so that lib/kalman builds in
 * the "native" PlatformIO environment, where the algebra is tested on the host.
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stddef.h>

namespace kalman {

// Matrix algebra goes here.

}  // namespace kalman

#endif  // MATRIX_H
