/*
 * Fixed-size storage for the EKF: vectors and square covariance matrices.
 *
 * STORAGE, NOT ALGEBRA. There is deliberately no matrix product, no transpose
 * and no inverse here. A generic matrix library is the slow way to run a small
 * filter: pronenewbits/Embedded_EKF_Library (GitHub), fixed-size and
 * heap-free, benchmarked on this very MCU (iMXRT1062), spends 86 us on a 4-state
 * filter, while our budget is 40-80 us for 6 states. The filter code therefore works on these arrays directly and
 * exploits its own structure:
 *
 *   - every measurement update is scalar (R is diagonal, the sources are
 *     independent), so the innovation covariance is a float and nothing is ever
 *     inverted;
 *   - the state transition F is the identity plus a handful of terms, so
 *     F P F^T is expanded by hand instead of computed as two dense products.
 *
 * The dense, generic version of every equation still exists, in the host tests
 * only, as the reference the hand-written code must match (see
 * test/support/dense_reference.h).
 *
 * WHY NOT REUSE THE 2026 PROTOTYPE (archive/main-2026-04, kalman/matrix (1).hpp):
 * it stores its data as vector<vector<float>> and throws std::invalid_argument
 * on a dimension mismatch. Heap allocation and exceptions in a 100 Hz real-time
 * path are not a style question. Its Joseph-form update is kept, as an equation;
 * its implementation is not.
 *
 * This header must stay free of any Arduino include so that lib/kalman builds in
 * the "native" PlatformIO environment, where it is tested on the host.
 */

#ifndef MATRIX_H
#define MATRIX_H

#include <stddef.h>

namespace kalman {

/**
 * @brief Column vector of N floats, stored inline.
 *
 * An aggregate with no constructor: its content is undefined until assigned, so
 * start from zero() when in doubt.
 */
template <size_t N>
struct Vec {
    float v[N];

    float &operator[](size_t i) { return v[i]; }
    const float &operator[](size_t i) const { return v[i]; }

    static constexpr size_t size() { return N; }

    /** @return A vector with every element set to 0. */
    static Vec zero() {
        Vec out;
        for (size_t i = 0; i < N; ++i) {
            out.v[i] = 0.0f;
        }
        return out;
    }
};

/**
 * @brief N x N matrix of floats, stored inline, row-major.
 *
 * Only square matrices exist here: the filter never needs anything else once
 * every update is scalar.
 */
template <size_t N>
struct Mat {
    float m[N][N];

    float &operator()(size_t row, size_t col) { return m[row][col]; }
    const float &operator()(size_t row, size_t col) const { return m[row][col]; }

    static constexpr size_t size() { return N; }

    /** @return A matrix with every element set to 0. */
    static Mat zero() {
        Mat out;
        for (size_t i = 0; i < N; ++i) {
            for (size_t j = 0; j < N; ++j) {
                out.m[i][j] = 0.0f;
            }
        }
        return out;
    }

    /** @return The identity matrix. */
    static Mat identity() {
        Mat out = zero();
        for (size_t i = 0; i < N; ++i) {
            out.m[i][i] = 1.0f;
        }
        return out;
    }

    /** @return A diagonal matrix built from a vector. */
    static Mat diagonal(const Vec<N> &d) {
        Mat out = zero();
        for (size_t i = 0; i < N; ++i) {
            out.m[i][i] = d[i];
        }
        return out;
    }
};

/**
 * @brief Forces exact symmetry by averaging each pair of mirrored elements.
 *
 * A covariance computed in float32 drifts away from symmetric by rounding, and
 * an asymmetric P slowly loses positive-definiteness. Cheap insurance, run after
 * every step that writes P.
 *
 * @param P The matrix to symmetrise in place.
 */
template <size_t N>
void symmetrize(Mat<N> &P) {
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            const float mean = 0.5f * (P.m[i][j] + P.m[j][i]);
            P.m[i][j] = mean;
            P.m[j][i] = mean;
        }
    }
}

}  // namespace kalman

#endif  // MATRIX_H
