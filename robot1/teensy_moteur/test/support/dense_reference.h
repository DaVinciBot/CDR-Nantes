/*
 * Dense, generic reference implementation of the Kalman equations. TESTS ONLY.
 *
 * lib/kalman deliberately has no matrix product and no inverse: its predictions
 * are expanded by hand and its updates are scalar, because that is what fits the
 * real-time budget. Hand-expanded code is exactly where a sign or an index slips,
 * so every hand-written step is checked against this file, which writes the
 * textbook equations as plainly as possible:
 *
 *     P = F P F^T + Q
 *     K = P H^T (H P H^T + R)^-1
 *     x = x + K y
 *     P = (I - K H) P (I - K H)^T + K R K^T
 *
 * Everything here is in double, so the tolerance of a comparison measures the
 * float32 error of the production code, not the error of the reference.
 *
 * Not a test suite: PlatformIO only runs folders prefixed with test_. Include it
 * with a relative path, "../support/dense_reference.h".
 */

#ifndef DENSE_REFERENCE_H
#define DENSE_REFERENCE_H

#include <math.h>
#include <stddef.h>

#include "matrix.h"

namespace dense {

/** @brief R x C matrix of doubles. */
template <size_t R, size_t C>
struct DMat {
    double a[R][C];

    double &operator()(size_t r, size_t c) { return a[r][c]; }
    const double &operator()(size_t r, size_t c) const { return a[r][c]; }

    static DMat zero() {
        DMat out;
        for (size_t i = 0; i < R; ++i) {
            for (size_t j = 0; j < C; ++j) {
                out.a[i][j] = 0.0;
            }
        }
        return out;
    }
};

template <size_t N>
DMat<N, N> identity() {
    DMat<N, N> out = DMat<N, N>::zero();
    for (size_t i = 0; i < N; ++i) {
        out.a[i][i] = 1.0;
    }
    return out;
}

template <size_t R, size_t K, size_t C>
DMat<R, C> mul(const DMat<R, K> &lhs, const DMat<K, C> &rhs) {
    DMat<R, C> out = DMat<R, C>::zero();
    for (size_t i = 0; i < R; ++i) {
        for (size_t j = 0; j < C; ++j) {
            double sum = 0.0;
            for (size_t k = 0; k < K; ++k) {
                sum += lhs.a[i][k] * rhs.a[k][j];
            }
            out.a[i][j] = sum;
        }
    }
    return out;
}

template <size_t R, size_t C>
DMat<C, R> transpose(const DMat<R, C> &in) {
    DMat<C, R> out;
    for (size_t i = 0; i < R; ++i) {
        for (size_t j = 0; j < C; ++j) {
            out.a[j][i] = in.a[i][j];
        }
    }
    return out;
}

template <size_t R, size_t C>
DMat<R, C> add(const DMat<R, C> &lhs, const DMat<R, C> &rhs) {
    DMat<R, C> out;
    for (size_t i = 0; i < R; ++i) {
        for (size_t j = 0; j < C; ++j) {
            out.a[i][j] = lhs.a[i][j] + rhs.a[i][j];
        }
    }
    return out;
}

template <size_t R, size_t C>
DMat<R, C> sub(const DMat<R, C> &lhs, const DMat<R, C> &rhs) {
    DMat<R, C> out;
    for (size_t i = 0; i < R; ++i) {
        for (size_t j = 0; j < C; ++j) {
            out.a[i][j] = lhs.a[i][j] - rhs.a[i][j];
        }
    }
    return out;
}

/**
 * @brief Inverse by Gauss-Jordan elimination with partial pivoting.
 *
 * Only ever called on small, well-conditioned innovation covariances.
 */
template <size_t N>
DMat<N, N> inverse(const DMat<N, N> &in) {
    DMat<N, N> work = in;
    DMat<N, N> inv = identity<N>();
    for (size_t col = 0; col < N; ++col) {
        size_t pivot = col;
        for (size_t row = col + 1; row < N; ++row) {
            if (fabs(work.a[row][col]) > fabs(work.a[pivot][col])) {
                pivot = row;
            }
        }
        for (size_t k = 0; k < N; ++k) {
            double tmp = work.a[col][k];
            work.a[col][k] = work.a[pivot][k];
            work.a[pivot][k] = tmp;
            tmp = inv.a[col][k];
            inv.a[col][k] = inv.a[pivot][k];
            inv.a[pivot][k] = tmp;
        }
        const double scale = work.a[col][col];
        for (size_t k = 0; k < N; ++k) {
            work.a[col][k] /= scale;
            inv.a[col][k] /= scale;
        }
        for (size_t row = 0; row < N; ++row) {
            if (row == col) {
                continue;
            }
            const double factor = work.a[row][col];
            for (size_t k = 0; k < N; ++k) {
                work.a[row][k] -= factor * work.a[col][k];
                inv.a[row][k] -= factor * inv.a[col][k];
            }
        }
    }
    return inv;
}

/**
 * @brief Lower-triangular L with L L^T = A, for a symmetric positive-definite A.
 *
 * Used to draw correlated noise: if n is standard normal, L n has covariance A.
 * A pivot that is not strictly positive is clamped to zero, so a semi-definite
 * A still yields a usable (rank-deficient) factor.
 */
template <size_t N>
DMat<N, N> cholesky(const DMat<N, N> &A) {
    DMat<N, N> L = DMat<N, N>::zero();
    for (size_t j = 0; j < N; ++j) {
        double diag = A.a[j][j];
        for (size_t k = 0; k < j; ++k) {
            diag -= L.a[j][k] * L.a[j][k];
        }
        L.a[j][j] = diag > 0.0 ? sqrt(diag) : 0.0;
        for (size_t i = j + 1; i < N; ++i) {
            double sum = A.a[i][j];
            for (size_t k = 0; k < j; ++k) {
                sum -= L.a[i][k] * L.a[j][k];
            }
            L.a[i][j] = L.a[j][j] > 0.0 ? sum / L.a[j][j] : 0.0;
        }
    }
    return L;
}

// === CONVERSIONS FROM lib/kalman ===

template <size_t N>
DMat<N, 1> from(const kalman::Vec<N> &v) {
    DMat<N, 1> out;
    for (size_t i = 0; i < N; ++i) {
        out.a[i][0] = v[i];
    }
    return out;
}

template <size_t N>
DMat<N, N> from(const kalman::Mat<N> &m) {
    DMat<N, N> out;
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = 0; j < N; ++j) {
            out.a[i][j] = m(i, j);
        }
    }
    return out;
}

// === REFERENCE EQUATIONS ===

/** @brief P = F P F^T + Q, as two dense products. */
template <size_t N>
DMat<N, N> predict_covariance(const DMat<N, N> &P, const DMat<N, N> &F,
                              const DMat<N, N> &Q) {
    return add(mul(mul(F, P), transpose(F)), Q);
}

/**
 * @brief Joint update with M measurements at once, Joseph form, full inverse.
 *
 * @param x State, updated in place.
 * @param P Covariance, updated in place.
 * @param H Measurement matrix, M x N.
 * @param y Innovation vector z - h(x), M x 1.
 * @param R Measurement noise covariance, M x M.
 */
template <size_t N, size_t M>
void joint_update(DMat<N, 1> &x, DMat<N, N> &P, const DMat<M, N> &H,
                  const DMat<M, 1> &y, const DMat<M, M> &R) {
    const DMat<N, M> Ht = transpose(H);
    const DMat<M, M> S = add(mul(mul(H, P), Ht), R);
    const DMat<N, M> K = mul(mul(P, Ht), inverse(S));
    x = add(x, mul(K, y));
    const DMat<N, N> I_KH = sub(identity<N>(), mul(K, H));
    P = add(mul(mul(I_KH, P), transpose(I_KH)), mul(mul(K, R), transpose(K)));
}

}  // namespace dense

#endif  // DENSE_REFERENCE_H
