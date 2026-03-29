#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include "Matrix.hpp"
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <iostream>

using namespace std;

class KalmanFilter {
private:
    size_t stateDim;
    size_t measDim;
    float  dt;

    Matrix x;  // État estimé         (stateDim × 1)
    Matrix P;  // Covariance           (stateDim × stateDim)
    Matrix F;  // Transition           (stateDim × stateDim)
    Matrix Q;  // Bruit processus      (stateDim × stateDim)
    Matrix H;  // Observation          (measDim  × stateDim)
    Matrix R;  // Bruit mesure         (measDim  × measDim)

public:
    KalmanFilter(size_t stateDim, size_t measDim, float dt)
        : stateDim(stateDim), measDim(measDim), dt(dt),
          x(stateDim, 1),
          P(stateDim, stateDim),
          F(stateDim, stateDim),
          Q(stateDim, stateDim),
          H(measDim, stateDim),
          R(measDim, measDim)
    {
        initializeDefaultMatrices();
    }

    void initializeDefaultMatrices() {
        for (size_t i = 0; i < stateDim; i++)
            x(i, 0) = 0.0f;

        P = Matrix::identity(stateDim) * 100.0f;
        F = Matrix::identity(stateDim);
        Q = Matrix::identity(stateDim) * 0.01f;
        H = Matrix::identity(measDim);
        R = Matrix::identity(measDim) * 0.1f;
    }

    // ── Setters ────────────────────────────────────────────────────────────────

    void setState(const Matrix& initialState) {
        if (initialState.getRows() != stateDim || initialState.getCols() != 1)
            throw invalid_argument("Dimension de l'état incorrecte");
        x = initialState;
    }

    void setF(const Matrix& transitionMatrix) {
        if (transitionMatrix.getRows() != stateDim || transitionMatrix.getCols() != stateDim)
            throw invalid_argument("Dimension de F incorrecte");
        F = transitionMatrix;
    }

    void setQ(const Matrix& processNoise) {
        if (processNoise.getRows() != stateDim || processNoise.getCols() != stateDim)
            throw invalid_argument("Dimension de Q incorrecte");
        Q = processNoise;
    }

    void setH(const Matrix& observationMatrix) {
        if (observationMatrix.getRows() != measDim || observationMatrix.getCols() != stateDim)
            throw invalid_argument("Dimension de H incorrecte");
        H = observationMatrix;
    }

    void setR(const Matrix& measurementNoise) {
        if (measurementNoise.getRows() != measDim || measurementNoise.getCols() != measDim)
            throw invalid_argument("Dimension de R incorrecte");
        R = measurementNoise;
    }

    void setP(const Matrix& covariance) {
        if (covariance.getRows() != stateDim || covariance.getCols() != stateDim)
            throw invalid_argument("Dimension de P incorrecte");
        P = covariance;
    }

    // ── Cœur du filtre ────────────────────────────────────────────────────────

    // Prédiction : x = F·x   |   P = F·P·Fᵀ + Q
    void predict() {
        x = F * x;
        P = F * P * F.transpose() + Q;
    }

    // Correction : y = z - H·x   |   S = H·P·Hᵀ + R
    //              K = P·Hᵀ·S⁻¹  |   x = x + K·y   |   P = (I - K·H)·P
    void update(const Matrix& z) {
        if (z.getRows() != measDim || z.getCols() != 1)
            throw invalid_argument("Dimension de la mesure incorrecte");

        Matrix y   = z - H * x;
        Matrix S   = H * P * H.transpose() + R;
        Matrix K   = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Matrix::identity(stateDim) - K * H) * P;
    }

    // ── Getters ───────────────────────────────────────────────────────────────

    Matrix getState()      const { return x; }
    Matrix getCovariance() const { return P; }

    float getStateElement(size_t index) const {
        if (index >= stateDim) throw out_of_range("Index hors limites");
        return x(index, 0);
    }

    float getUncertainty(size_t index) const {
        if (index >= stateDim) throw out_of_range("Index hors limites");
        return sqrt(P(index, index));
    }

    // ── Debug ─────────────────────────────────────────────────────────────────

    void printState() const {
        cout << "État estimé :\n";
        for (size_t i = 0; i < stateDim; i++) {
            cout << "  x[" << i << "] = "
                 << setw(10) << fixed << setprecision(4)
                 << x(i, 0) << " ± " << getUncertainty(i) << "\n";
        }
        cout << endl;
    }

    void printDebug() const {
        x.print("État x");
        P.print("Covariance P");
        F.print("Transition F");
        Q.print("Bruit processus Q");
        H.print("Observation H");
        R.print("Bruit mesure R");
    }
};

#endif // KALMAN_FILTER_HPP
