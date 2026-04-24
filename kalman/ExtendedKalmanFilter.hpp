#ifndef EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_HPP

#include "matrix.hpp"
#include <functional>
#include <cmath>
#include <stdexcept>
#include <iomanip>
#include <iostream>

using namespace std;

// ============================================================
//  Extended Kalman Filter (EKF)
// ============================================================
//
//  Modèle :
//    x_k  = f(x_{k-1}) + w_k        w ~ N(0, Q)   bruit processus
//    z_k  = h(x_k)     + v_k        v ~ N(0, R)   bruit mesure
//
//  Les jacobiennes Fj = ∂f/∂x  et  Hj = ∂h/∂x  sont calculées
//  numériquement (différences finies centrées) à chaque pas via
//  Matrix::jacobian().  On peut aussi les fournir analytiquement
//  via setFjacobian() / setHjacobian() pour plus de précision.
//
//  Étape de prédiction :
//    x̂⁻  = f(x̂)
//    P⁻   = Fj · P · Fjᵀ + Q
//
//  Étape de correction :
//    y    = z − h(x̂⁻)
//    S    = Hj · P⁻ · Hjᵀ + R
//    K    = P⁻ · Hjᵀ · S⁻¹
//    x̂    = x̂⁻ + K · y
//    P    = (I − K · Hj) · P⁻
// ============================================================

class ExtendedKalmanFilter {
public:
    using VecFn = function<Matrix(const Matrix&)>;

private:
    size_t stateDim;
    size_t measDim;
    float  dt;

    Matrix x;  // État estimé         (stateDim × 1)
    Matrix P;  // Covariance           (stateDim × stateDim)
    Matrix Q;  // Bruit processus      (stateDim × stateDim)
    Matrix R;  // Bruit mesure         (measDim  × measDim)

    // Fonctions non linéaires
    VecFn  f_fn;   // transition d'état  x → f(x)
    VecFn  h_fn;   // fonction d'observation x → h(x)

    // Jacobiennes optionnelles (analytiques)
    // Si nulles → calcul numérique automatique
    function<Matrix(const Matrix&)> Fj_fn;
    function<Matrix(const Matrix&)> Hj_fn;

    float jacEps = 1e-4f;   // pas différences finies

public:
    ExtendedKalmanFilter(size_t stateDim, size_t measDim, float dt)
        : stateDim(stateDim), measDim(measDim), dt(dt),
          x(stateDim, 1),
          P(stateDim, stateDim),
          Q(stateDim, stateDim),
          R(measDim, measDim)
    {
        initializeDefaultMatrices();
    }

    void initializeDefaultMatrices() {
        for (size_t i = 0; i < stateDim; i++)
            x(i, 0) = 0.0f;

        P = Matrix::identity(stateDim) * 100.0f;
        Q = Matrix::identity(stateDim) * 0.01f;
        R = Matrix::identity(measDim)  * 0.1f;

        // Fonctions par défaut : identité (comportement KF linéaire)
        f_fn = [this](const Matrix& xk) {
            return Matrix::identity(stateDim) * xk;
        };
        h_fn = [this](const Matrix& xk) {
            return Matrix::identity(measDim) * xk;
        };
    }

    // ── Setters ────────────────────────────────────────────────────────────────

    void setState(const Matrix& initialState) {
        if (initialState.getRows() != stateDim || initialState.getCols() != 1)
            throw invalid_argument("Dimension de l'état incorrecte");
        x = initialState;
    }

    void setQ(const Matrix& processNoise) {
        if (processNoise.getRows() != stateDim || processNoise.getCols() != stateDim)
            throw invalid_argument("Dimension de Q incorrecte");
        Q = processNoise;
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

    // Fonction de transition non linéaire  f : R^n → R^n
    void setF(const VecFn& fn) { f_fn = fn; }

    // Fonction d'observation non linéaire  h : R^n → R^m
    void setH(const VecFn& fn) { h_fn = fn; }

    // Jacobienne analytique de f (optionnelle — remplace le calcul numérique)
    void setFjacobian(const function<Matrix(const Matrix&)>& jfn) { Fj_fn = jfn; }

    // Jacobienne analytique de h (optionnelle — remplace le calcul numérique)
    void setHjacobian(const function<Matrix(const Matrix&)>& jfn) { Hj_fn = jfn; }

    // Pas utilisé pour le calcul des jacobiennes numériques (défaut 1e-4)
    void setJacobianEps(float eps) { jacEps = eps; }

    // ── Cœur de l'EKF ─────────────────────────────────────────────────────────

    // Prédiction :
    //   x̂⁻ = f(x̂)
    //   P⁻  = Fj · P · Fjᵀ + Q
    void predict() {
        // Jacobienne de f en x courant
        Matrix Fj = Fj_fn
            ? Fj_fn(x)
            : Matrix::jacobian(f_fn, x, stateDim, jacEps);

        x = f_fn(x);
        P = Fj * P * Fj.transpose() + Q;
    }

    // Correction :
    //   y = z − h(x̂⁻)
    //   S = Hj · P⁻ · Hjᵀ + R
    //   K = P⁻ · Hjᵀ · S⁻¹
    //   x̂ = x̂⁻ + K · y
    //   P = (I − K · Hj) · P⁻
    void update(const Matrix& z) {
        if (z.getRows() != measDim || z.getCols() != 1)
            throw invalid_argument("Dimension de la mesure incorrecte");

        // Jacobienne de h en x̂⁻
        Matrix Hj = Hj_fn
            ? Hj_fn(x)
            : Matrix::jacobian(h_fn, x, measDim, jacEps);

        Matrix y = z - h_fn(x);
        Matrix S = Hj * P * Hj.transpose() + R;
        Matrix K = P * Hj.transpose() * S.robustInverse();

        x = x + K * y;

        // Forme de Joseph (numériquement stable, garantit P symétrique définie positive)
        Matrix I = Matrix::identity(stateDim);
        P = (I - K * Hj) * P * (I - K * Hj).transpose()
            + K * R * K.transpose();
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
        cout << "État estimé (EKF) :\n";
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
        Q.print("Bruit processus Q");
        R.print("Bruit mesure R");
    }
};

#endif // EXTENDED_KALMAN_FILTER_HPP
