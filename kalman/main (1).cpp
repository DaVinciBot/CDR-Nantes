#include "ExtendedKalmanFilter.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
using namespace std;

// ============================================================
//  STATE (6)
//    [0] x       position  (m)
//    [1] y       position  (m)
//    [2] theta   orientation (rad)
//    [3] vx      vitesse X  (m/s)
//    [4] vy      vitesse Y  (m/s)
//    [5] omega   vitesse angulaire (rad/s)
//
//  MEAS (5)
//    [0] x_teensy
//    [1] y_teensy
//    [2] theta_teensy
//    [3] lidar_distance
//    [4] lidar_angle
// ============================================================

// ── Transition d'état (robot holonome) ────────────────────────────────────────
Matrix stateTransition(const Matrix& x, float dt) {
    Matrix xn(6, 1);
    float px = x(0, 0), py = x(1, 0), th = x(2, 0);
    float vx = x(3, 0), vy = x(4, 0), w  = x(5, 0);

    xn(0, 0) = px + vx * dt;
    xn(1, 0) = py + vy * dt;
    xn(2, 0) = atan2f(sinf(th + w * dt), cosf(th + w * dt)); // normalise en [-pi,pi]
    xn(3, 0) = vx;
    xn(4, 0) = vy;
    xn(5, 0) = w;
    return xn;
}

// ── Modèle d'observation (non linéaire pour le LIDAR) ────────────────────────
Matrix observationModel(const Matrix& x) {
    Matrix z(5, 1);
    float px = x(0, 0), py = x(1, 0), th = x(2, 0);

    // Odométrie Teensy (linéaire)
    z(0, 0) = px;
    z(1, 0) = py;
    z(2, 0) = th;

    // LIDAR (non linéaire : coordonnées polaires)
    float r = sqrtf(px * px + py * py);
    if (r < 1e-6f) r = 1e-6f;
    z(3, 0) = r;
    z(4, 0) = atan2f(py, px);
    return z;
}

// ── Capteurs simulés (remplacer par USB + LIDAR réels) ───────────────────────
Matrix readSensors() {
    Matrix z(5, 1);
    z(0, 0) = 1.0f;
    z(1, 0) = 0.5f;
    z(2, 0) = 0.1f;
    z(3, 0) = sqrtf(1.0f * 1.0f + 0.5f * 0.5f);
    z(4, 0) = atan2f(0.5f, 1.0f);
    return z;
}

// ── Configuration EKF ────────────────────────────────────────────────────────
void setupEKF(ExtendedKalmanFilter& ekf, float dt) {

    // Fonction de transition non linéaire
    ekf.setF([dt](const Matrix& x) {
        return stateTransition(x, dt);
    });

    // Fonction d'observation non linéaire
    ekf.setH([](const Matrix& x) {
        return observationModel(x);
    });

    // Jacobienne analytique de f (approx. linéaire locale)
    ekf.setFjacobian([dt](const Matrix&) {
        Matrix Fj = Matrix::identity(6);
        Fj(0, 3) = dt;
        Fj(1, 4) = dt;
        Fj(2, 5) = dt;  // d(theta)/d(omega) = dt
        return Fj;
    });

    // Q — bruit processus
    Matrix Q = Matrix::identity(6) * 0.01f;
    Q(0, 0) = 0.001f;
    Q(1, 1) = 0.001f;
    Q(2, 2) = 0.0005f;
    ekf.setQ(Q);

    // R — bruit mesure (datasheets)
    Matrix R(5, 5);
    R(0, 0) = 1e-4f;   // Teensy x
    R(1, 1) = 1e-4f;   // Teensy y
    R(2, 2) = 1e-4f;   // Teensy theta
    R(3, 3) = 4e-4f;   // LIDAR distance (~20 mm)
    R(4, 4) = 1.5e-5f; // LIDAR angle   (~0.225 deg)
    ekf.setR(R);

    // P — covariance initiale
    Matrix P = Matrix::identity(6) * 1.0f;
    ekf.setP(P);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    const size_t STATE_DIM = 6;
    const size_t MEAS_DIM  = 5;
    const float  dt        = 0.1f;   // 10 Hz

    ExtendedKalmanFilter ekf(STATE_DIM, MEAS_DIM, dt);
    setupEKF(ekf, dt);

    cout << fixed << setprecision(4);

    for (int i = 0; i < 10; i++) {
        cout << "--- Iteration " << i + 1 << " ---\n";

        auto t0 = chrono::high_resolution_clock::now();
        ekf.predict();
        auto t1 = chrono::high_resolution_clock::now();

        Matrix z = readSensors();

        auto t2 = chrono::high_resolution_clock::now();
        ekf.update(z);
        auto t3 = chrono::high_resolution_clock::now();

        cout << "  x="     << ekf.getStateElement(0)
             << "  y="     << ekf.getStateElement(1)
             << "  theta=" << ekf.getStateElement(2) << " rad\n";
        cout << "  vx="    << ekf.getStateElement(3)
             << "  vy="    << ekf.getStateElement(4)
             << "  omega=" << ekf.getStateElement(5) << " rad/s\n";
        cout << "  sigma: x=" << ekf.getUncertainty(0)
             << "  y="        << ekf.getUncertainty(1)
             << "  theta="    << ekf.getUncertainty(2) << "\n";

        auto predict_us = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
        auto update_us  = chrono::duration_cast<chrono::microseconds>(t3 - t2).count();
        cout << "  timing: predict=" << predict_us
             << " us  update=" << update_us
             << " us  total=" << (predict_us + update_us) << " us\n\n";

        this_thread::sleep_for(chrono::milliseconds(100));
    }

    cout << "\nEtat final :\n";
    ekf.printState();
    return 0;
}
