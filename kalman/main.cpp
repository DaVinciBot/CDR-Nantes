#include "KalmanFilter.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

using namespace std;

// État : [x, y, z, vx, vy, vz, theta, phi, psi]
// Mesure: [x_opt, y_opt, ax, ay, az, gx, gy, gz, mx, my, mz, lidar_dist, lidar_angle, servo_pos]

Matrix readSensors() {
    Matrix z(14, 1);

    // Capteur optique (PAA5100JE) — position en mètres
    z(0, 0) = 1.0f;
    z(1, 0) = 0.5f;

    // Accéléromètre (BNO080) — m/s²
    z(2, 0) = 0.1f;
    z(3, 0) = 0.0f;
    z(4, 0) = 9.81f;

    // Gyroscope (BNO080) — rad/s
    z(5, 0) = 0.01f;
    z(6, 0) = 0.0f;
    z(7, 0) = 0.0f;

    // Magnétomètre (BNO080) — Tesla
    z(8, 0)  = 0.3e-6f;
    z(9, 0)  = 0.2e-6f;
    z(10, 0) = 0.5e-6f;

    // LIDAR (RPLIDAR A2M12) — distance (m), angle (rad)
    z(11, 0) = 2.5f;
    z(12, 0) = 0.1f;

    // Servomoteur (MKS-SERVO57B) — rad
    z(13, 0) = 0.0f;

    return z;
}

void setupKalmanFilter(KalmanFilter& kf, float dt) {

    // ── F : modèle cinématique à vitesse constante ────────────────────────────
    Matrix F(9, 9);
    // x += vx*dt,  y += vy*dt,  z += vz*dt
    F(0, 0) = 1.0f;  F(0, 3) = dt;
    F(1, 1) = 1.0f;  F(1, 4) = dt;
    F(2, 2) = 1.0f;  F(2, 5) = dt;
    F(3, 3) = 1.0f;
    F(4, 4) = 1.0f;
    F(5, 5) = 1.0f;
    F(6, 6) = 1.0f;
    F(7, 7) = 1.0f;
    F(8, 8) = 1.0f;
    kf.setF(F);

    // ── Q : bruit d'accélération aléatoire ───────────────────────────────────
    float sigma_a = 0.5f;   // m/s²  — À ajuster
    float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt;

    Matrix Q(9, 9);
    // Blocs position-vitesse X, Y, Z
    for (int i = 0; i < 3; i++) {
        Q(i,   i)   = dt4 / 4 * sigma_a * sigma_a;
        Q(i,   i+3) = dt3 / 2 * sigma_a * sigma_a;
        Q(i+3, i)   = dt3 / 2 * sigma_a * sigma_a;
        Q(i+3, i+3) = dt2     * sigma_a * sigma_a;
    }
    float sigma_angle = 0.01f;  // rad — À ajuster
    Q(6, 6) = sigma_angle * sigma_angle;
    Q(7, 7) = sigma_angle * sigma_angle;
    Q(8, 8) = sigma_angle * sigma_angle;
    kf.setQ(Q);

    // ── H : relation capteurs → état ─────────────────────────────────────────
    Matrix H(14, 9);
    H(0, 0)  = 1.0f;  // x_opt  → x
    H(1, 1)  = 1.0f;  // y_opt  → y
    H(2, 3)  = 1.0f;  // ax     → vx  (approximation)
    H(3, 4)  = 1.0f;  // ay     → vy
    H(4, 5)  = 1.0f;  // az     → vz
    // gyroscope (z[5..7]) : non linéaire, ignoré pour l'instant
    H(8, 8)  = 1.0f;  // mx     → psi (simplifié)
    H(11, 0) = 1.0f;  // lidar  → x   (TODO: EKF pour la non-linéarité)
    H(13, 8) = 1.0f;  // servo  → psi
    kf.setH(H);

    // ── R : variances capteurs (datasheets) ───────────────────────────────────
    Matrix R(14, 14);
    float v = 1.0f;  // vitesse typique (m/s) pour le calcul bruit optique
    R(0, 0)   = (0.028f * v) * (0.028f * v);  // PAA5100JE : 2.8% à 45 ips
    R(1, 1)   = (0.028f * v) * (0.028f * v);
    R(2, 2)   = 9e-4f;    // BNO080 accel  ~0.03 m/s²
    R(3, 3)   = 9e-4f;
    R(4, 4)   = 9e-4f;
    R(5, 5)   = 1e-4f;    // BNO080 gyro   ~0.01 rad/s
    R(6, 6)   = 1e-4f;
    R(7, 7)   = 1e-4f;
    R(8, 8)   = 9e-14f;   // BNO080 mag    ~0.3 µT
    R(9, 9)   = 9e-14f;
    R(10, 10) = 9e-14f;
    R(11, 11) = 4e-4f;    // RPLIDAR A2    ~20 mm
    R(12, 12) = 1.52e-5f; // RPLIDAR A2    ~0.225°
    R(13, 13) = 1e-6f;    // MKS-SERVO57B  ~0.001 rad
    kf.setR(R);

    // ── P : incertitude initiale ──────────────────────────────────────────────
    Matrix P(9, 9);
    P(0, 0) = 100.0f;  P(1, 1) = 100.0f;  P(2, 2) = 100.0f;  // positions
    P(3, 3) = 10.0f;   P(4, 4) = 10.0f;   P(5, 5) = 10.0f;   // vitesses
    P(6, 6) = 1.0f;    P(7, 7) = 1.0f;    P(8, 8) = 1.0f;    // angles
    kf.setP(P);
}

int main() {
    const size_t STATE_DIM = 9;
    const size_t MEAS_DIM  = 14;
    const float  dt        = 0.1f;  // 10 Hz

    KalmanFilter kf(STATE_DIM, MEAS_DIM, dt);
    setupKalmanFilter(kf, dt);

    for (int iter = 0; iter < 10; iter++) {
        cout << "┌─ Itération " << iter + 1 << " " << string(45, '─') << "┐\n";

        auto t0 = chrono::high_resolution_clock::now();
        kf.predict();
        auto t1 = chrono::high_resolution_clock::now();

        cout << "│ Après PRÉDICTION :\n";
        cout << fixed << setprecision(3);
        cout << "│   Position : (" << kf.getStateElement(0) << ", "
                                   << kf.getStateElement(1) << ", "
                                   << kf.getStateElement(2) << ") m\n";
        cout << "│   Vitesse  : (" << kf.getStateElement(3) << ", "
                                   << kf.getStateElement(4) << ", "
                                   << kf.getStateElement(5) << ") m/s\n";

        Matrix z = readSensors();

        auto t2 = chrono::high_resolution_clock::now();
        kf.update(z);
        auto t3 = chrono::high_resolution_clock::now();

        cout << "│\n│ Après MISE À JOUR :\n";
        cout << "│   Position : (" << kf.getStateElement(0) << ", "
                                   << kf.getStateElement(1) << ", "
                                   << kf.getStateElement(2) << ") m\n";
        cout << "│   Vitesse  : (" << kf.getStateElement(3) << ", "
                                   << kf.getStateElement(4) << ", "
                                   << kf.getStateElement(5) << ") m/s\n";
        cout << "│   Angles   : (" << kf.getStateElement(6) << ", "
                                   << kf.getStateElement(7) << ", "
                                   << kf.getStateElement(8) << ") rad\n";

        cout << "│\n│ Incertitudes σ : x=" << kf.getUncertainty(0)
             << "  y=" << kf.getUncertainty(1)
             << "  z=" << kf.getUncertainty(2) << " m\n";

        auto predict_us = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
        auto update_us  = chrono::duration_cast<chrono::microseconds>(t3 - t2).count();
        cout << "│\n│ Temps : predict=" << predict_us
             << " µs  update=" << update_us
             << " µs  total=" << (predict_us + update_us) << " µs\n";

        cout << "└" << string(59, '─') << "┘\n\n";

        this_thread::sleep_for(chrono::milliseconds(static_cast<int>(dt * 1000)));
    }

    cout << "\nÉtat final :\n";
    kf.printState();

    cout << "\nAfficher les matrices de debug ? (y/n) : ";
    char choice;
    cin >> choice;
    if (choice == 'y' || choice == 'Y')
        kf.printDebug();

    return 0;
}
