// Minimal LQR lane keeping demo without external libraries
// Simulates lateral error ey and heading error epsi with a simple kinematic model.
// Outputs CSV log: time, ey, epsi, delta.

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

struct LQR {
    // 2x2 matrices and vectors for a simple discrete-time system
    // x_{k+1} = A x_k + B u_k
    double A11, A12, A21, A22; // A
    double B1, B2;             // B (column vector)
    double Q11, Q12, Q21, Q22; // Q (symmetric)
    double R;                  // scalar R

    // Riccati matrix P
    double P11, P12, P21, P22;

    // Feedback gain K (row vector)
    double K1, K2;

    static double det2(double a11, double a12, double a21, double a22) {
        return a11 * a22 + (-a12) * a21;
    }

    // Invert symmetric 1x1 + scalar, but here we only need scalar inverse for S = R + B^T P B
    static double inv_scalar(double s) { return 1.0 / s; }

    void solveRiccatiIterative(int iters = 500) {
        // Initialize P as Q
        P11 = Q11; P12 = Q12; P21 = Q21; P22 = Q22;

        for (int k = 0; k < iters; ++k) {
            // Compute S = R + B^T P B
            double PB1 = P11 * B1 + P12 * B2; // (P * B)_1
            double PB2 = P21 * B1 + P22 * B2; // (P * B)_2
            double BT_P_B = B1 * PB1 + B2 * PB2;
            double S = R + BT_P_B;
            double Sinv = inv_scalar(S);

            // Compute A^T P A
            double PA11 = P11 * A11 + P12 * A21;
            double PA12 = P11 * A12 + P12 * A22;
            double PA21 = P21 * A11 + P22 * A21;
            double PA22 = P21 * A12 + P22 * A22;

            double AT_PA11 = A11 * PA11 + A21 * PA21;
            double AT_PA12 = A11 * PA12 + A21 * PA22;
            double AT_PA21 = A12 * PA11 + A22 * PA21;
            double AT_PA22 = A12 * PA12 + A22 * PA22;

            // Compute A^T P B
            double AT_PB1 = A11 * PB1 + A21 * PB2;
            double AT_PB2 = A12 * PB1 + A22 * PB2;

            // Update P = Q + A^T P A - A^T P B * S^{-1} * B^T P A
            // The last term is rank-1: (A^T P B) * Sinv * (B^T P A)
            double term11 = AT_PB1 * Sinv * AT_PB1;
            double term12 = AT_PB1 * Sinv * AT_PB2;
            double term21 = AT_PB2 * Sinv * AT_PB1;
            double term22 = AT_PB2 * Sinv * AT_PB2;

            P11 = Q11 + (AT_PA11 - term11);
            P12 = Q12 + (AT_PA12 - term12);
            P21 = Q21 + (AT_PA21 - term21);
            P22 = Q22 + (AT_PA22 - term22);
        }

        // K = S^{-1} B^T P A
        double PB1 = P11 * B1 + P12 * B2;
        double PB2 = P21 * B1 + P22 * B2;
        double BT_P_B = B1 * PB1 + B2 * PB2;
        double S = R + BT_P_B;
        double Sinv = inv_scalar(S);

        // B^T P A: row vector length 2
        double PA11 = P11 * A11 + P12 * A21;
        double PA12 = P11 * A12 + P12 * A22;
        double PA21 = P21 * A11 + P22 * A21;
        double PA22 = P21 * A12 + P22 * A22;
        double BT_PA1 = B1 * PA11 + B2 * PA21;
        double BT_PA2 = B1 * PA12 + B2 * PA22;

        K1 = Sinv * BT_PA1;
        K2 = Sinv * BT_PA2;
    }
};

int main() {
    // Vehicle and simulation parameters
    const double v = 10.0;     // speed [m/s]
    const double L = 2.5;      // wheelbase [m]
    const double dt = 0.05;    // sampling time [s]
    const int steps = 600;     // simulation steps (30s)

    // Discrete-time linearized lateral dynamics around straight motion
    // ey_{k+1}   = ey_k + dt * v * epsi_k
    // epsi_{k+1} = epsi_k + dt * v / L * delta_k
    LQR ctrl;
    ctrl.A11 = 1.0; ctrl.A12 = dt * v;
    ctrl.A21 = 0.0; ctrl.A22 = 1.0;
    ctrl.B1 = 0.0; ctrl.B2 = dt * v / L;

    // Cost weights: tune these to balance tracking vs. control effort
    ctrl.Q11 = 3.0; ctrl.Q12 = 0.0;
    ctrl.Q21 = 0.0; ctrl.Q22 = 1.5;
    ctrl.R = 0.5; // steering effort weight

    ctrl.solveRiccatiIterative(800);

    // Initial state: lateral offset and heading error
    double ey = 1.0;    // meters
    double epsi = 0.20; // radians
    double delta = 0.0; // steering command [rad]

    // Saturation for steering
    const double delta_min = -0.6;
    const double delta_max = 0.6;

    std::ofstream ofs("lqr_log.csv");
    ofs << "t,ey,epsi,delta\n";

    double t = 0.0;
    for (int k = 0; k < steps; ++k) {
        // State vector x = [ey, epsi]^T; control u = -K x
        delta = -(ctrl.K1 * ey + ctrl.K2 * epsi);
        if (delta < delta_min) delta = delta_min;
        if (delta > delta_max) delta = delta_max;

        // Log before update
        ofs << std::fixed << std::setprecision(6)
            << t << "," << ey << "," << epsi << "," << delta << "\n";

        // System update
        double ey_next = ctrl.A11 * ey + ctrl.A12 * epsi + ctrl.B1 * delta;
        double epsi_next = ctrl.A21 * ey + ctrl.A22 * epsi + ctrl.B2 * delta;

        ey = ey_next;
        epsi = epsi_next;
        t += dt;
    }
    ofs.close();

    std::cout << "LQR lane keeping simulation finished.\n";
    std::cout << "CSV saved to lqr_log.csv (t, ey, epsi, delta).\n";
    std::cout << "Final ey=" << std::fixed << std::setprecision(4) << ey
              << ", epsi=" << epsi << "\n";
    std::cout << "Gain K = [" << ctrl.K1 << ", " << ctrl.K2 << "]\n";

    return 0;
}