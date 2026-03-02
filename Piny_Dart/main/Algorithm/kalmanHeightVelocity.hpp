#pragma once
#include <cmath>

class kalmanHeight
{
public:
    void init(double initial_height)
    {
        x[0] = initial_height; // height
        x[1] = 0.0;            // velocity

        P[0][0] = 1;
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 1;
    }

    void update(double measured_height, double dt)
    {
        double x_pred[2];
        x_pred[0] = x[0] + dt * x[1];
        x_pred[1] = x[1];

        // P = F P F^T + Q
        double P_pred[2][2];

        P_pred[0][0] = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q_h;
        P_pred[0][1] = P[0][1] + dt * P[1][1];
        P_pred[1][0] = P[1][0] + dt * P[1][1];
        P_pred[1][1] = P[1][1] + Q_v;

        double y = measured_height - x_pred[0];

        double S = P_pred[0][0] + R;

        double K0 = P_pred[0][0] / S;
        double K1 = P_pred[1][0] / S;

        x[0] = x_pred[0] + K0 * y;
        x[1] = x_pred[1] + K1 * y;

        P[0][0] = (1 - K0) * P_pred[0][0];
        P[0][1] = (1 - K0) * P_pred[0][1];
        P[1][0] = P_pred[1][0] - K1 * P_pred[0][0];
        P[1][1] = P_pred[1][1] - K1 * P_pred[0][1];
    }

    double getHeight() const
    {
        return x[0];
    }
    double getVelocity() const
    {
        return x[1];
    }

private:
    double x[2];    // 状态
    double P[2][2]; // 协方差

    const double Q_h = 0.01;
    const double Q_v = 0.001;
    const double R = 0.04;
};