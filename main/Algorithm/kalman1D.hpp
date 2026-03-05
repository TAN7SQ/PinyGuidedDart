#pragma once

class Kalman1D
{
public:
    void init(double initial_height)
    {
        x = initial_height;
        P = 1.0;
    }

    void update(double measured_height)
    {
        P += Q;

        double K = P / (P + R);
        x = x + K * (measured_height - x);
        P = (1 - K) * P;
    }

    double getHeight() const
    {
        return x;
    }

private:
    double x; // 状态：高度
    double P; // 协方差

    // ===== 可调参数 =====
    const double Q = 0.0005; // 过程噪声（越小越稳）
    const double R = 0.05;   // 测量噪声（根据你实际噪声调）
};