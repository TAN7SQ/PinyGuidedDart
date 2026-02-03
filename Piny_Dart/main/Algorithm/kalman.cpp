#include "kalman.hpp"
#include "AuxiliaryMath.hpp"

// reference:https://github.com/ZCM1231/BMI088IMU_kalman/blob/main/App/kalman_filter.c

#include "kalman.hpp"

using namespace AuxMath;

void AttitudeEKF::Init(const Vec3 &initial_accel)
{
    process_noise_q_ = KF_PROCESS_NOISE_Q;
    max_bias_ = KF_MAX_BIAS;
    adaptive_r_threshold_ = KF_ADAPTIVE_R_THRESHOLD;
    adaptive_r_max_scale_ = KF_ADAPTIVE_R_MAX_SCALE;
    adaptive_r_exponent_ = KF_ADAPTIVE_R_EXPONENT;
    r_scale = 1.0f;
    acc_horizontal = 0.0f;
    dt_ = KF_DT_NOMINAL;

    static_gyro_thresh_ = KF_STATIC_GYRO_THRESH;
    static_accel_dev_ = KF_STATIC_ACCEL_DEV;
    gyro_norm = 0.0f;
    accel_norm = 0.0f;
    is_static = true;

    const float ax = initial_accel.x;
    const float ay = initial_accel.y;
    const float az = initial_accel.z;
    const float acc_norm = std::sqrtf(ax * ax + ay * ay + az * az);

    if (acc_norm < 1e-6f) {

        x.quat = Quat(1.0f, 0.0f, 0.0f, 0.0f);
    }
    else {
        const float pitch_check = -ax / acc_norm;
        if (std::fabsf(pitch_check) > 0.99f) {
            // 接近万向节死锁，单位四元数
            x.quat = Quat(1.0f, 0.0f, 0.0f, 0.0f);
        }
        else {
            const float roll = std::atan2f(ay, az);
            const float pitch = std::atan2f(-ax, std::sqrtf(ay * ay + az * az));
            const float yaw = 0.0f;

            const float cr = std::cosf(roll * 0.5f);
            const float sr = std::sinf(roll * 0.5f);
            const float cp = std::cosf(pitch * 0.5f);
            const float sp = std::sinf(pitch * 0.5f);
            const float cy = std::cosf(yaw * 0.5f);
            const float sy = std::sinf(yaw * 0.5f);

            x.quat.w = cr * cp * cy + sr * sp * sy;
            x.quat.x = sr * cp * cy - cr * sp * sy;
            x.quat.y = cr * sp * cy + sr * cp * sy;
            x.quat.z = cr * cp * sy - sr * sp * cy;
        }
    }

    x.bias = Vec3(0.0f, 0.0f, 0.0f);

    Mat7Identity(P);
    for (int i = 0; i < 4; ++i)
        P.m[i][i] = 0.01f;
    for (int i = 4; i < 7; ++i)
        P.m[i][i] = 1.0f;

    Mat7Zero(Q);
    Q.m[4][4] = KF_PROCESS_NOISE_B;
    Q.m[5][5] = KF_PROCESS_NOISE_B;
    Q.m[6][6] = KF_PROCESS_NOISE_B;

    Mat3Identity(R_base);
    Mat3Scale(R_base, R_base, KF_MEASUREMENT_NOISE_A);
    R = R_base;
}

void AttitudeEKF::Predict(const Vec3 &gyro_meas, float dt)
{
    const float q0 = x.quat.w;
    const float q1 = x.quat.x;
    const float q2 = x.quat.y;
    const float q3 = x.quat.z;

    // 陀螺偏置补偿
    const float wx = gyro_meas.x - x.bias.x;
    const float wy = gyro_meas.y - x.bias.y;
    const float wz = gyro_meas.z - x.bias.z;

    float omega[4][4] = {{0.0f, -wx, -wy, -wz}, {wx, 0.0f, wz, -wy}, {wy, -wz, 0.0f, wx}, {wz, wy, -wx, 0.0f}};

    const float q_old[4] = {q0, q1, q2, q3};
    float q_new[4] = {0.0f};
    for (int i = 0; i < 4; ++i) {
        q_new[i] = q_old[i];
        for (int j = 0; j < 4; ++j) {
            q_new[i] += 0.5f * omega[i][j] * dt * q_old[j];
        }
    }

    const float q_norm =
        std::sqrtf(q_new[0] * q_new[0] + q_new[1] * q_new[1] + q_new[2] * q_new[2] + q_new[3] * q_new[3]);
    if (q_norm > 1e-6f) {
        x.quat.w = q_new[0] / q_norm;
        x.quat.x = q_new[1] / q_norm;
        x.quat.y = q_new[2] / q_norm;
        x.quat.z = q_new[3] / q_norm;
    }

    float F[7][7] = {{0.0f}};
    // F_qq = I + 0.5*omega*dt
    for (int i = 0; i < 4; ++i) {
        F[i][i] = 1.0f;
        for (int j = 0; j < 4; ++j) {
            F[i][j] += 0.5f * omega[i][j] * dt;
        }
    }
    // F_qb = -0.5*dt*Xi(q)
    F[0][4] = 0.5f * dt * q1;
    F[0][5] = 0.5f * dt * q2;
    F[0][6] = 0.5f * dt * q3;
    F[1][4] = -0.5f * dt * q0;
    F[1][5] = 0.5f * dt * q3;
    F[1][6] = -0.5f * dt * q2;
    F[2][4] = -0.5f * dt * q3;
    F[2][5] = -0.5f * dt * q0;
    F[2][6] = 0.5f * dt * q1;
    F[3][4] = 0.5f * dt * q2;
    F[3][5] = -0.5f * dt * q1;
    F[3][6] = -0.5f * dt * q0;
    // F_bb = I
    F[4][4] = 1.0f;
    F[5][5] = 1.0f;
    F[6][6] = 1.0f;

    // 构建4x3过程噪声传播矩阵G
    float G[4][3] = {0.0f};
    G[0][0] = -0.5f * dt * q1;
    G[0][1] = -0.5f * dt * q2;
    G[0][2] = -0.5f * dt * q3;
    G[1][0] = 0.5f * dt * q0;
    G[1][1] = -0.5f * dt * q3;
    G[1][2] = 0.5f * dt * q2;
    G[2][0] = 0.5f * dt * q3;
    G[2][1] = 0.5f * dt * q0;
    G[2][2] = -0.5f * dt * q1;
    G[3][0] = -0.5f * dt * q2;
    G[3][1] = 0.5f * dt * q1;
    G[3][2] = -0.5f * dt * q0;

    // 计算Q_q = G * Q_gyro * G^T，更新Q的四元数部分
    const float Q_gyro_scalar = process_noise_q_;
    float Q_q[4][4] = {{0.0f}};
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 3; ++k) {
                Q_q[i][j] += G[i][k] * Q_gyro_scalar * G[j][k];
            }
        }
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Q.m[i][j] = Q_q[i][j];
        }
    }

    // 协方差预测：P = F*P*F^T + Q
    const Mat7x7 P_old = P;
    Mat7x7 FP;
    // FP = F * P_old
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                FP.m[i][j] += F[i][k] * P_old.m[k][j];
            }
        }
    }
    // FPFT = FP * F^T
    Mat7x7 FPFT;
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                FPFT.m[i][j] += FP.m[i][k] * F[j][k];
            }
        }
    }
    // P = FPFT + Q
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            P.m[i][j] = FPFT.m[i][j] + Q.m[i][j];
        }
    }

    x.bias.z = 0.0f;
}

void AttitudeEKF::Update(const Vec3 &acc_meas)
{
    // 归一化加速度计测量值
    Vec3 z = acc_meas;
    Vec3Normalize(z);

    // 提取当前四元数
    const float qw = x.quat.w;
    const float qx = x.quat.x;
    const float qy = x.quat.y;
    const float qz = x.quat.z;

    // 根据静止状态和水平加速度自适应观测噪声R调整
    const float acc_nav_x =
        (1 - 2 * (qy * qy + qz * qz)) * z.x + 2 * (qx * qy - qw * qz) * z.y + 2 * (qx * qz + qw * qy) * z.z;
    const float acc_nav_y =
        2 * (qx * qy + qw * qz) * z.x + (1 - 2 * (qx * qx + qz * qz)) * z.y + 2 * (qy * qz - qw * qx) * z.z;
    acc_horizontal = std::sqrtf(acc_nav_x * acc_nav_x + acc_nav_y * acc_nav_y);
    if (is_static) {
        r_scale = 1.0f;
        R = R_base;
    }
    else if (acc_horizontal > adaptive_r_threshold_) {
        const float ratio = acc_horizontal / adaptive_r_threshold_;
        r_scale = std::powf(ratio, adaptive_r_exponent_);
        if (r_scale > adaptive_r_max_scale_) {
            r_scale = adaptive_r_max_scale_;
        }
        Mat3Scale(R, R_base, r_scale);
    }
    else {
        r_scale = 2.0f;
        Mat3Scale(R, R_base, r_scale);
    }

    // 预测观测值（导航系重力方向）
    float z_pred[3] = {0.0f};
    z_pred[0] = 2.0f * (qx * qz - qw * qy);
    z_pred[1] = 2.0f * (qy * qz + qw * qx);
    z_pred[2] = qw * qw - qx * qx - qy * qy + qz * qz;

    // 观测雅可比矩阵H
    float H_q[3][4] = {{-2.0f * qy, 2.0f * qz, -2.0f * qw, 2.0f * qx},
                       {2.0f * qx, 2.0f * qw, 2.0f * qz, 2.0f * qy},
                       {2.0f * qw, -2.0f * qx, -2.0f * qy, 2.0f * qz}};
    float H[3][7] = {{0.0f}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j)
            H[i][j] = H_q[i][j];
        for (int j = 4; j < 7; ++j)
            H[i][j] = 0.0f;
    }

    // S = H*P*H^T + R（3x3）
    float HP[3][7] = {{0.0f}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                HP[i][j] += H[i][k] * P.m[k][j];
            }
        }
    }
    float S[3][3] = {{0.0f}};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 7; ++k) {
                S[i][j] += HP[i][k] * H[j][k];
            }
            if (i == j)
                S[i][j] += R.m[i][j];
        }
    }

    float S_inv[3][3] = {{0.0f}};
    const float det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) -
                      S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                      S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
    if (std::fabsf(det) < 1e-9f)
        return; // 防止除零
    const float inv_det = 1.0f / det;

    S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * inv_det;
    S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * inv_det;
    S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * inv_det;
    S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * inv_det;
    S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * inv_det;
    S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * inv_det;
    S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * inv_det;
    S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * inv_det;
    S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * inv_det;

    // K = P*H^T*S_inv（7x3）
    float PHT[7][3] = {{0.0f}};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 7; ++k) {
                PHT[i][j] += P.m[i][k] * H[j][k];
            }
        }
    }
    float K[7][3] = {{0.0f}};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                K[i][j] += PHT[i][k] * S_inv[k][j];
            }
        }
    }
    std::memcpy(debug_K, K, sizeof(debug_K));

    // 计算新息y = z - z_pred
    y[0] = z.x - z_pred[0];
    y[1] = z.y - z_pred[1];
    y[2] = z.z - z_pred[2];

    // x = x + K*y
    float Ky[7] = {0.0f};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 3; ++j) {
            Ky[i] += K[i][j] * y[j];
        }
    }

    // correction
    x.quat.w += Ky[0];
    x.quat.x += Ky[1];
    x.quat.y += Ky[2];
    x.quat.z += Ky[3];
    x.bias.x += Ky[4];
    x.bias.y += Ky[5];
    x.bias.z += Ky[6];

    QuatNormalize(x.quat);
    x.bias.x = std::fmaxf(-max_bias_, std::fminf(max_bias_, x.bias.x));
    x.bias.y = std::fmaxf(-max_bias_, std::fminf(max_bias_, x.bias.y));
    x.bias.z = 0.0f; // 强制Z轴偏置为0

    // 协方差更新
    float I_KH[7][7] = {{0.0f}};
    for (int i = 0; i < 7; ++i) {
        I_KH[i][i] = 1.0f;
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 3; ++k) {
                I_KH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }

    // P1 = (I-KH) * P
    float P1[7][7] = {{0.0f}};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                P1[i][j] += I_KH[i][k] * P.m[k][j];
            }
        }
    }

    // P2 = P1 * (I-KH)^T
    float P2[7][7] = {{0.0f}};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 7; ++k) {
                P2[i][j] += P1[i][k] * I_KH[j][k];
            }
        }
    }

    // KR = K * R
    float KR[7][3] = {{0.0f}};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                KR[i][j] += K[i][k] * R.m[k][j];
            }
        }
    }

    // KRKT = KR * K^T
    float KRKT[7][7] = {{0.0f}};
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            for (int k = 0; k < 3; ++k) {
                KRKT[i][j] += KR[i][k] * K[j][k];
            }
        }
    }

    // P = P2 + KRKT
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            P.m[i][j] = P2[i][j] + KRKT[i][j];
        }
    }
}

void AttitudeEKF::QuatToEuler(const Quat &q, float &roll, float &pitch, float &yaw) const
{
    const float qw = q.w, qx = q.x, qy = q.y, qz = q.z;

    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    roll = std::atan2f(sinr_cosp, cosr_cosp);

    const float sinp = 2.0f * (qw * qy - qz * qx);
    if (std::fabsf(sinp) >= 1.0f) {
        pitch = std::copysignf(M_PI / 2.0f, sinp);
    }
    else {
        pitch = std::asinf(sinp);
    }

    const float siny_cosp = 2.0f * (qw * qz + qx * qy);
    const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = std::atan2f(siny_cosp, cosy_cosp);

    roll *= 180.0f / M_PI;
    pitch *= 180.0f / M_PI;
    yaw *= 180.0f / M_PI;
}

void AttitudeEKF::CalculateAccelOnlyEuler(const Vec3 &acc_meas)
{
    Vec3 z = acc_meas;
    Vec3Normalize(z);
    accel_only_euler[0] = std::atan2f(z.y, z.z) * 180.0f / M_PI;
    accel_only_euler[1] = std::atan2f(-z.x, std::sqrtf(z.y * z.y + z.z * z.z)) * 180.0f / M_PI;
    accel_only_euler[2] = 0.0f;
}

// still detect static
void AttitudeEKF::StaticDetect(const Vec3 &gyro, const Vec3 &acc)
{
    gyro_norm = std::sqrtf(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);
    accel_norm = std::sqrtf(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    is_static = (gyro_norm < static_gyro_thresh_) && (std::fabsf(accel_norm - g_) < static_accel_dev_);
}