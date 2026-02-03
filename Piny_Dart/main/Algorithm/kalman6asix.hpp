#pragma once

#ifndef KALMAN_FILTER_H
    #define KALMAN_FILTER_H

    #include <cmath>
    #include <cstring>

    #include "AuxiliaryMath.hpp"

    #define KF_PROCESS_NOISE_Q 0.001f     // 陀螺仪过程噪声
    #define KF_PROCESS_NOISE_B 0.0001f    // 陀螺偏置过程噪声
    #define KF_MEASUREMENT_NOISE_A 0.1f   // 加速度计基础观测噪声
    #define KF_MAX_BIAS 0.5f              // 陀螺偏置最大限幅 (rad/s)
    #define KF_ADAPTIVE_R_THRESHOLD 0.1f  // 自适应R阈值
    #define KF_ADAPTIVE_R_MAX_SCALE 20.0f // 自适应R最大缩放倍数
    #define KF_ADAPTIVE_R_EXPONENT 2.0f   // 自适应R指数
    #define KF_DT_NOMINAL 0.005f          // 标称采样时间 (5ms)
    #define KF_STATIC_GYRO_THRESH 0.05f   // 静止检测陀螺阈值 (rad/s)
    #define KF_STATIC_ACCEL_DEV 0.5f      // 静止检测加速度偏差 (m/s²)

// 命名空间：避免全局命名冲突

// 姿态估计EKF类：封装所有状态、参数、方法
class AttitudeEKF
{
public:
    AttitudeEKF() = default;
    ~AttitudeEKF() = default;

    void Init(const AuxMath::Vec3 &initial_accel);

    void Predict(const AuxMath::Vec3 &gyro_meas, float dt);
    void Update(const AuxMath::Vec3 &acc_meas);
    void StaticDetect(const AuxMath::Vec3 &gyro, const AuxMath::Vec3 &acc);
    void CalculateAccelOnlyEuler(const AuxMath::Vec3 &acc_meas);
    void QuatToEuler(const AuxMath::Quat &q, float &roll, float &pitch, float &yaw) const;

    AuxMath::EKFState x;    // 核心状态：姿态+偏置
    AuxMath::Mat7x7 P;      // 协方差矩阵
    AuxMath::Mat7x7 Q;      // 过程噪声矩阵
    AuxMath::Mat3x3 R_base; // 基础观测噪声矩阵
    AuxMath::Mat3x3 R;      // 自适应观测噪声矩阵

    float debug_K[7][3] = {{0.0f}};     // 卡尔曼增益
    float y[3] = {0.0f};                // 新息（观测-预测）
    float accel_only_euler[3] = {0.0f}; // 纯加速度计欧拉角（调试，度）
    float acc_horizontal = 0.0f;        // 水平加速度（自适应R用）
    float r_scale = 1.0f;               // 观测噪声缩放因子

    float gyro_norm = 0.0f;  // 陀螺仪模值（静止检测用）
    float accel_norm = 0.0f; // 加速度计模值（静止检测用）
    bool is_static = true;

private:
    float process_noise_q_ = 0.0f;      // 陀螺仪过程噪声
    float g_ = 9.81f;                   // 重力加速度
    float max_bias_ = 0.0f;             // 陀螺偏置最大限幅
    float adaptive_r_threshold_ = 0.0f; // 自适应R阈值
    float adaptive_r_max_scale_ = 0.0f; // 自适应R最大缩放倍数
    float adaptive_r_exponent_ = 0.0f;  // 自适应R指数
    float dt_ = 0.0f;                   // 采样时间步长
    float static_gyro_thresh_ = 0.0f;   // 静止检测陀螺阈值
    float static_accel_dev_ = 0.0f;     // 静止检测加速度偏差
};

#endif // KALMAN_FILTER_H