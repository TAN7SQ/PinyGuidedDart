#include "complementary6asix.hpp"
#include <cmath>

// 兼容ESP32：去掉std::，用单精度浮点函数（f后缀）
#define M_PI_F 3.14159265358979323846f // 单精度PI

namespace SixAxisIMU
{

// 构造函数：初始化滤波系数和姿态
ComplementaryFilter::ComplementaryFilter(float filter_alpha)
{
    _alpha = (filter_alpha > 0.0f && filter_alpha < 1.0f) ? filter_alpha : 0.98f;
    _gyro_bias = AuxMath::Vec3(0.0f, 0.0f, 0.0f);           // 初始零漂为0
    _attitude.quat = AuxMath::Quat(1.0f, 0.0f, 0.0f, 0.0f); // 初始四元数（单位四元数，无旋转）
    _attitude.euler = AuxMath::Vec3(0.0f, 0.0f, 0.0f);      // 初始欧拉角为0
}

// 从加速度计解算横滚/俯仰角（静态有效）
AuxMath::Vec3 ComplementaryFilter::accToEuler(const AuxMath::Vec3 &acc)
{
    AuxMath::Vec3 euler;
    // 归一化加速度计数据（消除重力加速度幅值影响，兼容不同量程）
    AuxMath::Vec3 acc_norm = acc;
    AuxMath::Vec3Normalize(acc_norm);

    // 解算横滚角（Roll）：x轴旋转，范围[-π, π]
    euler.x = atan2f(acc_norm.y, acc_norm.z);
    // 解算俯仰角（Pitch）：y轴旋转，范围[-π/2, π/2]，防止万向节锁死
    euler.y = asinf(-acc_norm.x);
    // 六轴无偏航角参考，yaw=0
    euler.z = 0.0f;

    return euler;
}

// 陀螺仪积分更新四元数（四元数微分方程）
AuxMath::Quat ComplementaryFilter::gyroUpdateQuat(const AuxMath::Quat &quat, const AuxMath::Vec3 &gyro, float dt)
{
    AuxMath::Quat q = quat;
    // 四元数微分方程：dQ/dt = 0.5 * Q ⊗ ω（ω为陀螺仪角速度向量，转四元数形式）
    AuxMath::Quat omega(0.0f, gyro.x * 0.5f, gyro.y * 0.5f, gyro.z * 0.5f);
    // 四元数乘法：Q ⊗ ω
    AuxMath::Quat q_omega;
    q_omega.w = q.w * omega.w - q.x * omega.x - q.y * omega.y - q.z * omega.z;
    q_omega.x = q.w * omega.x + q.x * omega.w + q.y * omega.z - q.z * omega.y;
    q_omega.y = q.w * omega.y - q.x * omega.z + q.y * omega.w + q.z * omega.x;
    q_omega.z = q.w * omega.z + q.x * omega.y - q.y * omega.x + q.z * omega.w;
    // 前向欧拉积分更新四元数
    q.w += q_omega.w * dt;
    q.x += q_omega.x * dt;
    q.y += q_omega.y * dt;
    q.z += q_omega.z * dt;
    // 归一化四元数，保证单位四元数
    AuxMath::QuatNormalize(q);

    return q;
}

// 初始化姿态：静止时从加速度计解算初始姿态，并校准陀螺仪零漂
AuxMath::Quat ComplementaryFilter::initAttitude(const AuxMath::Vec3 &acc)
{
    // 从加速度计解算初始欧拉角
    AuxMath::Vec3 init_euler = accToEuler(acc);
    // 欧拉角转四元数，作为初始姿态
    AuxMath::EulerToQuat(init_euler, _attitude.quat);
    // 初始姿态同步到欧拉角
    AuxMath::QuatToEuler(_attitude.quat, _attitude.euler);
    // 初始化陀螺仪零漂（开机静止，陀螺仪读数即为零漂）
    _gyro_bias = AuxMath::Vec3(0.0f, 0.0f, 0.0f);

    return _attitude.quat;
}

// 核心解算函数：互补滤波融合陀螺仪+加速度计
IMUAttitude ComplementaryFilter::update(const IMURawData &imu_data, float dt)
{
    // 1. 陀螺仪数据预处理：去零漂（简单校准，减少漂移）
    AuxMath::Vec3 gyro_calib;
    gyro_calib.x = imu_data.gyro.x - _gyro_bias.x;
    gyro_calib.y = imu_data.gyro.y - _gyro_bias.y;
    gyro_calib.z = imu_data.gyro.z - _gyro_bias.z;

    // 2. 陀螺仪积分更新四元数（高频更新）
    AuxMath::Quat q_gyro = gyroUpdateQuat(_attitude.quat, gyro_calib, dt);

    // 3. 加速度计解算四元数（低频校准）
    AuxMath::Vec3 euler_acc = accToEuler(imu_data.acc);
    AuxMath::Quat q_acc;
    AuxMath::EulerToQuat(euler_acc, q_acc);

    // 4. 互补滤波融合：q = alpha*q_gyro + (1-alpha)*q_acc
    // 四元数线性融合后重新归一化，保证单位四元数
    _attitude.quat.w = _alpha * q_gyro.w + (1.0f - _alpha) * q_acc.w;
    _attitude.quat.x = _alpha * q_gyro.x + (1.0f - _alpha) * q_acc.x;
    _attitude.quat.y = _alpha * q_gyro.y + (1.0f - _alpha) * q_acc.y;
    _attitude.quat.z = _alpha * q_gyro.z + (1.0f - _alpha) * q_acc.z;
    AuxMath::QuatNormalize(_attitude.quat);

    // 5. 四元数转欧拉角，供外部使用（工程中推荐直接用四元数，避免万向节锁死）
    AuxMath::QuatToEuler(_attitude.quat, _attitude.euler);

    return _attitude;
}

} // namespace SixAxisIMU