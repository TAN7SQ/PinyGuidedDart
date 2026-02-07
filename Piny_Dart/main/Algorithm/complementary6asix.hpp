#ifndef SIX_AXIS_IMU_HPP
#define SIX_AXIS_IMU_HPP

#include "AuxiliaryMath.hpp" // 兼容你之前的数学工具库（Vec3/Quat）

namespace SixAxisIMU
{

// IMU原始数据结构体（陀螺仪+加速度计，单位：弧度/秒、m/s²）
struct IMURawData
{
    AuxMath::Vec3 gyro; // 陀螺仪数据：x(roll)、y(pitch)、z(yaw)，单位rad/s
    AuxMath::Vec3 acc;  // 加速度计数据：x、y、z，单位m/s²
};

// 姿态解算结果结构体（欧拉角+四元数，姿态角单位：弧度）
struct IMUAttitude
{
    AuxMath::Vec3 euler; // 欧拉角：x(roll)、y(pitch)、z(yaw)，单位rad
    AuxMath::Quat quat;  // 姿态四元数（单位四元数，推荐工程使用）
};

// 六轴IMU互补滤波姿态解算类
class ComplementaryFilter
{
public:
    /**
     * @brief 构造函数：初始化滤波参数和姿态
     * @param filter_alpha 滤波系数（0~1），越小越信任加速度计，越大越信任陀螺仪
     *                     推荐值：0.98（静态/低速）、0.99（中速）、0.995（高速）
     */
    ComplementaryFilter(float filter_alpha = 0.98f);

    /**
     * @brief 初始化姿态：从加速度计静态解算初始姿态（开机必调用，IMU需静止）
     * @param acc 加速度计原始数据（单位m/s²）
     * @return 初始化后的姿态四元数
     */
    AuxMath::Quat initAttitude(const AuxMath::Vec3 &acc);

    /**
     * @brief 核心姿态解算函数（主循环调用）
     * @param imu_data IMU原始数据（陀螺仪+加速度计）
     * @param dt 解算周期（主循环时间间隔，单位：秒，如10ms则传0.01f）
     * @return 解算后的姿态（欧拉角+四元数）
     */
    IMUAttitude update(const IMURawData &imu_data, float dt);

    /**
     * @brief 设置滤波系数
     * @param alpha 滤波系数（0~1）
     */
    void setFilterAlpha(float alpha)
    {
        _alpha = alpha;
    }

    /**
     * @brief 获取当前姿态
     * @return 当前姿态
     */
    IMUAttitude getAttitude() const
    {
        return _attitude;
    }

private:
    float _alpha;             // 互补滤波系数
    IMUAttitude _attitude;    // 当前姿态（欧拉角+四元数）
    AuxMath::Vec3 _gyro_bias; // 陀螺仪静态零漂（简单校准，减少漂移）

    /**
     * @brief 从加速度计解算横滚/俯仰角（静态有效）
     * @param acc 加速度计数据（m/s²）
     * @return 欧拉角（x(roll)、y(pitch)，z(yaw)=0），单位rad
     */
    AuxMath::Vec3 accToEuler(const AuxMath::Vec3 &acc);

    /**
     * @brief 陀螺仪积分更新四元数（基于四元数微分方程）
     * @param quat 当前四元数
     * @param gyro 陀螺仪数据（rad/s，已去零漂）
     * @param dt 积分时间（s）
     * @return 更新后的四元数
     */
    AuxMath::Quat gyroUpdateQuat(const AuxMath::Quat &quat, const AuxMath::Vec3 &gyro, float dt);
};

} // namespace SixAxisIMU

#endif // SIX_AXIS_IMU_HPP