#ifndef AUXILIARY_MATH_HPP
#define AUXILIARY_MATH_HPP

#include <cmath>

namespace AuxMath
{

struct Vec3
{
    union {
        struct
        {
            float x, y, z;
        };
        struct
        {
            float roll, pitch, yaw;
        };
        float v[3];
    };
    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_)
    {
    }
};

struct _Vec3
{
    union {
        struct
        {
            float x, y, z;
        };
        struct
        {
            float roll, pitch, yaw;
        };
        float v[3];
    };
};

struct Quat
{
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_)
    {
    }
};

struct _Quat
{
    float w, x, y, z;
};

struct Mat3x3
{
    float m[3][3] = {{0.0f}};
    Mat3x3() = default;
};

struct Mat7x7
{
    float m[7][7] = {{0.0f}};
    Mat7x7() = default;
};

struct EKFState
{
    Quat quat; // 姿态四元数
    Vec3 bias; // 陀螺仪三轴偏置
    EKFState() = default;
};

void Vec3Normalize(Vec3 &v);                                    // 归一化三维向量
void QuatNormalize(Quat &q);                                    // 归一化四元数
void Mat3Identity(Mat3x3 &mat);                                 // 3x3矩阵置单位矩阵
void Mat3Scale(Mat3x3 &result, const Mat3x3 &mat, float scale); // 3x3矩阵缩放
void Mat7Identity(Mat7x7 &mat);                                 // 7x7矩阵置单位矩阵
void Mat7Zero(Mat7x7 &mat);                                     // 7x7矩阵置零矩阵
void Mat7Scale(Mat7x7 &result, const Mat7x7 &mat, float scale); // 7x7矩阵缩放
void QuatToEuler(const Quat &q, Vec3 &euler);                   // 四元数转欧拉角（弧度）
void EulerToQuat(const Vec3 &euler, Quat &q);                   // 欧拉角转四元数（弧度）

} // namespace AuxMath

namespace xAxisIMU
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

} // namespace xAxisIMU
#endif // AUXILIARY_MATH_HPP  // 头文件保护结束