#pragma once

#include <cmath>
#include <cstring>
#include <iostream>

namespace AuxMath
{
#ifndef M_PI
constexpr float M_PI = 3.14159265358979323846f;
#endif

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

// 四元数：表示姿态（单位四元数）
struct Quat
{
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    Quat() = default;
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_)
    {
    }
};

// 3x3矩阵
struct Mat3x3
{
    float m[3][3] = {{0.0f}};
    Mat3x3() = default;
};

// 7x7矩阵
struct Mat7x7
{
    float m[7][7] = {{0.0f}};
    Mat7x7() = default;
};

// EKF核心状态：4维四元数 + 3维陀螺偏置
struct EKFState
{
    Quat quat; // 姿态四元数
    Vec3 bias; // 陀螺仪偏置
    EKFState() = default;
};

//===============================================================
/**
 * @brief 归一化向量
 *
 * @param v
 */
void Vec3Normalize(Vec3 &v)
{
    float norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (norm > 0.0f) {
        v.x /= norm;
        v.y /= norm;
        v.z /= norm;
    }
}

//===============================================================
/**
 * @brief 归一化四元数
 *
 * @param q
 */
void QuatNormalize(Quat &q)
{
    float norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 0.0f) {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
}

/**
 * @brief 3x3矩阵设为单位矩阵
 *
 * @param mat
 */
void Mat3Identity(Mat3x3 &mat)
{
    std::memset(mat.m, 0, sizeof(mat.m));
    mat.m[0][0] = mat.m[1][1] = mat.m[2][2] = 1.0f;
}

/**
 * @brief 3x3矩阵按比例缩放
 *
 * @param result
 * @param mat
 * @param scale
 */
void Mat3Scale(Mat3x3 &result, const Mat3x3 &mat, float scale)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = mat.m[i][j] * scale;
        }
    }
}

/**
 * @brief 7x7矩阵设为单位矩阵
 *
 * @param mat
 */
void Mat7Identity(Mat7x7 &mat)
{
    std::memset(mat.m, 0, sizeof(mat.m));
    mat.m[0][0] = mat.m[1][1] = mat.m[2][2] = mat.m[3][3] = mat.m[4][4] = mat.m[5][5] = mat.m[6][6] = 1.0f;
}

/**
 * @brief 7x7矩阵设为零矩阵
 *
 * @param mat
 */
void Mat7Zero(Mat7x7 &mat)
{
    std::memset(mat.m, 0, sizeof(mat.m));
}

void Mat7Scale(Mat7x7 &result, const Mat7x7 &mat, float scale)
{
    for (int i = 0; i < 7; ++i) {
        for (int j = 0; j < 7; ++j) {
            result.m[i][j] = mat.m[i][j] * scale;
        }
    }
}

void QuatToEuler(const Quat &q, Vec3 &euler)
{

    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    euler.x = atan2(sinr_cosp, cosr_cosp);
    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1.0f) {
        euler.y = copysign(M_PI / 2.0f, sinp); // 避免计算错误
    }
    else {
        euler.y = asin(sinp);
    }
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.z = atan2(siny_cosp, cosy_cosp);
}

void EulerToQuat(const Vec3 &euler, Quat &q)
{
    float roll = euler.x, pitch = euler.y, yaw = euler.z;
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

} // namespace AuxMath