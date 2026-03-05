#include "AuxiliaryMath.hpp"
#include <cstring>

namespace AuxMath
{

void Vec3Normalize(Vec3 &v)
{
    float norm = sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
    if (norm > 1e-6f) {
        v.x /= norm;
        v.y /= norm;
        v.z /= norm;
    }
}

void QuatNormalize(Quat &q)
{
    float norm = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm > 1e-6f) {
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }
}

void Mat3Identity(Mat3x3 &mat)
{
    memset(mat.m, 0, sizeof(mat.m));
    mat.m[0][0] = mat.m[1][1] = mat.m[2][2] = 1.0f;
}

void Mat3Scale(Mat3x3 &result, const Mat3x3 &mat, float scale)
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = mat.m[i][j] * scale;
        }
    }
}

void Mat7Identity(Mat7x7 &mat)
{
    memset(mat.m, 0, sizeof(mat.m));
    // 循环赋值，避免手写7个，简洁不易错
    for (int i = 0; i < 7; ++i) {
        mat.m[i][i] = 1.0f;
    }
}

void Mat7Zero(Mat7x7 &mat)
{
    memset(mat.m, 0, sizeof(mat.m));
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
    euler.x = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1.0f) {
        euler.y = copysignf(M_PI / 2.0f, sinp);
    }
    else {
        euler.y = asinf(sinp);
    }

    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    euler.z = atan2f(siny_cosp, cosy_cosp);
}

void EulerToQuat(const Vec3 &euler, Quat &q)
{
    float roll = euler.x, pitch = euler.y, yaw = euler.z;
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    QuatNormalize(q);
}

} // namespace AuxMath