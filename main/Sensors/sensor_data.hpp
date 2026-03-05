#pragma once

#include <array>
#include <cstdint>

namespace sensorData
{

static constexpr uint8_t PREFIX = 0xA5; /* Packet header (0xA5) */

typedef struct __attribute__((packed))
{
    float x;
    float y;
    float z;
} Vector3;

typedef struct __attribute__((packed))
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

typedef struct __attribute__((packed))
{
    float rol;
    float pit;
    float yaw;
} EulerAngles;

typedef struct __attribute__((packed))
{
    uint8_t prefix; /* Packet header (0xA5) */
    uint64_t time : 40;
    uint64_t sync : 40;
    Quaternion quat_;
    Vector3 gyro_;
    Vector3 accl_;
    EulerAngles eulr_;
    uint8_t crc8;
} Data;

}; // namespace sensorData
