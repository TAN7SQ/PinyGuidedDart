
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

#define CORRECT_IMU_DATA 1

typedef struct
{
    float accelT[3][3];
    float accelOffs[3];
} AccCaliParams_s;

typedef struct
{
    float gxBias, gyBias, gzBias;
    float gxTcoK, gxTcoB0;
    float gyTcoK, gyTcoB0;
    float gzTcoK, gzTcoB0;
} GyroCaliParams_s;

// default accelerometer calibration
// static constexpr AccCaliParams_s ACC_CALI = { //六面校准结果
//     .accelT = {{0.990712216f, 0.000017548f, 0.000320257f},
//                {0.000017523f, 0.993296551f, 0.000100572f},
//                {0.000320266f, 0.000100483f, 0.991154054f}},
//     .accelOffs = {0.003507002f, 0.001457691f, -0.013434193f}};
static constexpr AccCaliParams_s ACC_CALI = {.accelT =
                                                 {
                                                     {0.992426304f, -0.007008484f, 0.000702798f},
                                                     {-0.007008484f, 0.995577745f, 0.000723205f},
                                                     {0.000702798f, 0.000723205f, 0.992946840f},
                                                 },
                                             .accelOffs = {0.003505164f, 0.001532927f, -0.013584518f}};
// default gyroscope calibration
static constexpr GyroCaliParams_s GYRO_CALI = {

    .gxBias = -0.898322f,
    .gyBias = -4.99465f,
    .gzBias = -0.234681f,
    .gxTcoK = 0.f,
    .gxTcoB0 = 0.f,
    .gyTcoK = 0.f,
    .gyTcoB0 = 0.f,
    .gzTcoK = 0.f,
    .gzTcoB0 = 0.f};

typedef struct
{
    float gx;
    float gy;
    float gz;
    float ax;
    float ay;
    float az;
} CaliOutput_s;

class IMUCalibration
{
    static constexpr float BIAS_ALPHA = 0.007f;
    static constexpr uint16_t STEADY_CNT_MAX = 10;
    static constexpr float STEADY_ACCEL_RANGE = 0.28f;
    static constexpr float STEADY_GYRO_RANGE = 0.03f;
    static constexpr float GYRO_BIAS_MAX_RAW = 100.f;
    static constexpr float G = 9.80665f; // (GuangZhou) Default gravity constant in m/s^2

public:
    void init(const AccCaliParams_s &_accCali, const GyroCaliParams_s &_gyroCali);
    void correctA(int16_t _ax, int16_t _ay, int16_t _az);
    void correctG(int16_t _gx, int16_t _gy, int16_t _gz);
    void correctM(int16_t _mx, int16_t _my, int16_t _mz);

    const CaliOutput_s &getOutput() const
    {
        return corrDat_;
    }

    // Detect steady state for gyro calibration
    void steadyStateDetection();
    void updateTemperature(float _temp)
    {
        temperature_ = _temp;
    }

private:
    AccCaliParams_s acc_cali_;
    GyroCaliParams_s gyro_cali_;

    // steady state detection parameters
    uint16_t staticSteadyStateCnt_ = 0;

    CaliOutput_s corrDat_; // calibrated output

    float temperature_ = 0.0f; // temperature data , unit:degC

    static constexpr float aTransK_ = 1.0f;
    float gTransK_ = 1.0f;
};