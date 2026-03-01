
#pragma once

#include "cstdint"

#include "esp_err.h"
#include "esp_log.h"

#include "AuxiliaryMath.hpp"
#include "servo.hpp"
typedef struct
{
    uint16_t LF = 0;
    uint16_t RF = 0;
    uint16_t LB = 0;
    uint16_t LB1 = 0;
    uint16_t RB = 0;
    uint16_t RB1 = 0;

    uint16_t FAN = 0;
} ServoTarget;
class Control
{
public:
    static constexpr char *TAG = "Ctrl";
    Control() = default;
    ~Control() = default;

    esp_err_t Init(Servo servo);
    esp_err_t ControlLoop(const xAxisIMU::IMUAttitude imuAttitude);

private:
    xAxisIMU::IMUAttitude _imuAttitude;
    Servo _servo;
    ServoTarget _servoTarget;
};