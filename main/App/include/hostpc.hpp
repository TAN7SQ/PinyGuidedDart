#pragma once

#include "basicInclude.hpp"
#include "uart.hpp"

class HostPC
{
public:
    static constexpr const char *TAG = "HostPC";
    HostPC()
        : muart1(GPIO_NUM_41, GPIO_NUM_46, 1500000, UART_NUM_1), //
          muart2(GPIO_NUM_12, GPIO_NUM_7, 115200, UART_NUM_2) {};

    static HostPC &GetInstance()
    {
        static HostPC instance;
        return instance;
    }

    HostPC(const HostPC &) = delete;
    HostPC &operator=(const HostPC &) = delete;

    static void HostPCTask(void *pvParameters);

private:
    uart muart1;
    uart muart2;

    xAxisIMU::IMUAttitude imuAttitude;
    sensor::MS5611::ConvertData baroData;
};
