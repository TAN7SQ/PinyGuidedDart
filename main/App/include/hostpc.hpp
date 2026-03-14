#pragma once

#include "basicInclude.hpp"
#include "uart.hpp"

#include "servo.hpp"

class HostPC
{

    /**
     * @brief 主机PC与ESP32通信协议
     * 1~9    传感器
     * 10~19  控制
     * 20~29  系统
     * 100+   调试
     */
    enum class MsgType : uint8_t
    {
        IMU = 1,
        BARO = 2,
        ATTITUDE = 3,

        CONTROL = 10,

        SYSTEM = 20,

        DEBUG = 100,
    };

public:
    static constexpr const char *TAG = "HostPC";

    static HostPC &GetInstance()
    {
        static HostPC instance;
        return instance;
    }

    HostPC(const HostPC &) = delete;
    HostPC &operator=(const HostPC &) = delete;

    void start(void)
    {
        Tools::taskCreate(HostPCTask, "hostpc_task", 8192, this, tskIDLE_PRIORITY + 3, NULL, 0);
    }

private:
    uart muart1;
    uart muart2;

    xAxisIMU::IMUAttitude imuAttitude;
    sensor::MS5611::ConvertData baro;

private:
    HostPC()
        : muart1(GPIO_NUM_41, GPIO_NUM_46, 1500000, UART_NUM_1), //
          muart2(GPIO_NUM_12, GPIO_NUM_7, 115200, UART_NUM_2)
    {
        this->muart1.initialize();
        // this->muart2.initialize();
    };

    static void HostPCTask(void *param)
    {
        HostPC *self = static_cast<HostPC *>(param);
        self->run();
    }

    void run();
    void update();

    esp_err_t sendData(MsgType type, const void *payload, uint16_t payloadLength);
    esp_err_t receiveData(void *payload, uint16_t *payloadLength);
    esp_err_t parseData(const MsgType type, void *payload, uint16_t payloadLength);
};
