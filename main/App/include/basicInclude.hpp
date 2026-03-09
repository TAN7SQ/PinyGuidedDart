#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <cstdint>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/uart.h"
/************************************************/

#include "AuxiliaryMath.hpp"

#include "ms5611.hpp"
#include "servo.hpp"

/************************************************/

#define START_SYNC_BIT (1 << 0)

/************************************************/

typedef struct
{
    QueueHandle_t imuQueue;
    QueueHandle_t BaroQueue;

    QueueHandle_t ControlQueue;

    SemaphoreHandle_t InitCountSem;
    EventGroupHandle_t StartSyncGroup;
} rtosHandler;
extern rtosHandler rtoshandler;
esp_err_t rtosHandlerInit(void);

/************************************************/

class Tools
{
public:
    static constexpr const char *TAG = "Tools";

    static uint16_t crc16_ccitt(const uint8_t *data, int len);
};

class Comm
{
public:
    static constexpr const char *TAG = "Comm";
    struct ControlData
    {
        uint8_t mode;
        uint16_t servo_us[Servo::ALL]; // 舵机角度，单位：微秒
    };
};
