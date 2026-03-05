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

/************************************************/

#define START_SYNC_BIT (1 << 0)

/************************************************/

typedef struct
{
    QueueHandle_t xImuQueue;
    QueueHandle_t xBaroQueue;
    SemaphoreHandle_t xInitCountSem;
    EventGroupHandle_t xStartSyncGroup;
} rtosHandler;

extern rtosHandler rtoshandler;

esp_err_t rtosHandlerInit(void);

class Tools
{
public:
    static uint16_t crc16_ccitt(const uint8_t *data, int len);
};
