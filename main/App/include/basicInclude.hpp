#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "AuxiliaryMath.hpp"

#include <cstdint>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/uart.h"

typedef struct
{
    QueueHandle_t xSensorQueue;
    SemaphoreHandle_t xInitCountSem;
    EventGroupHandle_t xStartSyncGroup;
} rtosHandler;

extern rtosHandler rtoshandler;

#define START_SYNC_BIT (1 << 0)