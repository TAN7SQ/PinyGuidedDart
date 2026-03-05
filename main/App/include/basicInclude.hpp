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

extern QueueHandle_t xSensorQueue;

extern SemaphoreHandle_t xInitCountSem;
extern EventGroupHandle_t xStartSyncGroup;
#define START_SYNC_BIT (1 << 0)