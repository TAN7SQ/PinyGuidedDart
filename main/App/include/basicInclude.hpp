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

#define LOG_DATA_MAX_LEN 256
#define LOG_QUEUE_LEN 10

#define SENSOR_DATA_MAX_LEN 256
#define SENSOR_QUEUE_LEN 10

#define START_SYNC_BIT (1 << 0)

/************************************************/

typedef struct
{
    QueueHandle_t imuQueueRaw;
    QueueHandle_t imuQueueFiltered;

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

    /************************************************************************ */
    static std::vector<const char *> gTaskNames;
    static uint8_t TASK_TOTAL_NUM;

    static void taskCreate(TaskFunction_t task_function, const char *name, uint32_t stack_depth, void *pvParameters,
                           uint32_t priority, TaskHandle_t *pxCreatedTask, BaseType_t core_id)
    {
        if (xTaskCreatePinnedToCore(task_function, name, stack_depth, pvParameters, priority, NULL, core_id) !=
            pdPASS) {
            ESP_LOGE(TAG, "Failed to create task %s", name);
            TASK_TOTAL_NUM++;
        }
        Tools::gTaskNames.push_back(name);
    };

    static void AppManagerTask(void *pvParameters)
    {
        vTaskDelay(200);
        for (int i = 0; i < TASK_TOTAL_NUM; i++) {
            const char *taskName = Tools::gTaskNames[i];
            if (xSemaphoreTake(rtoshandler.InitCountSem, pdMS_TO_TICKS(3000)) != pdTRUE) {
                ESP_LOGE("APPMAN", "wait task %d init timeout: %s", i + 1, taskName);
                abort();
            }
            ESP_LOGI("APPMAN", "task %d init done: %s", i + 1, taskName);
        }

        ESP_LOGI("APPMAN", "all tasks init done, start sync semaphore");
        printf("---------------------------------------------\n");
        ESP_LOGW("APPMAN", "Free heap: %.2f KB", esp_get_free_heap_size() / (1024.f));
        xEventGroupSetBits(rtoshandler.StartSyncGroup, START_SYNC_BIT);
        vTaskDelete(NULL);
    }

    /************************************************************************ */
};

class Comm
{
public:
    static constexpr const char *TAG = "Comm";
    struct ControlCmd_t
    {
        float yaw_rate_cmd;
        float pitch_rate_cmd;
        uint8_t valid;
    };
};
