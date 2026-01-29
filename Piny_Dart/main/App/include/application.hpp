#ifndef APPLICATION_HPP
#define APPLICATION_HPP
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

#include "freertos/semphr.h"
#include "freertos/task.h"
#include <freertos/event_groups.h>
#include <freertos/task.h>

#include "i2c_bus.hpp"
#include "spi_bus.hpp"

#include "tfcard.hpp"

#include <iostream>

extern QueueHandle_t xLogQueue;
extern SemaphoreHandle_t xTFCardMutex;

// 日志任务的参数结构体：封装TF卡对象指针 + 日志队列句柄
typedef struct
{
    TF_Card *tf_card_ptr;    // TF卡对象指针
    QueueHandle_t log_queue; // 日志队列句柄
} LogTaskParams_t;

void LogTask(void *pvParameters);

class Application
{
public:
    static constexpr const char *TAG = "Application";

    QueueHandle_t xSpiSensorQueue = NULL;
    QueueHandle_t xI2cSensorQueue = NULL;
    QueueHandle_t xLogQueue = NULL;
    QueueHandle_t xRecvPCQueue = NULL;
    QueueHandle_t xSendPCQueue = NULL;

    SemaphoreHandle_t xTFCardMutex = NULL;

    static Application &GetInstance()
    {
        static Application instance;
        return instance;
    }
    // Delete copy constructor and assignment operator
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    void Initialize();
    void Run();

    TF_Card tfCard;

private:
    Application();
    ~Application();
};

#endif // APPLICATION_HPP