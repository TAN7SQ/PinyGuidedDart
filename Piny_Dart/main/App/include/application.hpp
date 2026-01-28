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

// 日志数据的最大长度
#define LOG_DATA_MAX_LEN 256
// 日志队列的缓存长度
#define LOG_QUEUE_LEN 10

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