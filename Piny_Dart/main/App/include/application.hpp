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

#include "beeper.hpp"
#include "tfcard.hpp"
#include "wifi_udp_client.hpp"

#include <iostream>

extern QueueHandle_t xLogQueue;
extern SemaphoreHandle_t xTFCardMutex;

typedef struct
{
    TF_Card *tf_card_ptr;
    QueueHandle_t log_queue;
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
    Application(const Application &) = delete;
    Application &operator=(const Application &) = delete;
    void Initialize();
    void Run();

    //============================================
    Beeper beeper;
    static Beeper *sBeeper;

    TF_Card tfCard;
    WifiUdpClient &client; // 引用成员只能在初始化列表中初始化, 不能在构造函数中初始化
    WifiUdpClient::WifiUdpConfig client_config;

private:
    SemaphoreHandle_t xTFCardMutex = NULL;
    QueueHandle_t xSpiSensorQueue = NULL;
    QueueHandle_t xI2cSensorQueue = NULL;
    QueueHandle_t xLogQueue = NULL;
    QueueHandle_t xRecvPCQueue = NULL;
    QueueHandle_t xSendPCQueue = NULL;

    Application();
    ~Application();
};

#endif // APPLICATION_HPP