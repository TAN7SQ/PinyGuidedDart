#include "application.hpp"

#include <chrono>
#include <functional>

#include "bmi088.hpp"
#include "spi_bus.hpp"

#include "i2c_bus.hpp"
#include "ms5611.hpp"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "beeper.hpp"
#include "mlog.hpp"

#include "tfcard.hpp"
#include "ws2812.hpp"

//************************************************************ */
QueueHandle_t xLogQueue = NULL;
SemaphoreHandle_t xTFCardMutex = NULL;
//************************************************************ */

void LedTask(void *pvParameters)
{

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << GPIO_NUM_38,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t gpio_ret = gpio_config(&gpio_cfg);
    if (gpio_ret != ESP_OK) {
        ESP_LOGE("led_task", "GPIO config failed, err code: %d", gpio_ret);
        vTaskDelete(NULL); // 配置失败则删除当前任务
        return;
    }
    led_strip_handle_t led_strip = configure_led();
    if (led_strip == NULL) {
        ESP_LOGE(Application::TAG, "LED strip configuration failed");
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        gpio_set_level(GPIO_NUM_38, 0);
        ws2812b_RGBOn(led_strip, 0, 70, 70, 70);
        ws2812b_RGBOn(led_strip, 1, 70, 70, 70);
        vTaskDelay(pdMS_TO_TICKS(500));
        ws2812b_Off(led_strip, 0);
        ws2812b_Off(led_strip, 1);
        gpio_set_level(GPIO_NUM_38, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void SensorI2cTask(void *pvParameters)
{
    ESP_LOGI(Application::TAG, "ms5611_task start");
    i2c::BusConfig i2c_bus_config = {
        .port = I2C_NUM_1,               //
        .scl_pin = GPIO_NUM_10,          //
        .sda_pin = GPIO_NUM_11,          //
        .clk_speed_hz = 1 * 1000 * 1000, //
    };
    i2c::I2CBus &i2c_bus = i2c::I2CBus::get_instance(i2c_bus_config);
    sensor::MS5611 ms5611(i2c_bus, 0x77, sensor::OSR::OSR_512);
    esp_err_t ret = ms5611.init();
    if (ret != ESP_OK) {
        ESP_LOGE(Application::TAG, "MS5611 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        sensor::MS5611::Data data;
        ret = ms5611.read_data(data);
        if (ret != ESP_OK) {
            ESP_LOGE(Application::TAG, "MS5611 read data failed: %s", esp_err_to_name(ret));
            continue;
        }
        // ESP_LOGI(Application::TAG, "%.2f,%ld", data.pressure_mbar, data.raw_d1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void SensorSpiTask(void *pvParameters)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void key_task(void *pvParameters)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << GPIO_NUM_35,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t gpio_ret = gpio_config(&gpio_cfg);
    if (gpio_ret != ESP_OK) {
        ESP_LOGE("key_task", "GPIO config failed, err code: %d", gpio_ret);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        if (gpio_get_level(GPIO_NUM_35) == 1) {
            ESP_LOGI("key_task", "key pressed");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#include "servo.hpp"
void ServoTask(void *pvParameters)
{
    Servo mServo;
    mServo.Initialize();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void LogTask(void *pvParameters)
{
    TF_Card *tfCard = (TF_Card *)pvParameters;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        // TODO: 设置队列，并异步地写入TF卡，句柄应该通过参数来传递
    }
}

Application::Application()
{
    ESP_LOGI(Application::TAG, "App start");
}

Application::~Application()
{
}

void Application::Initialize()
{
    ESP_LOGI(Application::TAG, "Application开始初始化");

    /************************  ************************/
    xLogQueue = xQueueCreate(LOG_QUEUE_LEN, LOG_DATA_MAX_LEN);
    if (xLogQueue == NULL) {
        ESP_LOGE(Application::TAG, "Failed to create log queue");
        return;
    }
    xTFCardMutex = xSemaphoreCreateMutex();
    if (xTFCardMutex == NULL) {
        ESP_LOGE(Application::TAG, "Failed to create TF card mutex");
        return;
    }

    /************************  ************************/
    esp_err_t tf_ret = this->tfCard.Initialize();
    if (tf_ret != ESP_OK) {
        ESP_LOGE(Application::TAG, "TF card initialization failed: %s", esp_err_to_name(tf_ret));
        return;
    }
    ESP_LOGI(Application::TAG, "TF card initialized successfully");

    /************************  ************************/
    LogTaskParams_t log_task_params = {
        .tf_card_ptr = &this->tfCard, //
        .log_queue = xLogQueue        //
    };
    if (xTaskCreatePinnedToCore(LogTask,              //
                                "log_task",           //
                                4096,                 //
                                &log_task_params,     //
                                tskIDLE_PRIORITY + 1, //
                                NULL,                 //
                                1) != pdPASS) {       //
        ESP_LOGE(Application::TAG, "Failed to create log task");
        return;
    }

    /************************ 第四步：创建其他外设任务 ************************/
    xTaskCreatePinnedToCore(LedTask, "led_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(key_task, "key_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(SensorI2cTask, "SensorI2cTask", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(ServoTask, "servo_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);

    ESP_LOGI(Application::TAG, "All tasks created successfully, Application initialized");
}

void Application::Run()
{
    ESP_LOGI(Application::TAG, "App run");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}