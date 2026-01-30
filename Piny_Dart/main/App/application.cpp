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
#define LOG_DATA_MAX_LEN 256
#define LOG_QUEUE_LEN 10
// QueueHandle_t xLogQueue = NULL;

#define SENSOR_DATA_MAX_LEN 256
#define SENSOR_QUEUE_LEN 10
// QueueHandle_t xSensorQueue = NULL;
// SemaphoreHandle_t xTFCardMutex = NULL;
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
    QueueHandle_t xSensorQueue = (QueueHandle_t)pvParameters;

    spi::BusConfig spi_bus_config = {
        .host_num = SPI2_HOST,
        .sclk_pin = GPIO_NUM_12, //
        .mosi_pin = GPIO_NUM_13, //
        .miso_pin = GPIO_NUM_14, //
    };
    spi::SPIBus &spiBus = spi::SPIBus::get_instance(spi_bus_config);

    spi::DeviceConfig bmi088_acc_cfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .cs_pin = GPIO_NUM_15,
    };
    spi::DeviceConfig bmi088_gyro_cfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .cs_pin = GPIO_NUM_16,
    };

    sensor::BMI088 bmi088(bmi088_acc_cfg, bmi088_gyro_cfg);
    esp_err_t ret = bmi088.init();
    if (ret != ESP_OK) {
        ESP_LOGE(Application::TAG, "BMI088 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        sensor::BMI088::Data data;
        ret = bmi088.read_data(data);
        if (ret != ESP_OK) {
            ESP_LOGE(Application::TAG, "BMI088 read data failed: %s", esp_err_to_name(ret));
            continue;
        }
        // 发送队列
        // ESP_LOGI(Application::TAG,
        //          "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
        //          data.acc_x_g,
        //          data.acc_y_g,
        //          data.acc_z_g,
        //          data.gyro_x_dps,
        //          data.gyro_y_dps,
        //          data.gyro_z_dps);

        // TODO:发送数据到主机
        xQueueSend(xSensorQueue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// send sensor and log data to host pc
#include "uart.hpp"
void HostPCTask(void *pvParameters)
{
    uart muart1(GPIO_NUM_46, GPIO_NUM_45, 115200, UART_NUM_1);
    uart muart2(GPIO_NUM_12, GPIO_NUM_7, 115200, UART_NUM_2);
    muart1.initialize();
    muart2.initialize();
    while (1) {
        muart1.write((const uint8_t *)"hello world1\n", strlen("hello world1\n"));
        vTaskDelay(pdMS_TO_TICKS(100));
        muart2.write((const uint8_t *)"hello world2\n", strlen("hello world2\n"));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void KeyTask(void *pvParameters)
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
void ControlTask(void *pvParameters)
{
    Servo mServo;
    mServo.Initialize();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 异步记录日志，同时使用串口发送、http发送
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
    ESP_LOGI(Application::TAG, "Application");

    /************************  ************************/
    this->xLogQueue = xQueueCreate(LOG_QUEUE_LEN, LOG_DATA_MAX_LEN);
    this->xSpiSensorQueue = xQueueCreate(SENSOR_QUEUE_LEN, SENSOR_DATA_MAX_LEN);
    if (this->xSpiSensorQueue == NULL || this->xLogQueue == NULL) {
        ESP_LOGE(Application::TAG, "Failed to create sensor queue");
        return;
    }
    this->xTFCardMutex = xSemaphoreCreateMutex();
    if (this->xTFCardMutex == NULL) {
        ESP_LOGE(Application::TAG, "Failed to create TF card mutex");
        return;
    }

    /************************ LOG SYSTEM INITIALIZE ************************/
    esp_err_t tf_ret = this->tfCard.Initialize();
    if (tf_ret != ESP_OK) {
        ESP_LOGE(Application::TAG, "TF card initialization failed: %s", esp_err_to_name(tf_ret));
        return;
    }
    ESP_LOGI(Application::TAG, "TF card initialized successfully");

    LogTaskParams_t log_task_params = {
        .tf_card_ptr = &this->tfCard, //
        .log_queue = this->xLogQueue  //
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

    /************************  ************************/
    xTaskCreatePinnedToCore(LedTask, "led_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(KeyTask, "key_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);

    xTaskCreatePinnedToCore(SensorI2cTask, "SensorI2cTask", 4096, NULL, tskIDLE_PRIORITY + 2, NULL, 0);
    xTaskCreatePinnedToCore(
        SensorSpiTask, "SensorSpiTask", 4096, &this->xSpiSensorQueue, tskIDLE_PRIORITY + 2, NULL, 0);

    xTaskCreatePinnedToCore(HostPCTask, "HostPCTask", 4096, NULL, tskIDLE_PRIORITY + 3, NULL, 0);
    xTaskCreatePinnedToCore(ControlTask, "control_task", 4096, NULL, tskIDLE_PRIORITY + 3, NULL, 0);

    ESP_LOGI(Application::TAG, "All tasks created successfully, Application initialized");
}

void Application::Run()
{
    Beeper beeper(GPIO_NUM_21);
    beeper.play_boot_music();
    ESP_LOGI(Application::TAG, "App run");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}