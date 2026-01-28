#include "application.hpp"

#include <chrono>
#include <functional>

#include "bmi088.hpp"
#include "spi_bus.hpp"

#include "i2c_bus.hpp"
#include "ms5611.hpp"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "beeper.hpp"
#include "mlog.hpp"

#include "tfcard.hpp"

void led_task(void *pvParameters)
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

    while (1) {
        gpio_set_level(GPIO_NUM_38, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(GPIO_NUM_38, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

#include "ws2812.hpp"

void ws2812_task(void *pvParameters)
{
    ESP_LOGI(Application::TAG, "ws2812_task start");
    led_strip_handle_t led_strip = configure_led();
    if (led_strip == NULL) {
        ESP_LOGE(Application::TAG, "LED strip configuration failed");
        vTaskDelete(NULL);
        return;
    }
    while (1) {
        // ws2812b_rainbow(led_strip);
        ws2812b_RGBOn(led_strip, 0, 70, 70, 70);
        ws2812b_RGBOn(led_strip, 1, 70, 70, 70);
        vTaskDelay(pdMS_TO_TICKS(500));
        ws2812b_Off(led_strip, 0);
        ws2812b_Off(led_strip, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void ms5611_task(void *pvParameters)
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
void servo_task(void *pvParameters)
{
    Servo mServo;
    // mServo.Initialize();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void tfcard_task(void *pvParameters)
{

    vTaskDelete(NULL);
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
    ESP_LOGI(Application::TAG, "App start");

    this->tfCard.Initialize();

    xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(ws2812_task, "ws2812_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(ms5611_task, "ms5611_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(key_task, "key_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);

    xTaskCreatePinnedToCore(tfcard_task, "tfcard_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);

    // vTaskDelay(pdMS_TO_TICKS(10));
    // Beeper beeper(GPIO_NUM_21);
}

void Application::Run()
{
    ESP_LOGI(Application::TAG, "App run");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}