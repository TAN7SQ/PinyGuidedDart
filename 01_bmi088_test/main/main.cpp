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

    ESP_LOGI(TAG, "ws2812_task start");
    led_strip_handle_t led_strip = configure_led();
    if (led_strip == NULL) {
        ESP_LOGE(TAG, "LED strip configuration failed");
        vTaskDelete(NULL); // 配置失败则删除当前任务
        return;
    }
    while (1) {
        // ws2812b_rainbow(led_strip);
        ws2812b_RGBOn(led_strip, 0, 100, 100, 100);
        ws2812b_RGBOn(led_strip, 1, 100, 100, 100);
        vTaskDelay(pdMS_TO_TICKS(500));
        ws2812b_Off(led_strip, 0);
        ws2812b_Off(led_strip, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void ms5611_task(void *pvParameters)
{
    ESP_LOGI(TAG, "ms5611_task start");
    i2c::BusConfig i2c_bus_config = {
        .port = I2C_NUM_1,      //
        .scl_pin = GPIO_NUM_10, //
        .sda_pin = GPIO_NUM_11, //
        .clk_speed_hz = 100000, //
    };
    i2c::I2CBus &i2c_bus = i2c::I2CBus::get_instance(i2c_bus_config);
    sensor::MS5611 ms5611(i2c_bus, 0x77, sensor::OSR::OSR_512);
    esp_err_t ret = ms5611.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MS5611 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL); // 配置失败则删除当前任务
        return;
    }
    while (1) {
        sensor::MS5611::Data data;
        ret = ms5611.read_data(data);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "MS5611 read data failed: %s", esp_err_to_name(ret));
            continue;
        }
        ESP_LOGI(TAG, "%.2f,%ld", data.pressure_mbar, data.raw_d1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{

#define BMI088_HOST SPI2_HOST
#define BMI088_CLK 33
#define BMI088_MOSI 34
#define BMI088_MISO 48
#define BMI088_CS_GYRO 26
#define BMI088_CS_ACCEL 47

    // mLog::changeLog(115200);
    ESP_LOGI(TAG, "App start");
    // gpio_reset_pin(GPIO_NUM_26);

    // spi::BusConfig bus_cfg{
    //     .host_num = BMI088_HOST,
    //     .sclk_pin = (gpio_num_t)BMI088_CLK,
    //     .mosi_pin = (gpio_num_t)BMI088_MOSI,
    //     .miso_pin = (gpio_num_t)BMI088_MISO,
    //     .max_transfer_sz = 32,
    //     .dma_chan = SPI_DMA_CH_AUTO,
    // };
    // spi::SPIBus &spi_bus = spi::SPIBus::get_instance(bus_cfg);

    // spi::DeviceConfig gyro_cfg{
    //     .clock_speed_hz = (uint16_t)(2 * 1000 * 1000),
    //     .cs_pin = (gpio_num_t)BMI088_CS_GYRO,
    //     .mode = 3,
    //     .cs_ena_pretrans = 4,
    //     .cs_ena_posttrans = 8,
    //     .queue_size = 5,
    // };

    // spi::DeviceConfig accel_cfg{
    //     .clock_speed_hz = (uint16_t)(2 * 1000 * 1000),
    //     .cs_pin = (gpio_num_t)BMI088_CS_ACCEL,
    //     .mode = 3,
    //     .cs_ena_pretrans = 4,
    //     .cs_ena_posttrans = 8,
    //     .queue_size = 5,
    // };
    // sensor::BMI088 bmi088(accel_cfg, gyro_cfg);
    // bmi088.init();
    // if (bmi088.init() != ESP_OK) {
    //     ESP_LOGE(TAG, "BMI088 initialization failed");
    //     return;
    // }

    xTaskCreatePinnedToCore(led_task, "led_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(ws2812_task, "ws2812_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);
    xTaskCreatePinnedToCore(ms5611_task, "ms5611_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 0);

    Beeper beeper(GPIO_NUM_21);
    while (1) {

        // sensor::BMI088::Data data;
        // bmi088.read_data(data);

        // ESP_LOGI(TAG, "gyro RAW: X=%d, Y=%d, Z=%d", data.gyro_x, data.gyro_y, data.gyro_z);
        // float acc_x_g = data.acc_x * 0.000088f;
        // float acc_y_g = data.acc_y * 0.000088f;
        // float acc_z_g = data.acc_z * 0.000088f;
        // float gyro_x_dps = data.gyro_x * 0.0152588f;
        // float gyro_y_dps = data.gyro_y * 0.0152588f;
        // float gyro_z_dps = data.gyro_z * 0.0152588f;

        // // 打印数据
        // ESP_LOGI(TAG, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", acc_x_g, acc_y_g, acc_z_g, gyro_x_dps, gyro_y_dps,
        // gyro_z_dps);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// namespace main
// {

// extern "C" void app_main(void)
// {
//
//
//
//
//
//
//
//

//     mLog::changeLog();

//     ESP_LOGI(TAG, "App start");

//     // // 初始化I2C总线
//     // i2c::I2CBus &i2c_bus = i2c::I2CBus::get_instance();

//     // // 创建MS5611传感器实例
//     // sensor::MS5611 ms5611(i2c_bus, 0x77, sensor::OSR::OSR_2048);

//     // // 初始化传感器
//     // esp_err_t ret = ms5611.init();
//     // if (ret != ESP_OK) {
//     //     ESP_LOGE(TAG, "MS5611 initialization failed: %s", esp_err_to_name(ret));
//     //     return;
//     // }

//     // ESP_LOGI(TAG, "MS5611 initialized successfully");

//     while (1) {
//         // sensor::MS5611::Data data;
//         // ret = ms5611.read_data(data);
//         // if (ret != ESP_OK) {
//         //     ESP_LOGE(TAG, "MS5611 read data failed: %s", esp_err_to_name(ret));
//         //     continue;
//         // }

//         // ESP_LOGI(TAG, "%.2f,%ld", data.pressure_mbar, data.raw_d1);

//         vTaskDelay(pdMS_TO_TICKS(100));
//         gpio_set_level(GPIO_NUM_38, 1);

//         vTaskDelay(pdMS_TO_TICKS(100));
//         gpio_set_level(GPIO_NUM_38, 0);
//     }
// }
// } // namespace main