#include <chrono>
#include <functional>

#include "bmi088.hpp"
#include "spi_bus.hpp"

#include "i2c_bus.hpp"
#include "ms5611.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "main"

#define BMI088_HOST SPI2_HOST
#define BMI088_CLK 13     // SPI时钟引脚
#define BMI088_MOSI 12    // SPI主机输出引脚
#define BMI088_MISO 11    // SPI主机输入引脚
#define BMI088_CS_GYRO 10 // 陀螺仪片选引脚
#define BMI088_CS_ACCEL 9 // 加速度计片选引脚

// extern "C" void app_main(void)
// {
//     ESP_LOGI(TAG, "App start");

//     spi::BusConfig bus_cfg{
//         .host_num = BMI088_HOST,
//         .sclk_pin = (gpio_num_t)BMI088_CLK,
//         .mosi_pin = (gpio_num_t)BMI088_MOSI,
//         .miso_pin = (gpio_num_t)BMI088_MISO,
//         .max_transfer_sz = 32,
//         .dma_chan = SPI_DMA_CH_AUTO,
//     };
//     spi::SPIBus &spi_bus = spi::SPIBus::get_instance(bus_cfg);

//     spi::DeviceConfig gyro_cfg{
//         .clock_speed_hz = (uint16_t)(10 * 1000 * 1000),
//         .cs_pin = (gpio_num_t)BMI088_CS_GYRO,
//         .mode = 3,
//         .cs_ena_pretrans = 4,
//         .cs_ena_posttrans = 8,
//         .queue_size = 5,
//     };

//     spi::DeviceConfig accel_cfg{
//         .clock_speed_hz = (uint16_t)(10 * 1000 * 1000),
//         .cs_pin = (gpio_num_t)BMI088_CS_ACCEL,
//         .mode = 3,
//         .cs_ena_pretrans = 4,
//         .cs_ena_posttrans = 8,
//         .queue_size = 5,
//     };
//     sensor::BMI088 bmi088(accel_cfg, gyro_cfg);
//     bmi088.init();

//     while (1) {
//         sensor::BMI088::Data data;
//         bmi088.read_data(data);

//         ESP_LOGI(TAG, "gyro RAW: X=%d, Y=%d, Z=%d", data.gyro_x, data.gyro_y, data.gyro_z);
//         float acc_x_g = data.acc_x * 0.000088f;
//         float acc_y_g = data.acc_y * 0.000088f;
//         float acc_z_g = data.acc_z * 0.000088f;
//         float gyro_x_dps = data.gyro_x * 0.0152588f;
//         float gyro_y_dps = data.gyro_y * 0.0152588f;
//         float gyro_z_dps = data.gyro_z * 0.0152588f;

//         // 打印数据
//         ESP_LOGI(TAG, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", acc_x_g, acc_y_g, acc_z_g, gyro_x_dps, gyro_y_dps,
//         gyro_z_dps);

//         vTaskDelay(pdMS_TO_TICKS(20));
//     }
// }
extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "App start");

    // 初始化I2C总线
    i2c::I2CBus &i2c_bus = i2c::I2CBus::get_instance();

    // 创建MS5611传感器实例
    sensor::MS5611 ms5611(i2c_bus, 0x77, sensor::OSR::OSR_2048);

    // 初始化传感器
    esp_err_t ret = ms5611.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MS5611 initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "MS5611 initialized successfully");

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