/*
 * BMI088 SPI驱动完整代码（ESP32）
 * 功能：读取BMI088加速度计和陀螺仪数据，支持全双工SPI、数据滤波、错误处理
 * 引脚定义：CLK=13, MOSI=12, MISO=11, CS_GYRO=10, CS_ACCEL=9
 * 配置：加速度计±3g量程/50Hz输出，陀螺仪±500°/s量程/100Hz输出
 */
// #include "esp_system.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "sdkconfig.h"
// #include <inttypes.h>
// #include <stdio.h>
// #include <string.h>

// #include "esp_log.h"

// #include "bmi088.h"
// #include "ms5611.h"
// ========================== 应用入口 ==========================
// void app_main(void)
// {
//     printf("APP Start!\n");

//     // i2c_init();
//     // i2c_scan_addr(0x77); // 存在，气压计 MS5611
//     // i2c_scan_addr(0x0d); // 不在，怀疑焊接有问题， 地磁传感器 QMC5883l
//     // i2c_scan_addr(0x6a); // 存在，陀螺仪 QMI8658C

//     // ms5611_init();
//     // xTaskCreate(bmi088_task, "bmi088_task", 10240, NULL, 5, NULL);
//     xTaskCreate(ms5611_task, "ms5611_task", 10240, NULL, 5, NULL);

//     // 主任务空循环
//     while (1) {
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

#include <chrono>
#include <functional>

#include "i2c.hpp"
#include "qmi8658.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void)
{
    // 初始化日志（简化版）
    espp::Logger logger({.tag = "QMI8658 Simple Read", .level = espp::Logger::Verbosity::INFO});
    logger.info("Starting QMI8658 simple data read example!");

    // 创建I2C实例
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_20,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    });

    // 定义QMI8658类型
    using Imu = espp::Qmi8658<espp::qmi8658::Interface::I2C>;

    // 配置IMU（移除所有滤波相关配置）
    Imu::Config config{
        .device_address = Imu::DEFAULT_ADDRESS_AD0_LOW,
        // 绑定I2C读写函数
        .write =
            std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        // IMU基础配置
        .imu_config =
            {
                .accelerometer_range = Imu::AccelerometerRange::RANGE_8G,
                .accelerometer_odr = Imu::ODR::ODR_250_HZ,
                .gyroscope_range = Imu::GyroscopeRange::RANGE_512_DPS,
                .gyroscope_odr = Imu::ODR::ODR_250_HZ,
            },
        // 禁用滤波和自动初始化（手动初始化更清晰）
        .orientation_filter = nullptr,
        .auto_init = true};

    // 创建IMU实例
    logger.info("Initializing QMI8658");
    Imu imu(config);

    // 打印数据表头
    fmt::print("Time (s), Accel X (m/s²), Accel Y (m/s²), Accel Z (m/s²), "
               "Gyro X (dps), Gyro Y (dps), Gyro Z (dps), Temperature (°C)\n");

    // 记录起始时间
    auto start_time = esp_timer_get_time();

    // 主循环读取数据
    while (true) {
        // 计算时间差（秒）
        auto current_time = esp_timer_get_time();
        float elapsed_time = (current_time - start_time) / 1000000.0f;

        // 读取传感器数据
        std::error_code ec;
        if (imu.update(0.004f, ec)) { // 250Hz对应4ms更新周期
            auto accel = imu.get_accelerometer();
            auto gyro = imu.get_gyroscope();
            float temp = imu.get_temperature();

            // 打印原始数据
            fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.1f}\n",
                       elapsed_time,
                       (float)accel.x,
                       (float)accel.y,
                       (float)accel.z,
                       (float)gyro.x,
                       (float)gyro.y,
                       (float)gyro.z,
                       temp);
        }
        else {
            logger.error("Failed to read QMI8658 data: {}", ec.message());
        }

        // 250Hz采样率，对应4ms延迟
        vTaskDelay(pdMS_TO_TICKS(4));
    }
}