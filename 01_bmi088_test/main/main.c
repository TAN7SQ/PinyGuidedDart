/*
 * BMI088 SPI驱动完整代码（ESP32）
 * 功能：读取BMI088加速度计和陀螺仪数据，支持全双工SPI、数据滤波、错误处理
 * 引脚定义：CLK=13, MOSI=12, MISO=11, CS_GYRO=10, CS_ACCEL=9
 * 配置：加速度计±3g量程/50Hz输出，陀螺仪±500°/s量程/100Hz输出
 */
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "bmi088.h"
#include "ms5611.h"
// ========================== 应用入口 ==========================
void app_main(void)
{
    printf("APP Start!\n");

    // i2c_init();
    // i2c_scan_addr(0x77); // 存在，气压计 MS5611
    // i2c_scan_addr(0x0d); // 不在，怀疑焊接有问题， 陀螺仪 QMC5883l
    // i2c_scan_addr(0x6a); // 存在，地磁QMI8658C

    // ms5611_init();
    // xTaskCreate(bmi088_task, "bmi088_task", 10240, NULL, 5, NULL);
    xTaskCreate(ms5611_task, "ms5611_task", 10240, NULL, 5, NULL);

    // 主任务空循环
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}