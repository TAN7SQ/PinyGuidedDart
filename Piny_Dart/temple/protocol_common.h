#ifndef PROTOCOL_COMMON_H
#define PROTOCOL_COMMON_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 平台检测
#if defined(__linux__) || defined(ESP_PLATFORM)
    #define PLATFORM_SUPPORTED
#else
    #error "Unsupported platform"
#endif

// 在ESP-IDF上包含必要的头文件
#ifdef ESP_PLATFORM
    #include "driver/uart.h"
    #include "esp_log.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "lwip/sockets.h"
    #define LOG_TAG "Protocol"
    #define LOG_INFO(format, ...) ESP_LOGI(LOG_TAG, format, ##__VA_ARGS__)
    #define LOG_ERROR(format, ...) ESP_LOGE(LOG_TAG, format, ##__VA_ARGS__)
#else
    // Linux/MaixCam上的日志
    #include <stdio.h>
    #define LOG_INFO(format, ...) printf("[INFO] " format "\n", ##__VA_ARGS__)
    #define LOG_ERROR(format, ...) printf("[ERROR] " format "\n", ##__VA_ARGS__)
#endif

// 协议帧头定义
#define PROTOCOL_MAGIC 0xAA55
#define PROTOCOL_VERSION 0x01

// 传感器数据结构体
#pragma pack(push, 1)
typedef struct
{
    float accel_x;       // 加速度 X
    float accel_y;       // 加速度 Y
    float accel_z;       // 加速度 Z
    float gyro_x;        // 陀螺仪 X
    float gyro_y;        // 陀螺仪 Y
    float gyro_z;        // 陀螺仪 Z
    float temperature;   // 温度
    uint32_t timestamp;  // 时间戳
    uint8_t status;      // 状态位
    uint8_t reserved[3]; // 保留位
} SensorData;

// 协议帧结构
typedef struct
{
    uint16_t magic;  // 魔数 0xAA55
    uint8_t version; // 协议版本
    uint8_t type;    // 数据类型
    uint16_t length; // 数据长度
    uint32_t seq;    // 序列号
    SensorData data; // 传感器数据
    uint16_t crc16;  // CRC16校验
} ProtocolFrame;
#pragma pack(pop)

// 帧大小
#define FRAME_SIZE sizeof(ProtocolFrame)

// CRC16计算 (Modbus)
uint16_t calculate_crc16(const uint8_t *data, size_t length);

// 验证帧
int validate_frame(const ProtocolFrame *frame);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_COMMON_H