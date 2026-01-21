#ifndef BMI088_H_
#define BMI088_H_

#include <stdio.h>

// ========================== 硬件配置 ==========================
#define BMI088_HOST SPI2_HOST
#define BMI088_CLK 13     // SPI时钟引脚
#define BMI088_MOSI 12    // SPI主机输出引脚
#define BMI088_MISO 11    // SPI主机输入引脚
#define BMI088_CS_GYRO 10 // 陀螺仪片选引脚
#define BMI088_CS_ACCEL 9 // 加速度计片选引脚

// ========================== SPI配置 ==========================
#define SPI_CLOCK_SPEED_HZ 10 * 1000 * 1000 // SPI时钟频率（5MHz）

// ========================== BMI088寄存器定义 ==========================
// 加速度计寄存器
#define BMI088_ACC_CHIP_ID 0x00    // 芯片ID寄存器（默认0x1E）
#define BMI088_ACC_STATUS 0x03     // 数据就绪状态寄存器
#define BMI088_ACC_DATA_X_LSB 0x12 // X轴加速度数据低字节
#define BMI088_ACC_DATA_X_MSB 0x13 // X轴加速度数据高字节
#define BMI088_ACC_DATA_Y_LSB 0x14 // Y轴加速度数据低字节
#define BMI088_ACC_DATA_Y_MSB 0x15 // Y轴加速度数据高字节
#define BMI088_ACC_DATA_Z_LSB 0x16 // Z轴加速度数据低字节
#define BMI088_ACC_DATA_Z_MSB 0x17 // Z轴加速度数据高字节
#define BMI088_ACC_CONF 0x40       // 输出速率配置寄存器
#define BMI088_ACC_RANGE 0x41      // 量程配置寄存器
#define BMI088_ACC_PWR_CONF 0x7C   // 电源配置寄存器
#define BMI088_ACC_PWR_CTRL 0x7D   // 电源控制寄存器
#define BMI088_ACC_SOFT_RESET 0x7E // 软复位寄存器
#define BMI088_ACC_INT_EN_1 0x1A   // 中断使能寄存器
#define BMI088_ACC_IF_CONF 0x1B    // 接口配置寄存器

// 陀螺仪寄存器
#define BMI088_GYRO_CHIP_ID 0x00    // 芯片ID寄存器（默认0x0F）
#define BMI088_GYRO_RATE_X_LSB 0x02 // X轴角速度数据低字节
#define BMI088_GYRO_RATE_X_MSB 0x03 // X轴角速度数据高字节
#define BMI088_GYRO_RATE_Y_LSB 0x04 // Y轴角速度数据低字节
#define BMI088_GYRO_RATE_Y_MSB 0x05 // Y轴角速度数据高字节
#define BMI088_GYRO_RATE_Z_LSB 0x06 // Z轴角速度数据低字节
#define BMI088_GYRO_RATE_Z_MSB 0x07 // Z轴角速度数据高字节
#define BMI088_GYRO_BANDWITH 0x10   // 带宽配置寄存器
#define BMI088_GYRO_RANGE 0x0F      // 量程配置寄存器

// ========================== 数据结构定义 ==========================
/**
 * @brief BMI088原始数据结构
 */
typedef struct
{
    int16_t acc_x;  // X轴加速度原始值
    int16_t acc_y;  // Y轴加速度原始值
    int16_t acc_z;  // Z轴加速度原始值
    int16_t gyro_x; // X轴角速度原始值
    int16_t gyro_y; // Y轴角速度原始值
    int16_t gyro_z; // Z轴角速度原始值
} bmi088_data_t;

void bmi088_task(void *arg);

#endif // BMI088_H_
