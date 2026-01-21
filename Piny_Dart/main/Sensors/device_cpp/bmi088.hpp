#pragma once

#include "spi_bus.hpp"
#include <array>
#include <cstdint>
#include <string>

// ========================== BMI088寄存器定义 ==========================
// 加速度计寄存器
#define ACC_CHIP_ID 0x00    // 芯片ID寄存器（默认0x1E）
#define ACC_STATUS 0x03     // 数据就绪状态寄存器
#define ACC_DATA_X_LSB 0x12 // X轴加速度数据低字节
#define ACC_DATA_X_MSB 0x13 // X轴加速度数据高字节
#define ACC_DATA_Y_LSB 0x14 // Y轴加速度数据低字节
#define ACC_DATA_Y_MSB 0x15 // Y轴加速度数据高字节
#define ACC_DATA_Z_LSB 0x16 // Z轴加速度数据低字节
#define ACC_DATA_Z_MSB 0x17 // Z轴加速度数据高字节
#define ACC_CONF 0x40       // 输出速率配置寄存器
#define ACC_RANGE 0x41      // 量程配置寄存器
#define ACC_PWR_CONF 0x7C   // 电源配置寄存器
#define ACC_PWR_CTRL 0x7D   // 电源控制寄存器
#define ACC_SOFT_RESET 0x7E // 软复位寄存器
#define ACC_INT_EN_1 0x1A   // 中断使能寄存器
#define ACC_IF_CONF 0x1B    // 接口配置寄存器

// 陀螺仪寄存器
#define GYRO_CHIP_ID 0x00    // 芯片ID寄存器（默认0x0F）
#define GYRO_RATE_X_LSB 0x02 // X轴角速度数据低字节
#define GYRO_RATE_X_MSB 0x03 // X轴角速度数据高字节
#define GYRO_RATE_Y_LSB 0x04 // Y轴角速度数据低字节
#define GYRO_RATE_Y_MSB 0x05 // Y轴角速度数据高字节
#define GYRO_RATE_Z_LSB 0x06 // Z轴角速度数据低字节
#define GYRO_RATE_Z_MSB 0x07 // Z轴角速度数据高字节
#define GYRO_BANDWITH 0x10   // 带宽配置寄存器
#define GYRO_RANGE 0x0F      // 量程配置寄存器

namespace sensor
{

// BMI088传感器类
class BMI088
{
public:
    // 传感器数据结构体
    struct Data
    {
        int16_t acc_x{0};
        int16_t acc_y{0};
        int16_t acc_z{0};
        int16_t gyro_x{0};
        int16_t gyro_y{0};
        int16_t gyro_z{0};

        // 转换为物理单位
        float acc_x_g() const
        {
            return acc_x * 0.000088f;
        }
        float acc_y_g() const
        {
            return acc_y * 0.000088f;
        }
        float acc_z_g() const
        {
            return acc_z * 0.000088f;
        }
        float gyro_x_dps() const
        {
            return gyro_x * 0.0152588f;
        }
        float gyro_y_dps() const
        {
            return gyro_y * 0.0152588f;
        }
        float gyro_z_dps() const
        {
            return gyro_z * 0.0152588f;
        }

        // 格式化输出
        std::string to_string() const;
    };

    // 构造函数（传入SPI设备配置）
    explicit BMI088(const spi::DeviceConfig &dev_cfg_acc, const spi::DeviceConfig &dev_cfg_gyro);

    // 禁用拷贝/移动
    BMI088(const BMI088 &) = delete;
    BMI088 &operator=(const BMI088 &) = delete;
    BMI088(BMI088 &&) = delete;
    BMI088 &operator=(BMI088 &&) = delete;

    // 初始化传感器
    esp_err_t init();

    // 读取传感器数据
    esp_err_t read_data(Data &data);

    // 析构释放设备
    ~BMI088();

private:
    // SPI总线实例（全局唯一）
    spi::SPIBus &spi_bus_;
    // 加速度计/陀螺仪设备句柄
    spi_device_handle_t acc_handle_ = nullptr;
    spi_device_handle_t gyro_handle_ = nullptr;

    // 加速度计上次有效数据
    Data last_acc_data_{};

    // 内部初始化函数
    esp_err_t init_accelerometer();
    esp_err_t init_gyroscope();

    // 内部数据读取函数
    esp_err_t read_accelerometer(Data &data);
    esp_err_t read_gyroscope(Data &data);
};

} // namespace sensor
