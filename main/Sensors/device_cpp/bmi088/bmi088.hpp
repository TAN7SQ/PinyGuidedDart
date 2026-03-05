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

// //-----------------------------------------------
// #define BMI088_ACC_BWP_SHFITS 0x4
// #define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)

// #define BMI088_ACC_ODR_SHFITS 0x0
// #define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
// #define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)
// #define BMI088_ACC_CONF_MUST_Set 0x80

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

namespace sensor
{

// BMI088传感器类
class BMI088
{
public:
#define GYRO_FS_2000DPS 2000.0f
// 1g对应的重力加速度（工程常用）
#define GRAVITY 9.81f
// 16位ADC最大有效值（加速度计/陀螺仪通用）
#define ADC_MAX_VAL 32767.0f
#define DEG2RAD 0.0174533f

    enum class AccRange
    {
        G3 = 0,
        G6 = 1,
        G12 = 2,
        G24 = 3,
    };

    static constexpr float acc_lsb_per_g[] = {10920.0f, 5460.0f, 2730.0f, 1365.0f};

    static constexpr float ACC_SENS_24G = acc_lsb_per_g[static_cast<int>(AccRange::G24)];

    static constexpr float gyro_scale_2000dps = GYRO_FS_2000DPS / ADC_MAX_VAL;

    struct Data
    {
        int16_t acc_x{0};
        int16_t acc_y{0};
        int16_t acc_z{0};
        int16_t gyro_x{0};
        int16_t gyro_y{0};
        int16_t gyro_z{0};

        float acc_x_g() const
        {
            return acc_x / ACC_SENS_24G;
        }
        float acc_y_g() const
        {
            return acc_y / ACC_SENS_24G;
        }
        float acc_z_g() const
        {
            return acc_z / ACC_SENS_24G;
        }

        // 陀螺仪数据转换为°/s单位
        float gyro_x_dps() const
        {
            return gyro_x * gyro_scale_2000dps;
        }
        float gyro_y_dps() const
        {
            return gyro_y * gyro_scale_2000dps;
        }
        float gyro_z_dps() const
        {
            return gyro_z * gyro_scale_2000dps;
        }

        float gyro_x_rads() const
        {
            return gyro_x_dps() * DEG2RAD;
        }
        float gyro_y_rads() const
        {
            return gyro_y_dps() * DEG2RAD;
        }
        float gyro_z_rads() const
        {
            return gyro_z_dps() * DEG2RAD;
        }

        std::string to_string() const;
    };

    explicit BMI088(const spi::DeviceConfig &dev_cfg_acc, const spi::DeviceConfig &dev_cfg_gyro);

    BMI088(const BMI088 &) = delete;
    ~BMI088();
    BMI088 &operator=(const BMI088 &) = delete;
    BMI088(BMI088 &&) = delete;
    BMI088 &operator=(BMI088 &&) = delete;

    esp_err_t init();
    esp_err_t read_data(Data &data);
    esp_err_t calibrate(sensor::BMI088::Data data);

private:
    spi::SPIBus &spi_bus_;
    spi_device_handle_t acc_handle_ = nullptr;
    spi_device_handle_t gyro_handle_ = nullptr;

    Data last_acc_data_{};
    float _accel_bias[3] = {0.0f};
    float _gyro_bias[3] = {0.0f};

    esp_err_t ensure_acc_range_24g();
    esp_err_t init_accelerometer();
    esp_err_t init_gyroscope();

    esp_err_t read_accelerometer(Data &data);
    esp_err_t read_gyroscope(Data &data);
};

} // namespace sensor
