#ifndef MS5611_HPP
#define MS5611_HPP

#include "i2c_bus.hpp"
#include <array>
#include <cstdint>
#include <functional>

namespace sensor
{

// MS5611默认地址
constexpr uint8_t MS5611_ADDR = 0x77;
constexpr uint8_t MS5611_ADDR_ALT = 0x76;

// MS5611命令
constexpr uint8_t MS5611_CMD_RESET = 0x1E;
constexpr uint8_t MS5611_CMD_PROM_READ = 0xA2;
constexpr uint8_t MS5611_CMD_ADC_READ = 0x00;

// 转换命令（不同的过采样率）
constexpr uint8_t MS5611_CMD_CONVERT_D1_256 = 0x40;
constexpr uint8_t MS5611_CMD_CONVERT_D1_512 = 0x42;
constexpr uint8_t MS5611_CMD_CONVERT_D1_1024 = 0x44;
constexpr uint8_t MS5611_CMD_CONVERT_D1_2048 = 0x46;
constexpr uint8_t MS5611_CMD_CONVERT_D1_4096 = 0x48;

constexpr uint8_t MS5611_CMD_CONVERT_D2_256 = 0x50;
constexpr uint8_t MS5611_CMD_CONVERT_D2_512 = 0x52;
constexpr uint8_t MS5611_CMD_CONVERT_D2_1024 = 0x54;
constexpr uint8_t MS5611_CMD_CONVERT_D2_2048 = 0x56;
constexpr uint8_t MS5611_CMD_CONVERT_D2_4096 = 0x58;

// OSR枚举
enum class OSR : uint8_t
{
    OSR_256 = 0,
    OSR_512 = 1,
    OSR_1024 = 2,
    OSR_2048 = 3,
    OSR_4096 = 4
};

class MS5611
{
public:
    // 数据结果结构体
    struct Data
    {
        double temperature = 0.0;   // 温度(℃)
        double pressure_mbar = 0.0; // 压力(mbar)
        double pressure_pa = 0.0;   // 压力(Pa)
        uint32_t raw_d1 = 0;        // 原始压力ADC值
        uint32_t raw_d2 = 0;        // 原始温度ADC值

        std::string to_string() const
        {
            char buffer[128];
            snprintf(buffer,
                     sizeof(buffer),
                     "Temperature: %.2f°C | Pressure: %.2f mbar (%.0f Pa)",
                     temperature,
                     pressure_mbar,
                     pressure_pa);
            return std::string(buffer);
        }
    };

    // 构造函数
    MS5611(i2c::I2CBus &i2c_bus, uint8_t device_address = MS5611_ADDR, OSR osr = OSR::OSR_2048);

    // 初始化传感器
    esp_err_t init();

    // 读取传感器数据
    esp_err_t read_data(Data &data);

    // 读取原始D1/D2值
    esp_err_t read_raw_data(uint32_t &d1, uint32_t &d2);

    // 设置OSR（过采样率）
    void set_osr(OSR osr);

    // 获取校准系数
    const std::array<uint16_t, 6> &get_calibration_coeffs() const
    {
        return calib_coeffs_;
    }

    // 验证校准系数
    bool is_calibration_valid() const;

    // 软复位传感器
    esp_err_t reset();

    // 获取设备地址
    uint8_t get_device_address() const
    {
        return device_address_;
    }

    ~MS5611();

private:
    // 读取PROM校准系数
    esp_err_t read_calibration_coeffs();

    // 读取ADC值
    esp_err_t read_reg(uint8_t convert_cmd, uint32_t &adc_data);

    // 发送命令
    esp_err_t send_command(uint8_t cmd);

    // 转换原始数据
    void convert_raw_data(uint32_t d1, uint32_t d2, Data &data);

    // 成员变量
    i2c::I2CBus &i2c_bus_;
    i2c_master_dev_handle_t device_handle_ = nullptr;
    uint8_t device_address_;
    OSR current_osr_;

    // 校准系数数组
    std::array<uint16_t, 6> calib_coeffs_ = {0};
    bool is_initialized_ = false;
    bool calibration_loaded_ = false;

    // OSR对应的转换命令
    std::array<uint8_t, 5> d1_convert_cmds_ = {MS5611_CMD_CONVERT_D1_256,
                                               MS5611_CMD_CONVERT_D1_512,
                                               MS5611_CMD_CONVERT_D1_1024,
                                               MS5611_CMD_CONVERT_D1_2048,
                                               MS5611_CMD_CONVERT_D1_4096};

    std::array<uint8_t, 5> d2_convert_cmds_ = {MS5611_CMD_CONVERT_D2_256,
                                               MS5611_CMD_CONVERT_D2_512,
                                               MS5611_CMD_CONVERT_D2_1024,
                                               MS5611_CMD_CONVERT_D2_2048,
                                               MS5611_CMD_CONVERT_D2_4096};

    // OSR对应的延迟（单位：ms）
    std::array<uint32_t, 5> osr_delays_ = {3, 3, 3, 5, 10};
};

} // namespace sensor

#endif // MS5611_HPP