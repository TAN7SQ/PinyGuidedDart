#ifndef I2C_BUS_HPP
#define I2C_BUS_HPP

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace i2c
{

// I2C总线配置结构体
struct BusConfig
{
    i2c_port_t port = I2C_NUM_0;        // I2C端口号
    gpio_num_t scl_pin = GPIO_NUM_20;   // SCL引脚
    gpio_num_t sda_pin = GPIO_NUM_21;   // SDA引脚
    uint32_t clk_speed_hz = 400000;     // 时钟频率
    bool enable_internal_pullup = true; // 启用内部上拉
    uint8_t glitch_ignore_cnt = 7;      // 毛刺忽略计数
};

// I2C设备配置结构体
struct DeviceConfig
{
    uint8_t device_address;                           // 7位设备地址
    uint32_t scl_speed_hz = 400000;                   // 设备时钟频率
    i2c_addr_bit_len_t addr_len = I2C_ADDR_BIT_LEN_7; // 地址长度
};

class I2CBus
{
public:
    // 单例模式 - 获取总线实例
    static I2CBus &get_instance(const BusConfig &config = default_config())
    {
        static I2CBus instance(config);
        return instance;
    }

    // 禁用拷贝和移动
    I2CBus(const I2CBus &) = delete;
    I2CBus &operator=(const I2CBus &) = delete;
    I2CBus(I2CBus &&) = delete;
    I2CBus &operator=(I2CBus &&) = delete;

    // 添加I2C设备
    esp_err_t add_device(const DeviceConfig &dev_cfg, i2c_master_dev_handle_t &dev_handle);

    // 移除I2C设备
    esp_err_t remove_device(i2c_master_dev_handle_t dev_handle);

    // 扫描I2C总线上的设备
    std::vector<uint8_t> scan_devices(uint8_t start_addr = 0x08, uint8_t end_addr = 0x77);

    // 检查特定地址是否有设备
    bool probe_device(uint8_t device_address);

    // 获取总线句柄
    i2c_master_bus_handle_t get_bus_handle() const
    {
        return bus_handle_;
    }

    // 判断总线是否初始化
    bool is_initialized() const
    {
        return is_initialized_;
    }

    ~I2CBus();

private:
    // 显式构造函数
    explicit I2CBus(const BusConfig &config = default_config());
    explicit I2CBus(i2c_port_t port, gpio_num_t scl_pin, gpio_num_t sda_pin, uint32_t clk_speed_hz = 400000);

    // 初始化总线
    esp_err_t init_bus(const BusConfig &config);

    // 默认配置
    static BusConfig default_config()
    {
        return {
            .port = I2C_NUM_0,
            .scl_pin = GPIO_NUM_20,
            .sda_pin = GPIO_NUM_21,
            .clk_speed_hz = 400000,
            .enable_internal_pullup = true,
        };
    }

    // 成员变量
    i2c_master_bus_handle_t bus_handle_ = nullptr;
    BusConfig bus_config_;
    bool is_initialized_ = false;
};

} // namespace i2c

#endif // I2C_BUS_HPP