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
    // 获取单例实例
    static I2CBus &get_instance(const BusConfig &config = default_config());

    // 初始化总线（必须在第一次使用前调用）
    esp_err_t init(const BusConfig &config = default_config());

    // 使用参数初始化
    esp_err_t init(i2c_port_t port, int scl_pin, int sda_pin, uint32_t clk_speed_hz = 400000, bool enable_pullup = true,
                   uint8_t glitch_ignore_cnt = 7)
    {
        BusConfig config{.port = port,
                         .scl_pin = static_cast<gpio_num_t>(scl_pin),
                         .sda_pin = static_cast<gpio_num_t>(sda_pin),
                         .clk_speed_hz = clk_speed_hz,
                         .enable_internal_pullup = enable_pullup,
                         .glitch_ignore_cnt = glitch_ignore_cnt};
        return init(config);
    }

    // 禁用拷贝和移动
    I2CBus(const I2CBus &) = delete;
    I2CBus &operator=(const I2CBus &) = delete;
    I2CBus(I2CBus &&) = delete;
    I2CBus &operator=(I2CBus &&) = delete;

    // === 设备管理 ===
    esp_err_t add_device(const DeviceConfig &dev_cfg, i2c_master_dev_handle_t &dev_handle);
    esp_err_t remove_device(i2c_master_dev_handle_t dev_handle);

    // === I2C传输封装 ===

    // 发送数据（写入）
    esp_err_t write_bytes(i2c_master_dev_handle_t dev_handle, const uint8_t *data, size_t data_len,
                          uint32_t timeout_ms = 100);

    // 接收数据（读取）
    esp_err_t read_bytes(i2c_master_dev_handle_t dev_handle, uint8_t *buffer, size_t buffer_len,
                         uint32_t timeout_ms = 100);

    // 发送数据然后接收数据（先写后读）
    esp_err_t write_then_read(i2c_master_dev_handle_t dev_handle, const uint8_t *write_data, size_t write_len,
                              uint8_t *read_buffer, size_t read_len, uint32_t timeout_ms = 100);

    // 写入单个字节（寄存器写入）
    esp_err_t write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data, uint32_t timeout_ms = 100);

    // 读取单个字节（寄存器读取）
    esp_err_t read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t &data, uint32_t timeout_ms = 100);

    // 读取多个字节（连续寄存器读取）
    esp_err_t read_regs(i2c_master_dev_handle_t dev_handle, uint8_t start_reg, uint8_t *buffer, size_t len,
                        uint32_t timeout_ms = 100);

    // === 总线操作 ===
    std::vector<uint8_t> scan_devices(uint8_t start_addr = 0x08, uint8_t end_addr = 0x77);
    bool probe_device(uint8_t device_address);

    // === 获取信息 ===
    i2c_master_bus_handle_t get_bus_handle() const
    {
        return bus_handle_;
    }
    bool is_initialized() const
    {
        return is_initialized_;
    }

    ~I2CBus();

private:
    // 私有构造函数
    explicit I2CBus(const BusConfig &config);

    // 初始化总线内部实现
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
            .glitch_ignore_cnt = 7,
        };
    }

    // 成员变量
    i2c_master_bus_handle_t bus_handle_ = nullptr;
    BusConfig bus_config_;
    bool is_initialized_ = false;
};

} // namespace i2c

#endif // I2C_BUS_HPP