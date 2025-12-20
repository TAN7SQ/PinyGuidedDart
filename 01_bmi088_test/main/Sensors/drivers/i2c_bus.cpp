#include "i2c_bus.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>

static const char *TAG = "I2CBus";

namespace i2c
{

// 构造函数
I2CBus::I2CBus(const BusConfig &config)
{
    if (init_bus(config) != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus initialization failed");
    }
}

I2CBus::I2CBus(i2c_port_t port, gpio_num_t scl_pin, gpio_num_t sda_pin, uint32_t clk_speed_hz)
    : I2CBus(BusConfig{.port = port,
                       .scl_pin = scl_pin,
                       .sda_pin = sda_pin,
                       .clk_speed_hz = clk_speed_hz,
                       .enable_internal_pullup = true,
                       .glitch_ignore_cnt = 7})
{
}

// 初始化I2C总线
esp_err_t I2CBus::init_bus(const BusConfig &config)
{
    bus_config_ = config;

    i2c_master_bus_config_t bus_cfg = {.i2c_port = config.port,
                                       .sda_io_num = config.sda_pin,
                                       .scl_io_num = config.scl_pin,
                                       .clk_source = I2C_CLK_SRC_DEFAULT,
                                       .glitch_ignore_cnt = config.glitch_ignore_cnt,
                                       .flags = {.enable_internal_pullup = config.enable_internal_pullup}};

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &bus_handle_);

    if (ret == ESP_OK) {
        is_initialized_ = true;
        ESP_LOGI(TAG,
                 "I2C bus initialized successfully (port: %d, SCL: %d, SDA: %d, speed: %ld Hz)",
                 config.port,
                 config.scl_pin,
                 config.sda_pin,
                 config.clk_speed_hz);
    }
    else {
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s", esp_err_to_name(ret));
    }

    return ret;
}

// 添加I2C设备
esp_err_t I2CBus::add_device(const DeviceConfig &dev_cfg, i2c_master_dev_handle_t &dev_handle)
{
    if (!is_initialized_) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_device_config_t device_cfg = {
        .dev_addr_length = dev_cfg.addr_len,
        .device_address = dev_cfg.device_address,
        .scl_speed_hz = dev_cfg.scl_speed_hz,
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle_, &device_cfg, &dev_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Added I2C device at address 0x%02X", dev_cfg.device_address);
    }
    else {
        ESP_LOGE(TAG, "Failed to add I2C device at address 0x%02X: %s", dev_cfg.device_address, esp_err_to_name(ret));
    }

    return ret;
}

// 移除I2C设备
esp_err_t I2CBus::remove_device(i2c_master_dev_handle_t dev_handle)
{
    if (!dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_bus_rm_device(dev_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Removed I2C device");
    }
    else {
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(ret));
    }

    return ret;
}

// 扫描I2C设备
std::vector<uint8_t> I2CBus::scan_devices(uint8_t start_addr, uint8_t end_addr)
{
    std::vector<uint8_t> found_devices;

    if (!is_initialized_) {
        ESP_LOGE(TAG, "I2C bus not initialized for scanning");
        return found_devices;
    }

    ESP_LOGI(TAG, "Scanning I2C bus from address 0x%02X to 0x%02X", start_addr, end_addr);

    for (uint8_t addr = start_addr; addr <= end_addr; addr++) {
        if (probe_device(addr)) {
            found_devices.push_back(addr);
            ESP_LOGI(TAG, "Found device at address 0x%02X", addr);
        }

        // 每扫描8个地址短暂延时，避免总线过载
        if (addr % 8 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    ESP_LOGI(TAG, "Scan complete, found %zu device(s)", found_devices.size());
    return found_devices;
}

// 探测特定地址是否有设备
bool I2CBus::probe_device(uint8_t device_address)
{
    if (!is_initialized_) {
        return false;
    }

    // 创建临时设备进行探测
    i2c_master_dev_handle_t temp_handle = nullptr;
    i2c_device_config_t temp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = 100000, // 使用较低的频率进行探测
    };

    esp_err_t ret = i2c_master_bus_add_device(bus_handle_, &temp_cfg, &temp_handle);

    if (ret != ESP_OK) {
        return false;
    }

    // 发送空数据包探测设备
    uint8_t dummy = 0;
    ret = i2c_master_transmit(temp_handle, &dummy, 1, 10);

    // 移除临时设备
    i2c_master_bus_rm_device(temp_handle);

    return ret == ESP_OK;
}

// 析构函数
I2CBus::~I2CBus()
{
    if (bus_handle_) {
        esp_err_t ret = i2c_del_master_bus(bus_handle_);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C bus deleted");
        }
        else {
            ESP_LOGE(TAG, "Failed to delete I2C bus: %s", esp_err_to_name(ret));
        }
        bus_handle_ = nullptr;
    }
    is_initialized_ = false;
}

} // namespace i2c