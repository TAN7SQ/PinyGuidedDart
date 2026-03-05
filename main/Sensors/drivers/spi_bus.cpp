#include "spi_bus.hpp"
#include "esp_log.h"
#include "freertos/task.h"
#include <memory>

static const char *TAG = "SPIBus";

namespace spi
{

SPIBus &SPIBus::get_instance(const BusConfig &config)
{
    static SPIBus instance(config); // 唯一实例
    return instance;
}

SPIBus::SPIBus(const BusConfig &config) : bus_config_(config), host_(config.host_num)
{
    if (init_bus(config) != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed!");
    }
}

esp_err_t SPIBus::init_bus(const BusConfig &config)
{
    // 创建总线互斥锁
    bus_mutex_ = xSemaphoreCreateMutex();
    if (!bus_mutex_) {
        ESP_LOGE(TAG, "Create SPI mutex failed");
        return ESP_FAIL;
    }

    // 配置SPI总线参数
    spi_bus_config_t spi_bus_cfg = {
        .mosi_io_num = config.mosi_pin,
        .miso_io_num = config.miso_pin,
        .sclk_io_num = config.sclk_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = config.max_transfer_sz,
    };

    ESP_LOGI("SPI",
             "Default SPI bus config: host_num=%d,sclk=%d, mosi=%d, miso=%d",
             config.host_num,
             config.sclk_pin,
             config.mosi_pin,
             config.miso_pin);

    esp_err_t ret = spi_bus_initialize(config.host_num, &spi_bus_cfg, config.dma_chan);
    if (ret == ESP_OK) {
        is_bus_init_ = true;
        ESP_LOGI(TAG, "SPI bus init success (host: %d)", config.host_num);
    }
    else {
        ESP_LOGE(TAG, "SPI bus init failed (host: %d), err: %d", config.host_num, ret);
    }
    return ret;
}

esp_err_t SPIBus::add_device(const DeviceConfig &dev_cfg, spi_device_handle_t &dev_handle)
{
    if (!is_bus_init_) {
        ESP_LOGE(TAG, "SPI bus not init!");
        return ESP_ERR_INVALID_STATE;
    }

    // 配置设备参数
    spi_device_interface_config_t spi_dev_cfg = {
        .mode = dev_cfg.mode,
        .cs_ena_pretrans = dev_cfg.cs_ena_pretrans,
        .cs_ena_posttrans = dev_cfg.cs_ena_posttrans,
        .clock_speed_hz = dev_cfg.clock_speed_hz,
        .spics_io_num = dev_cfg.cs_pin,
        .flags = 0, // 全双工模式
        .queue_size = dev_cfg.queue_size,
    };

    // 添加设备到总线
    esp_err_t ret = spi_bus_add_device(host_, &spi_dev_cfg, &dev_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Add SPI device success (CS: %d)", dev_cfg.cs_pin);
    }
    else {
        ESP_LOGE(TAG, "Add SPI device failed (CS: %d), err: %d", dev_cfg.cs_pin, ret);
    }
    return ret;
}

esp_err_t SPIBus::remove_device(spi_device_handle_t dev_handle)
{
    if (!dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = spi_bus_remove_device(dev_handle);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Remove SPI device success");
    }
    else {
        ESP_LOGE(TAG, "Remove SPI device failed, err: %d", ret);
    }
    return ret;
}

esp_err_t SPIBus::write_reg(spi_device_handle_t dev_handle, uint8_t reg, uint8_t data)
{
    if (!dev_handle || !is_bus_init_) {
        return ESP_ERR_INVALID_ARG;
    }

    // 申请总线锁
    if (xSemaphoreTake(bus_mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Take SPI mutex failed (write)");
        return ESP_ERR_TIMEOUT;
    }

    // RAII自动释放锁，需要的参数类型是void*
    // auto mutex_guard = [this](void *) {
    //     xSemaphoreGive(bus_mutex_);
    // };
    // std::unique_ptr<void, decltype(mutex_guard)> guard(nullptr, mutex_guard);
    // std::unique_ptr<void,                  // 管理的指针类型是void
    //                 decltype(mutex_guard)> // 删除器的类型，自动推导lambda的类型
    //     guard(nullptr,                     // 设置 unique_ptr管理的原始指针为nullptr，表示当前没有管理任何实际资源
    //                                        //  -- 但这里好像会有问题，可能导致无法正常释放互斥锁，应该使用非空占位符
    //           mutex_guard);                // 设置删除器为mutex_guard
    // 函数结束时，guard自动析构 -> 调用删除器 -> 释放互斥锁
    // 即使发生异常也会确保释放
    // std::unique_ptr<void, decltype(mutex_guard)> guard(reinterpret_cast<void *>(1), // 非空！
    //                                                    mutex_guard);

    // 构造写指令（写操作最高位为0）
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg & 0x7F), data};
    uint8_t rx_buf[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        // .user = nullptr,
    };

    esp_err_t ret = spi_device_transmit(dev_handle, &t);
    xSemaphoreGive(bus_mutex_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write reg 0x%02X failed, err: %d", reg, ret);
    }
    return ret;
}

esp_err_t SPIBus::read_reg(spi_device_handle_t dev_handle, uint8_t reg, uint8_t &data)
{
    if (!dev_handle || !is_bus_init_) {
        return ESP_ERR_INVALID_ARG;
    }

    // 申请总线锁
    if (xSemaphoreTake(bus_mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Take SPI mutex failed (read)");
        data = 0;
        return ESP_ERR_TIMEOUT;
    }

    // RAII自动释放锁
    // auto mutex_guard = [this](void *) {
    //     xSemaphoreGive(bus_mutex_);
    // };
    // std::unique_ptr<void, decltype(mutex_guard)> guard(nullptr, mutex_guard);
    // std::unique_ptr<void, decltype(mutex_guard)> guard(reinterpret_cast<void *>(1), // 非空！
    //                                                    mutex_guard);

    // 构造读指令（读操作最高位为1）
    uint8_t tx_buf[2] = {static_cast<uint8_t>(reg | 0x80), 0x00};
    uint8_t rx_buf[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        // .user = nullptr,
    };

    esp_err_t ret = spi_device_transmit(dev_handle, &t);
    xSemaphoreGive(bus_mutex_);
    if (ret == ESP_OK) {
        data = rx_buf[1]; // 第二个字节是有效数据
    }
    else {
        ESP_LOGE(TAG, "SPI read reg 0x%02X failed, err: %d", reg, ret);
        data = 0;
    }
    return ret;
}

esp_err_t SPIBus::read_regs(spi_device_handle_t dev_handle, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (!dev_handle || !data || len == 0 || !is_bus_init_) {
        return ESP_ERR_INVALID_ARG;
    }

    // 申请总线锁
    if (xSemaphoreTake(bus_mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Take SPI mutex failed (read regs)");
        return ESP_ERR_TIMEOUT;
    }

    // RAII自动释放锁
    // auto mutex_guard = [this](void *) {
    //     xSemaphoreGive(bus_mutex_);
    // };
    // std::unique_ptr<void, decltype(mutex_guard)> guard(nullptr, mutex_guard);

    // 申请缓冲区（1字节地址 + len字节数据）
    // std::unique_ptr会在函数结束时自动释放内存
    // std::unique_ptr<uint8_t[]> tx_buf(new (std::nothrow) uint8_t[len + 1]);
    // std::unique_ptr<uint8_t[]> rx_buf(new (std::nothrow) uint8_t[len + 1]);

    uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
    uint8_t *rx_buf = (uint8_t *)malloc(len + 1);

    if (!tx_buf || !rx_buf) {
        xSemaphoreGive(bus_mutex_);
        ESP_LOGE(TAG, "Malloc failed for SPI read regs");
        return ESP_ERR_NO_MEM;
    }

    // 填充读指令
    tx_buf[0] = reg | 0x80;
    std::memset(&tx_buf[1], 0, len);

    // 全双工传输
    spi_transaction_t t = {
        .length = (size_t)((len + 1) * 8), // 总位数
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        // .tx_buffer = tx_buf.get(),
        // .rx_buffer = rx_buf.get(),

    };

    esp_err_t ret = spi_device_transmit(dev_handle, &t);

    if (ret == ESP_OK) {
        std::memcpy(data, &rx_buf[1], len); // 跳过地址字节
    }
    else {
        ESP_LOGE(TAG, "SPI read regs 0x%02X (len: %d) failed, err: %d", reg, len, ret);
    }
    xSemaphoreGive(bus_mutex_);

    // 释放缓冲区
    free(tx_buf);
    free(rx_buf);

    return ret;
}

SPIBus::~SPIBus()
{
    if (bus_mutex_) {
        vSemaphoreDelete(bus_mutex_);
        bus_mutex_ = nullptr;
    }

    if (is_bus_init_) {
        spi_bus_free(host_);
        is_bus_init_ = false;
        ESP_LOGI(TAG, "SPI bus freed (host: %d)", host_);
    }
}

} // namespace spi