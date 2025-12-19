#ifndef SPI_BUS_HPP
#define SPI_BUS_HPP

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include <cstdint>
#include <cstring>
#include <memory>

namespace spi
{

struct BusConfig
{
    spi_host_device_t host_num;                // SPI主机（如SPI2_HOST）
    gpio_num_t sclk_pin;                       // 时钟引脚
    gpio_num_t mosi_pin;                       // MOSI引脚
    gpio_num_t miso_pin;                       // MISO引脚
    uint16_t max_transfer_sz = 32;             // 最大传输字节数
    spi_dma_chan_t dma_chan = SPI_DMA_CH_AUTO; // DMA通道
};

struct DeviceConfig
{
    uint16_t clock_speed_hz;
    gpio_num_t cs_pin;
    uint8_t mode = 3;
    uint8_t cs_ena_pretrans = 4;
    uint8_t cs_ena_posttrans = 8;
    uint8_t queue_size = 5;
};

class SPIBus
{
public:
    // 获取单例
    static SPIBus &get_instance(const BusConfig &config = default_bus_config());

    // 禁用拷贝/移动
    SPIBus(const SPIBus &) = delete;
    SPIBus &operator=(const SPIBus &) = delete;
    SPIBus(SPIBus &&) = delete;
    SPIBus &operator=(SPIBus &&) = delete;

    esp_err_t add_device(const spi::DeviceConfig &dev_cfg, spi_device_handle_t &dev_handle);
    esp_err_t remove_device(spi_device_handle_t dev_handle);

    esp_err_t write_reg(spi_device_handle_t dev_handle, uint8_t reg, uint8_t data);
    esp_err_t read_reg(spi_device_handle_t dev_handle, uint8_t reg, uint8_t &data);
    esp_err_t read_regs(spi_device_handle_t dev_handle, uint8_t reg, uint8_t len, uint8_t *data);

    ~SPIBus();

    static BusConfig default_bus_config()
    {
        return {.host_num = SPI2_HOST,
                .sclk_pin = GPIO_NUM_13,
                .mosi_pin = GPIO_NUM_12,
                .miso_pin = GPIO_NUM_11,
                .max_transfer_sz = 32,
                .dma_chan = SPI_DMA_CH_AUTO};
    }

private:
    // explicit 不能自动使用这个构造函数进行隐式转换，只能显式调用
    explicit SPIBus(const BusConfig &config);

    esp_err_t init_bus(const BusConfig &config);

    BusConfig bus_config_;

    spi_host_device_t host_;

    // 总线互斥锁（保证多设备访问安全）
    SemaphoreHandle_t bus_mutex_ = nullptr;
    bool is_bus_init_ = false;
};

} // namespace spi

#endif // SPI_BUS_HPP