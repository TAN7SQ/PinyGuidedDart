#pragma once

#include "esp_err.h"
#include "spi_bus.hpp"
#include <stdint.h>

namespace sensor
{

class ADXL375
{
public:
    static constexpr const char *TAG = "ADXL375";

    struct Data
    {
        int16_t x;
        int16_t y;
        int16_t z;
    };

public:
    ADXL375(const spi::DeviceConfig &dev_cfg);
    ~ADXL375();

    esp_err_t init();
    esp_err_t read_device_id(uint8_t &id);

private:
    spi::SPIBus &spi_bus_;
    spi_device_handle_t dev_handle_ = nullptr;

    esp_err_t check_device();
};

} // namespace sensor