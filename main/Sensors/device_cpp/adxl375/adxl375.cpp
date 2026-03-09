#include "adxl375.hpp"
#include "adxl375_reg.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace sensor
{

ADXL375::ADXL375(const spi::DeviceConfig &dev_cfg) : spi_bus_(spi::SPIBus::get_instance())
{
    spi_bus_.add_device(dev_cfg, dev_handle_);
}

esp_err_t ADXL375::init()
{
    esp_err_t ret = check_device();

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device check failed");
        return ret;
    }

    ESP_LOGI(TAG, "ADXL375 init success");
    return ESP_OK;
}

esp_err_t ADXL375::read_device_id(uint8_t &id)
{
    return spi_bus_.read_reg(dev_handle_, ADXL375_DEVID, id);
}

esp_err_t ADXL375::check_device()
{
    uint8_t id = 0;
    esp_err_t ret;

    for (int i = 0; i < 5; i++) {

        ret = spi_bus_.read_reg(dev_handle_, ADXL375_DEVID, id);

        if (ret == ESP_OK && id == ADXL375_EXPECTED_ID) {
            ESP_LOGI(TAG, "ADXL375 detected, ID=0x%02X", id);
            return ESP_OK;
        }

        ESP_LOGW(TAG, "Retry read device id (%d), id=0x%02X", i + 1, id);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGE(TAG, "ADXL375 device id error: 0x%02X", id);
    return ESP_FAIL;
}

ADXL375::~ADXL375()
{
    if (dev_handle_) {
        spi_bus_.remove_device(dev_handle_);
        dev_handle_ = nullptr;
    }
}

} // namespace sensor