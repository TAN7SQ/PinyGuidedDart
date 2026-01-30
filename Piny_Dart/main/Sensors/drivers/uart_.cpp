#include "uart_.hpp"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#define TAG "uart"

uart::uart()
{
}

uart::~uart()
{
}

esp_err_t uart::initialize(const uart_config_t &config)
{

    uart_config_t uart_config = {
        .baud_rate = config.baud_rate,
        .data_bits = config.data_bits,
        .parity = config.parity,
        .stop_bits = config.stop_bits,
        .flow_ctrl = config.flow_ctrl,
      
    };

    esp_err_t ret = uart_param_config(config.port, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(config.port, config.rx_buffer_size, config.tx_buffer_size, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}