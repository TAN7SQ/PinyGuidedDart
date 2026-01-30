#pragma once

#include "driver/uart.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class uart
{
public:
    uart();
    ~uart();

    esp_err_t initialize(const uart_config_t &config);

    esp_err_t write(const uint8_t *data, size_t size);
    esp_err_t read(uint8_t *data, size_t size);

private:
    QueueHandle_t rxQueueHandle;
    QueueHandle_t txQueueHandle;
};