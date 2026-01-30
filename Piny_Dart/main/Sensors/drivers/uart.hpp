#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class uart
{
public:
    uart(gpio_num_t tx_pin, gpio_num_t rx_pin, const int &baud_rate, const uart_port_t port);
    ~uart();

    esp_err_t initialize();

    int write(const uint8_t *data, size_t size);
    int read(uint8_t *data, size_t size);

    void uart_event_task(void *pvParameters);

private:
    static void uart_event_task_wrapper(void *pvParameters);

    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    int baud_rate;
    uart_port_t port;

    size_t rx_buffer_size = sizeof(rx_buffer);
    size_t tx_buffer_size = sizeof(tx_buffer);

    uint8_t rx_buffer[256];
    uint8_t tx_buffer[256];

    QueueHandle_t rxQueueHandle;
    QueueHandle_t txQueueHandle;

    QueueHandle_t uartEventQueueHandle;
};