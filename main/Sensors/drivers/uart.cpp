#include "uart.hpp"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <cstring>

#include "esp_log.h"

#define TAG "uart"

uart::uart(gpio_num_t tx_pin, gpio_num_t rx_pin, const int &baud_rate, const uart_port_t port)
    : tx_pin(tx_pin), rx_pin(rx_pin), baud_rate(baud_rate), port(port)
{
    rxQueueHandle = xQueueCreate(10, rx_buffer_size);
    txQueueHandle = xQueueCreate(10, tx_buffer_size);
}

uart::~uart()
{
    if (uartEventQueueHandle) {
        vQueueDelete(uartEventQueueHandle);
        uartEventQueueHandle = nullptr;
    }
    if (rxQueueHandle) {
        vQueueDelete(rxQueueHandle);
        rxQueueHandle = nullptr;
    }
    if (txQueueHandle) {
        vQueueDelete(txQueueHandle);
        txQueueHandle = nullptr;
    }
    uart_driver_delete(port);
}

void uart::uart_event_task_wrapper(void *pvParameters)
{
    uart *uart_obj = static_cast<uart *>(pvParameters);
    if (uart_obj == nullptr) {
        ESP_LOGE(TAG, "UART task get null object!");
        vTaskDelete(NULL);
        return;
    }
    uart_obj->uart_event_task(nullptr);
}

void uart::uart_event_task(void *pvParameters)
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    uart_event_t event;
    size_t buffered_size;
    for (;;) {
        if (uartEventQueueHandle == nullptr || !xQueueReceive(uartEventQueueHandle, (void *)&event, portMAX_DELAY)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        bzero(rx_buffer, rx_buffer_size);
        ESP_LOGD(TAG, "uart[%d] event type: %d", port, event.type);

        switch (event.type) {
        case UART_DATA: {
            ESP_LOGD(TAG, "[UART DATA]: recv %d bytes", event.size);
            uart_read_bytes(port, rx_buffer, event.size, portMAX_DELAY);
            ESP_LOGD(TAG, "[DATA EVT]: %.*s", event.size, rx_buffer);
            // uart_write_bytes(port, (const char *)rx_buffer, event.size);
            break;
        }
        case UART_FIFO_OVF: {
            uart_flush_input(port);
            xQueueReset(uartEventQueueHandle);
            break;
        }
        case UART_BUFFER_FULL: {
            // ESP_LOGE(TAG, "[UART ERR] ring buffer full");
            uart_flush_input(port);
            xQueueReset(uartEventQueueHandle);
            break;
        }
        case UART_BREAK: {
            // ESP_LOGW(TAG, "[UART WARN] rx break");
            break;
        }
        case UART_PARITY_ERR: {
            // ESP_LOGE(TAG, "[UART ERR] parity check error");
            break;
        }
        case UART_FRAME_ERR: {
            // ESP_LOGE(TAG, "[UART ERR] frame error");
            break;
        }
        case UART_PATTERN_DET: {
            uart_get_buffered_data_len(port, &buffered_size);
            int pos = uart_pattern_pop_pos(port);
            ESP_LOGD(TAG, "[UART PATTERN DET] pos: %d, buffered size: %d", pos, buffered_size);

            if (pos == -1) {
                ESP_LOGE(TAG, "[UART ERR] pattern pos queue full, flush rx buffer");
                uart_flush_input(port);
            }
            else {
                uart_read_bytes(port, rx_buffer, pos, pdMS_TO_TICKS(100));
                uint8_t pat[4] = {0};
                uart_read_bytes(port, pat, 3, pdMS_TO_TICKS(100));
                // ESP_LOGI(TAG, "read data: %s", rx_buffer);
                // ESP_LOGI(TAG, "read pattern: %s", pat);
            }
            break;
        }
        case UART_DATA_BREAK: {
            ESP_LOGW(TAG, "[UART WARN] data break event");
            uart_flush_input(port);
            break;
        }
        case UART_WAKEUP: {
            ESP_LOGD(TAG, "[UART INFO] wakeup event");
            break;
        }
        default: {
            ESP_LOGW(TAG, "[UART WARN] unhandled event type: %d (may be UART_EVENT_MAX)", event.type);
            break;
        }
        }
    }
    vTaskDelete(NULL);
}

esp_err_t uart::initialize()
{
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = 0,
    };

    esp_err_t ret = uart_driver_install(port, rx_buffer_size, tx_buffer_size, 20, &uartEventQueueHandle, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart driver install failed, err: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    BaseType_t task_ret = xTaskCreate(uart::uart_event_task_wrapper, "uart_event_task", 3072, this, 12, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "create uart event task failed!");
        uart_driver_delete(port);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "uart[%d] initialize success, baud: %d", port, baud_rate);
    return ESP_OK;
}

int uart::write(const uint8_t *data, size_t size)
{
    if (data == nullptr || size == 0)
        return -1;
    return uart_write_bytes(port, data, size);
}

int uart::read(uint8_t *data, size_t size)
{
    if (data == nullptr || size == 0)
        return -1;
    return uart_read_bytes(port, data, size, portMAX_DELAY);
}