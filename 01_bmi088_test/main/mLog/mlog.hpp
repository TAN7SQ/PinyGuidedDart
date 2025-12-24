#pragma once

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_rom_uart.h"
#include "esp_vfs_dev.h"
#include <iostream>
#include <string>

#define TAG "mlog"

/**
 * @brief 改变日志输出到UART0
 * @warning 调用该函数后，需要修改vscode的idf monitor baud rate
 *
 */

namespace mLog
{
void changeLog(void)
{
    const int TARGET_BAUDRATE = 576000;

    esp_log_level_set("*", ESP_LOG_NONE);

    uart_config_t uart0_config = {
        .baud_rate = TARGET_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,
        .flags = 0,
    };

    ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart0_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0));

    uart_vfs_dev_use_driver(UART_NUM_0);

    esp_rom_uart_set_as_console(UART_NUM_0);
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_LF);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_LF);

    esp_log_level_set("*", ESP_LOG_INFO);

    ESP_LOGI(TAG, "console baudrate: %d", TARGET_BAUDRATE);
}
} // namespace mLog