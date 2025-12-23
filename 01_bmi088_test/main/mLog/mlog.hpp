#pragma once
// 修正：esp_console.h 不在 driver 目录下，直接包含
#include "driver/uart.h"
#include "esp_console.h"
#include "esp_log.h"
#include "esp_rom_uart.h"
#include "esp_vfs_dev.h"
// 新增：IDF 5.x 必需的 uart_vfs 头文件
#include "driver/uart_vfs.h"
#include <iostream>
#include <string>

#define TAG "mlog"

namespace mLog
{
void changeLog(void)
{
    // 目标波特率（按需修改）
    const int TARGET_BAUDRATE = 576000;

    // 1. 禁用当前日志输出（避免配置过程乱码）
    esp_log_level_set("*", ESP_LOG_NONE);

    // 2. 完整初始化 uart_config_t（消除所有字段未初始化警告）
    uart_config_t uart0_config = {
        .baud_rate = TARGET_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0, // IDF 5.x 新增字段
        .source_clk = UART_SCLK_APB,
        .flags = 0,
    };

    // 3. 重新初始化 UART0
    ESP_ERROR_CHECK(uart_driver_delete(UART_NUM_0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart0_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0));

    // 4. 替换废弃 API（IDF 5.x 正确接口）
    uart_vfs_dev_use_driver(UART_NUM_0);

    // 5. 绑定控制台到 UART0
    esp_rom_uart_set_as_console(UART_NUM_0);
    // 替换废弃的行结束符配置
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_LF);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_LF);

    // 6. 恢复日志级别
    esp_log_level_set("*", ESP_LOG_INFO);

    // 验证输出
    ESP_LOGI(TAG, "console baudrate: %d", TARGET_BAUDRATE);
}
} // namespace mLog