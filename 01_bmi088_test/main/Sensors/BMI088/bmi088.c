#include "bmi088.h"
/*
 * BMI088 SPI驱动完整代码（ESP32）
 * 功能：读取BMI088加速度计和陀螺仪数据，支持全双工SPI、数据滤波、错误处理
 * 引脚定义：CLK=13, MOSI=12, MISO=11, CS_GYRO=10, CS_ACCEL=9
 * 配置：加速度计±3g量程/50Hz输出，陀螺仪±500°/s量程/100Hz输出
 */
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static SemaphoreHandle_t spi_bus_mutex = NULL; // SPI总线互斥锁

// SPI设备句柄
static spi_device_handle_t spi_acc_handle = NULL;
static spi_device_handle_t spi_gyro_handle = NULL;

// ========================== SPI底层操作函数 ==========================
/**
 * @brief SPI写寄存器（全双工模式）
 * @param handle SPI设备句柄
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @return esp_err_t 操作结果
 */
static esp_err_t bmi088_spi_write_reg(spi_device_handle_t handle, uint8_t reg, uint8_t data)
{
    esp_err_t ret;
    if (xSemaphoreTake(spi_bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Take SPI mutex failed for write");
        return ESP_ERR_TIMEOUT;
    }

    uint8_t tx_buf[2] = {reg & 0x7F, data}; // 写操作最高位为0
    uint8_t rx_buf[2] = {0};
    spi_transaction_t t = {
        .length = 16, // 2字节=16位
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf, // 全双工必须配置rx_buffer
        .user = NULL,
    };

    ret = spi_device_transmit(handle, &t);
    xSemaphoreGive(spi_bus_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI write reg 0x%02X failed", reg);
    }
    return ret;
}

/**
 * @brief SPI读寄存器（全双工模式）
 * @param handle SPI设备句柄
 * @param reg 寄存器地址
 * @param data 读取到的数据指针
 * @return esp_err_t 操作结果
 */
static esp_err_t bmi088_spi_read_reg(spi_device_handle_t handle, uint8_t reg, uint8_t *data)
{
    esp_err_t ret;
    if (xSemaphoreTake(spi_bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Take SPI mutex failed for read");
        *data = 0;
        return ESP_ERR_TIMEOUT;
    }

    uint8_t tx_buf[2] = {reg | 0x80, 0x00}; // 读操作最高位为1
    uint8_t rx_buf[2] = {0};
    spi_transaction_t t = {
        .length = 16,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
        .user = NULL,
    };

    ret = spi_device_transmit(handle, &t);
    xSemaphoreGive(spi_bus_mutex);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI read reg 0x%02X failed", reg);
        *data = 0;
        return ret;
    }

    *data = rx_buf[1]; // 第二个字节是有效数据
    return ret;
}

/**
 * @brief SPI连续读多个寄存器（全双工模式）
 * @param handle SPI设备句柄
 * @param reg 起始寄存器地址
 * @param len 要读取的字节数
 * @param data 读取到的数据缓冲区
 * @return esp_err_t 操作结果
 */
static esp_err_t bmi088_spi_read_regs(spi_device_handle_t handle, uint8_t reg, uint8_t len, uint8_t *data)
{
    esp_err_t ret;
    if (xSemaphoreTake(spi_bus_mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGE(TAG, "Take SPI mutex failed for read regs");
        return ESP_ERR_TIMEOUT;
    }

    // 申请缓冲区：1字节地址 + len字节数据
    uint8_t *tx_buf = (uint8_t *)malloc(len + 1);
    uint8_t *rx_buf = (uint8_t *)malloc(len + 1);
    if (tx_buf == NULL || rx_buf == NULL) {
        xSemaphoreGive(spi_bus_mutex);
        ESP_LOGE(TAG, "Malloc failed");
        return ESP_ERR_NO_MEM;
    }

    // 填充读指令
    tx_buf[0] = reg | 0x80;
    memset(&tx_buf[1], 0, len);

    // 全双工传输
    spi_transaction_t t = {
        .length = (len + 1) * 8, // 总位数
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    ret = spi_device_transmit(handle, &t);
    if (ret == ESP_OK) {
        memcpy(data, &rx_buf[1], len); // 跳过地址字节
    }
    else {
        ESP_LOGE(TAG, "SPI read regs 0x%02X len %d failed", reg, len);
    }

    free(tx_buf);
    free(rx_buf);
    xSemaphoreGive(spi_bus_mutex);
    return ret;
}

// ========================== BMI088初始化函数 ==========================
/**
 * @brief 加速度计初始化
 * @return esp_err_t 操作结果
 */
static esp_err_t bmi088_acc_init(void)
{
    uint8_t chip_id = 0;
    esp_err_t ret;

    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_acc_handle, BMI088_ACC_SOFT_RESET, 0xB6)); // 软复位，至少50ms后再读取
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 2. 读取芯片ID（重试3次）
    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(bmi088_spi_read_reg(spi_acc_handle, BMI088_ACC_CHIP_ID, &chip_id));
        if (chip_id == 0x1E) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGW(TAG, "Retry ACC ID read (%d/3), current ID: 0x%02X", i + 1, chip_id);
    }
    if (chip_id != 0x1E) {
        ESP_LOGE(TAG, "ACC chip ID error: 0x%02X", chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ACC init success (ID: 0x%02X)", chip_id);

    // 3. 接口配置（SPI模式，禁用中断）
    ret = bmi088_spi_write_reg(spi_acc_handle, BMI088_ACC_IF_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(5));

    ret |= bmi088_spi_write_reg(spi_acc_handle, BMI088_ACC_PWR_CONF, 0x00); // 退出休眠
    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_acc_handle, BMI088_ACC_PWR_CTRL, 0x04)); // 开启测量模式
    vTaskDelay(pdMS_TO_TICKS(100));
    // 800Hz输出速率 1 0x02 0x0B = 1 010 1011 = 0x8B / 1600Hz输出速率 1 0x02 0x0C = 1 010 1100 = 0xAC
    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_acc_handle, BMI088_ACC_CONF, 0x8C));
    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_acc_handle, BMI088_ACC_RANGE, 0x03)); // ±24g量程

    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_acc_handle, BMI088_ACC_INT_EN_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t pwr_ctrl = 0, acc_conf = 0;

    ESP_ERROR_CHECK(bmi088_spi_read_reg(spi_acc_handle, BMI088_ACC_CONF, &acc_conf));

    for (int i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(bmi088_spi_read_reg(spi_acc_handle, BMI088_ACC_PWR_CTRL, &pwr_ctrl));
        if (pwr_ctrl == 0x04) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGW(TAG, "Retry ACC PWR_CTRL read (%d/3), current PWR_CTRL: 0x%02X", i + 1, pwr_ctrl);
    }
    ESP_LOGI(TAG, "ACC PWR_CTRL: 0x%02X, CONF: 0x%02X", pwr_ctrl, acc_conf);
    if (pwr_ctrl != 0x04) {
        ESP_LOGE(TAG, "ACC not in measurement mode!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

/**
 * @brief 陀螺仪初始化（降噪配置）
 * @return esp_err_t 操作结果
 */
static esp_err_t bmi088_gyro_init(void)
{
    uint8_t chip_id = 0;
    esp_err_t ret;

    vTaskDelay(pdMS_TO_TICKS(10));
    // 读取芯片ID
    ESP_ERROR_CHECK(bmi088_spi_read_reg(spi_gyro_handle, BMI088_GYRO_CHIP_ID, &chip_id));
    if (chip_id != 0x0F) {
        ESP_LOGE(TAG, "GYRO chip ID error: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // 软复位陀螺仪
    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_gyro_handle, 0x14, 0xB6));
    vTaskDelay(pdMS_TO_TICKS(20));

    // 低噪声配置
    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_gyro_handle, BMI088_GYRO_BANDWITH, 0x02)); // 1000Hz输出+116低通滤波带宽

    // 0x01 = ±1000°/s量程  | 0x00 = ±2000°/s量程
    ESP_ERROR_CHECK(bmi088_spi_write_reg(spi_gyro_handle, BMI088_GYRO_RANGE, 0x00));

    // 验证配置
    uint8_t gyro_range = 0;
    bmi088_spi_read_reg(spi_gyro_handle, BMI088_GYRO_RANGE, &gyro_range);
    ESP_LOGI(TAG, "GYRO RANGE: 0x%02X", gyro_range);

    ESP_LOGI(TAG, "GYRO init success (ID: 0x%02X)", chip_id);
    return ESP_OK;
}

/**
 * @brief SPI总线和BMI088设备初始化
 */
esp_err_t spi_setup(void)
{
    // 创建SPI总线互斥锁
    spi_bus_mutex = xSemaphoreCreateMutex();
    if (spi_bus_mutex == NULL) {
        ESP_LOGE(TAG, "Create SPI mutex failed");
        return ESP_FAIL;
    }

    // 配置SPI总线参数
    spi_bus_config_t bmi088_spi_bus = {
        .mosi_io_num = BMI088_MOSI,
        .miso_io_num = BMI088_MISO,
        .sclk_io_num = BMI088_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    // 初始化SPI总线（使用DMA）
    ESP_ERROR_CHECK(spi_bus_initialize(BMI088_HOST, &bmi088_spi_bus, SPI_DMA_CH_AUTO));

    // 配置陀螺仪SPI设备
    spi_device_interface_config_t gyro_dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
        .cs_ena_posttrans = 8,
        .cs_ena_pretrans = 4,
        .mode = 3, // SPI模式3（CPOL=1, CPHA=1）
        .spics_io_num = BMI088_CS_GYRO,
        .queue_size = 5,
        .flags = 0, // 全双工模式
    };
    ESP_ERROR_CHECK(spi_bus_add_device(BMI088_HOST, &gyro_dev_cfg, &spi_gyro_handle));

    // 配置加速度计SPI设备
    spi_device_interface_config_t acc_dev_cfg = {
        .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
        .cs_ena_posttrans = 8,
        .cs_ena_pretrans = 4,
        .mode = 3, // SPI模式3（CPOL=1, CPHA=1）
        .spics_io_num = BMI088_CS_ACCEL,
        .queue_size = 5,
        .flags = 0, // 全双工模式
    };
    ESP_ERROR_CHECK(spi_bus_add_device(BMI088_HOST, &acc_dev_cfg, &spi_acc_handle));

    // 初始化传感器
    if (bmi088_gyro_init() != ESP_OK) {
        ESP_LOGE(TAG, "GYRO init failed");
        return ESP_FAIL;
    }

    if (bmi088_acc_init() != ESP_OK) {
        ESP_LOGE(TAG, "ACC init failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// ========================== 数据读取函数 ==========================
/**
 * @brief 读取加速度计数据（带数据就绪等待）
 * @param data 加速度计数据指针
 * @return esp_err_t 操作结果
 */
static esp_err_t bmi088_read_acc(bmi088_data_t *data)
{
    uint8_t acc_data[6] = {0};
    uint8_t status = 0;
    esp_err_t ret;
    static bmi088_data_t last_acc_data = {0};

    // 等待数据就绪（最多等待8次，每次2ms）
    int retry = 0;
    do {
        ret = bmi088_spi_read_reg(spi_acc_handle, BMI088_ACC_STATUS, &status);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Read ACC status failed");
            return ret;
        }
        retry++;
        vTaskDelay(pdMS_TO_TICKS(2));
    } while ((status & 0x80) == 0 && retry < 8);

    // 数据未就绪时使用上一次有效数据
    if (retry >= 8) {
        ESP_LOGW(TAG, "ACC data not ready, status: 0x%02X", status);
        data->acc_x = last_acc_data.acc_x;
        data->acc_y = last_acc_data.acc_y;
        data->acc_z = last_acc_data.acc_z;
        return ESP_OK;
    }

    // 读取6字节加速度数据
    ret = bmi088_spi_read_regs(spi_acc_handle, BMI088_ACC_DATA_X_LSB, 6, acc_data);
    if (ret == ESP_OK) {
        // 拼接16位数据（LSB在前，MSB在后）
        data->acc_x = (int16_t)((acc_data[1] << 8) | acc_data[0]);
        data->acc_y = (int16_t)((acc_data[3] << 8) | acc_data[2]);
        data->acc_z = (int16_t)((acc_data[5] << 8) | acc_data[4]);
        // 保存最后一次有效数据
        last_acc_data = *data;
    }
    else {
        ESP_LOGE(TAG, "Read ACC data failed");
    }

    return ret;
}

/**
 * @brief 读取陀螺仪数据（带滑动平均滤波）
 * @param data 陀螺仪数据指针
 * @return esp_err_t 操作结果
 */
static esp_err_t bmi088_read_gyro(bmi088_data_t *data)
{
#define GYRO_FILTER_WINDOW 5
    static int16_t gyro_x_buf[GYRO_FILTER_WINDOW] = {0};
    static int16_t gyro_y_buf[GYRO_FILTER_WINDOW] = {0};
    static int16_t gyro_z_buf[GYRO_FILTER_WINDOW] = {0};
    static int filter_idx = 0;

    uint8_t gyro_data[6] = {0};
    esp_err_t ret = bmi088_spi_read_regs(spi_gyro_handle, BMI088_GYRO_RATE_X_LSB, 6, gyro_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read GYRO data failed");
        return ret;
    }

    int16_t gyro_x = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    int16_t gyro_y = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    int16_t gyro_z = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    // 滑动平均滤波
    gyro_x_buf[filter_idx] = gyro_x;
    gyro_y_buf[filter_idx] = gyro_y;
    gyro_z_buf[filter_idx] = gyro_z;
    filter_idx = (filter_idx + 1) % GYRO_FILTER_WINDOW;

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < GYRO_FILTER_WINDOW; i++) {
        sum_x += gyro_x_buf[i];
        sum_y += gyro_y_buf[i];
        sum_z += gyro_z_buf[i];
    }

    data->gyro_x = sum_x / GYRO_FILTER_WINDOW;
    data->gyro_y = sum_y / GYRO_FILTER_WINDOW;
    data->gyro_z = sum_z / GYRO_FILTER_WINDOW;

    return ESP_OK;
}

/**
 * @brief 读取BMI088完整数据（加速度计+陀螺仪）
 * @param data 数据结构体指针
 * @return esp_err_t 操作结果
 */
esp_err_t bmi088_read_data(bmi088_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = bmi088_read_acc(data);
    if (ret != ESP_OK)
        return ret;

    ret = bmi088_read_gyro(data);
    return ret;
}

// ========================== 测试任务 ==========================
/**
 * @brief BMI088数据读取测试任务
 * @param arg 任务参数（未使用）
 */
void bmi088_task(void *arg)
{
    bmi088_data_t sensor_data;
    esp_err_t ret;

    // 等待系统初始化
    vTaskDelay(pdMS_TO_TICKS(500));
    // 初始化SPI和BMI088
    ESP_ERROR_CHECK(spi_setup());
    // 等待传感器稳定
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 打印量程参考信息
    ESP_LOGI(TAG, "ACC ±3g range: 1LSB = 0.000088g, 1g = 11468 LSB");
    ESP_LOGI(TAG, "GYRO ±500°/s range: 1LSB = 0.0152588°/s");

    // 循环读取数据
    while (1) {
        ret = bmi088_read_data(&sensor_data);
        if (ret == ESP_OK) {
            // 转换为物理单位
            float acc_x_g = sensor_data.acc_x * 0.000088f;
            float acc_y_g = sensor_data.acc_y * 0.000088f;
            float acc_z_g = sensor_data.acc_z * 0.000088f;
            float gyro_x_dps = sensor_data.gyro_x * 0.0152588f;
            float gyro_y_dps = sensor_data.gyro_y * 0.0152588f;
            float gyro_z_dps = sensor_data.gyro_z * 0.0152588f;

            // 打印数据
            ESP_LOGI(
                TAG, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", acc_x_g, acc_y_g, acc_z_g, gyro_x_dps, gyro_y_dps, gyro_z_dps);
        }
        else {
            ESP_LOGE(TAG, "Read data failed, err: %d", ret);
        }

        // 50Hz读取频率（匹配加速度计输出速率）
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
