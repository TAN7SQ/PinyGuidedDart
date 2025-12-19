#include "ms5611.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"

#include "driver/i2c_master.h"

static const char *TAG = "MS5611";
i2c_master_bus_handle_t i2c_handle;

esp_err_t i2c_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = MS5611_SCL_GPIO_PIN,
        .sda_io_num = MS5611_SDA_GPIO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true, // 使能内部上拉
    };
    // ESP_ERROR_CHECK(i2c_master_bus_create(&i2c_bus_config, &i2c_handle));

    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &i2c_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C 主机初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C 主机初始化成功");
    return ret;

    // i2c_device_config_t ms5611_config = {
    //     .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    //     .device_address = MS5611_ADDR,
    //     .scl_speed_hz = IIC_CLOCK_SPEED_HZ,
    // };
    // ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &ms5611_config,));
}

/**
 * @brief 扫描单个 I2C 地址，检测设备是否存在
 * @param addr 7位 I2C 地址
 * @return true: 设备存在，false: 设备不存在
 */
esp_err_t i2c_scan_addr(uint8_t addr)
{
    // 临时设备配置（用于检测地址）
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7位地址
        .device_address = addr,                // 待检测地址
        .scl_speed_hz = IIC_CLOCK_SPEED_HZ,    // I2C 速率
    };

    i2c_master_dev_handle_t temp_dev_handle = NULL;
    esp_err_t ret = i2c_master_bus_add_device(i2c_handle, &dev_cfg, &temp_dev_handle);
    if (ret != ESP_OK) {
        // 添加设备失败 = 地址无应答
        return false;
    }

    // 发送空数据包（仅检测 ACK）
    uint8_t dummy_data = 0;

    ret = i2c_master_transmit(temp_dev_handle, &dummy_data, sizeof(uint8_t), 10);

    // 移除临时设备（避免占用资源）
    i2c_master_bus_rm_device(temp_dev_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "地址 0x%02X 存在", addr);
    }
    else {
        ESP_LOGW(TAG, "地址 0x%02X 无应答", addr);
    }

    // 仅当 ACK 正常时判定设备存在
    return ret;
}

/**
 * @brief 扫描所有 I2C 地址，输出检测结果
 */
void i2c_full_scan(void)
{
    int device_count = 0;
    ESP_LOGI(TAG, "开始扫描 I2C 总线（地址范围: 0x%02X ~ 0x%02X）", 0x11, 0xFF);
    ESP_LOGI(TAG, "----------------------------------------");

    esp_log_level_set("*", ESP_LOG_INFO);

    // 遍历所有地址
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // 跳过 I2C 保留地址（0x00~0x07、0x78~0x7F 部分为保留，可选过滤）
        // if ((addr >= 0x00 && addr <= 0x07) || (addr >= 0x78 && addr <= 0x7F)) {
        //     continue;
        // }

        // 检测地址
        if (i2c_scan_addr(addr << 1)) {
            ESP_LOGI(TAG, "✅ 检测到 I2C 设备 - 地址: 0x%02X (十进制: %d)", addr, addr);
            device_count++;
        }

        // 每扫描8个地址短暂延时，避免总线过载
        if (addr % 8 == 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    ESP_LOGI(TAG, "----------------------------------------");
    if (device_count == 0) {
        ESP_LOGW(TAG, "❌ 未检测到任何 I2C 设备");
    }
    else {
        ESP_LOGI(TAG, "✅ 扫描完成，共检测到 %d 个 I2C 设备", device_count);
    }
}

/**
 * @brief 初始化 MS5611 气压计
 * 只有五个基础命令：重置，读PROM，转换D1/D2，读取ADC结果（24bit气压和温度）
 */
static i2c_master_dev_handle_t ms5611_dev_handle = NULL;

#define MS5611_CMD_RESET 0x1E
uint16_t ms5611_calib[6]; // 存储 6 个校准系数（C1~C6）
typedef struct
{
    double temperature;   // 温度(℃)
    double pressure_mbar; // 压力(mbar)
    double pressure_pa;   // 压力(Pa)
} ms5611_data_t;
/**
 * @brief 初始化I2C主机
 */
static esp_err_t i2c_master_init(void)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = MS5611_SCL_GPIO_PIN,
        .sda_io_num = MS5611_SDA_GPIO_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,

    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MS5611_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &ms5611_dev_handle));

    ESP_LOGI(TAG, "I2C master bus initialized");
    return ESP_OK;
}

/**
 * @brief 读取MS5611 PROM校准系数（复用你原有修复后逻辑）
 */
static esp_err_t ms5611_read_prom(void)
{
    esp_err_t ret;
    uint8_t cmd;
    uint8_t calib_data[2] = {0};

    // 发送复位命令
    uint8_t reset_cmd = MS5611_CMD_RESET;
    ret = i2c_master_transmit(ms5611_dev_handle, &reset_cmd, sizeof(reset_cmd), 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "reset command failed: %s", esp_err_to_name(ret));
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "MS5611 reset success");

    ESP_LOGI(TAG, "read calibration coefficients...");

    // 读取6个校准系数（C1~C6）
    for (int i = 0; i < 6; i++) {
        cmd = MS5611_CMD_PROM_READ + (i * 2);
        memset(calib_data, 0, sizeof(calib_data));

        ret = i2c_master_transmit_receive(ms5611_dev_handle, &cmd, 1, calib_data, 2, 200);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "read C%d error(code): %s (cmd=0x%02X)", i + 1, esp_err_to_name(ret), cmd);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        ms5611_calib[i] = (uint16_t)calib_data[0] << 8 | calib_data[1];
        ESP_LOGI(TAG, "C%d: 0x%04X (decimal: %d)", i + 1, ms5611_calib[i], ms5611_calib[i]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 校验校准系数有效性
    bool calib_valid = true;
    for (int i = 0; i < 6; i++) {
        if (ms5611_calib[i] == 0) {
            ESP_LOGW(TAG, "C%d calibration coefficient is 0, may be read error", i + 1);
            calib_valid = false;
        }
    }

    return calib_valid ? ESP_OK : ESP_ERR_INVALID_STATE;
}

/**
 * @brief 读取MS5611 ADC数据（D1/D2通用函数）
 * @param convert_cmd 转换命令（D1/D2）
 * @param adc_data 输出24位ADC数据
 * @return esp_err_t 执行结果
 */
static esp_err_t ms5611_read_adc(uint8_t convert_cmd, uint32_t *adc_data)
{
    esp_err_t ret;
    uint8_t adc_buf[3] = {0};

    if (adc_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ret = i2c_master_transmit(ms5611_dev_handle, &convert_cmd, 1, 100);
    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "send convert command (cmd=0x%02X) failed: %s", convert_cmd, esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t read_cmd = MS5611_CMD_ADC_READ;
    ret = i2c_master_transmit_receive(ms5611_dev_handle,
                                      &read_cmd,
                                      1, // 发送读取命令
                                      adc_buf,
                                      3,  // 接收3字节ADC数据
                                      200 // 超时时间
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read ADC data error(code): %s", esp_err_to_name(ret));
        return ret;
    }

    // 拼接24位ADC数据（高字节在前）
    *adc_data = ((uint32_t)adc_buf[0] << 16) | ((uint32_t)adc_buf[1] << 8) | adc_buf[2];

    // ESP_LOGD(TAG, "ADC data: 0x%06X (decimal: %lu)", *adc_data, *adc_data);
    return ESP_OK;
}

// OSR（Over Sampling Ratio）,过采样率
// 4096 分辨率：转换时间 2.4ms
// 2048 分辨率：转换时间 1.2ms
// 1024 分辨率：转换时间 0.6ms
// 512  分辨率：转换时间 0.3ms
// 256  分辨率：转换时间 0.15ms
#define MS5611_CMD_CONVERT_D1_256 0x40
#define MS5611_CMD_CONVERT_D1_512 0x42
#define MS5611_CMD_CONVERT_D1_1024 0x44
#define MS5611_CMD_CONVERT_D1_2048 0x46
#define MS5611_CMD_CONVERT_D1_4096 0x48

#define MS5611_CMD_CONVERT_D2_256 0x50
#define MS5611_CMD_CONVERT_D2_512 0x52
#define MS5611_CMD_CONVERT_D2_1024 0x54
#define MS5611_CMD_CONVERT_D2_2048 0x56
#define MS5611_CMD_CONVERT_D2_4096 0x58
#define MS5611_CONVERT_DELAY_MS 2

/**
 * @brief 读取D1（压力）和D2（温度）数据
 * @param d1 输出压力ADC数据
 * @param d2 输出温度ADC数据
 * @return esp_err_t 执行结果
 */
esp_err_t ms5611_read_d1_d2(uint32_t *d1, uint32_t *d2)
{
    esp_err_t ret;

    if (d1 == NULL || d2 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 读取D1（压力）
    ret = ms5611_read_adc(MS5611_CMD_CONVERT_D1_2048, d1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read D1 error(code): %s", esp_err_to_name(ret));
        return ret;
    }

    // 读取D2（温度）
    ret = ms5611_read_adc(MS5611_CMD_CONVERT_D2_2048, d2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read D2 error(code): %s", esp_err_to_name(ret));
        return ret;
    }

    // ESP_LOGI(TAG, "D1(pressure ADC): %lu, D2(temperature ADC): %lu", *d1, *d2);
    return ESP_OK;
}

/**
 * @brief 将D1/D2原始值转换为实际温度、压力
 * @param d1      压力原始值（ADC读数）
 * @param d2      温度原始值（ADC读数）
 * @param out_data 输出：转换后的温度、压力数据
 */
static void ms5611_convert_d1d2(uint32_t d1, uint32_t d2, ms5611_data_t *out_data)
{
    if (out_data == NULL)
        return;

    // 1. 提取校准系数（C1~C6）
    int32_t C1 = ms5611_calib[0];
    int32_t C2 = ms5611_calib[1];
    int32_t C3 = ms5611_calib[2];
    int32_t C4 = ms5611_calib[3];
    int32_t C5 = ms5611_calib[4];
    int32_t C6 = ms5611_calib[5];

    // 2. 核心计算（使用64位整数避免溢出）
    int64_t dT = (int64_t)d2 - ((int64_t)C5 << 8);         // 温度差值
    int64_t T = 2000 + ((dT * (int64_t)C6) >> 23);         // 初始温度(0.01℃)
    int64_t OFF = ((int64_t)C2 << 16) + ((C4 * dT) >> 7);  // 压力偏移量
    int64_t SENS = ((int64_t)C1 << 15) + ((C3 * dT) >> 8); // 压力灵敏度

    // 3. 低温补偿（<20℃时）
    if (T < 2000) {
        int64_t T2 = (dT * dT) >> 31;
        int64_t OFF2 = 5 * ((T - 2000) * (T - 2000)) / 2;
        int64_t SENS2 = 5 * ((T - 2000) * (T - 2000)) / 4;

        // 超低温补偿（<-15℃时）
        if (T < -1500) {
            OFF2 += 7 * ((T + 1500) * (T + 1500));
            SENS2 += 11 * ((T + 1500) * (T + 1500)) / 2;
        }

        T -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    // 4. 转换为实际物理量
    out_data->temperature = (double)T / 100.0;                              // 温度(℃)
    out_data->pressure_mbar = (((d1 * SENS) >> 21) - OFF) / (100.0 * 1024); // 压力(mbar)
    out_data->pressure_pa = out_data->pressure_mbar * 100.0;                // 压力(Pa)
}

/**
 * @brief MS5611初始化（校准系数+I2C）
 */
esp_err_t ms5611_init(void)
{
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init error(code): %s", esp_err_to_name(ret));
        return ret;
    }

    ret = ms5611_read_prom();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "read calibration coefficients error(code): %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "MS5611 init success");
    return ESP_OK;
}

/**
 * @brief 测试任务：循环读取D1/D2
 */
void ms5611_task(void *arg)
{
    uint32_t d1 = 0, d2 = 0;
    esp_err_t ret;

    // 初始化MS5611
    ret = ms5611_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MS5611 init error(code): %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
    }

    ms5611_data_t ms5611_data;

    // 循环读取D1/D2
    while (1) {
        ret = ms5611_read_d1_d2(&d1, &d2);
        if (ret == ESP_OK) {
            ms5611_convert_d1d2(d1, d2, &ms5611_data);
            ESP_LOGI(TAG,
                     "%lu,%lu,%.2f,%.2f,%.2f",
                     d1,
                     d2,
                     ms5611_data.temperature,
                     ms5611_data.pressure_mbar,
                     ms5611_data.pressure_pa);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}