#include "ms5611.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char *TAG = "MS5611";

namespace sensor
{

// 构造函数
MS5611::MS5611(i2c::I2CBus &i2c_bus, uint8_t device_address, OSR osr)
    : i2c_bus_(i2c_bus), device_address_(device_address), current_osr_(osr)
{
}

// 析构函数
MS5611::~MS5611()
{
    if (device_handle_) {
        i2c_bus_.remove_device(device_handle_);
        device_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "MS5611 device released");
}

// 初始化传感器
esp_err_t MS5611::init()
{
    if (!i2c_bus_.is_initialized()) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // 添加I2C设备
    i2c::DeviceConfig dev_cfg = {
        .device_address = device_address_, .scl_speed_hz = 400000, .addr_len = I2C_ADDR_BIT_LEN_7};

    esp_err_t ret = i2c_bus_.add_device(dev_cfg, device_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MS5611 device at address 0x%02X", device_address_);
        return ret;
    }

    // 软复位
    ret = reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MS5611 reset failed");
        return ret;
    }

    // 读取校准系数
    ret = read_calibration_coeffs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration coefficients");
        return ret;
    }

    // 验证校准系数
    if (!is_calibration_valid()) {
        ESP_LOGW(TAG, "Calibration coefficients may be invalid");
    }

    is_initialized_ = true;
    calibration_loaded_ = true;

    ESP_LOGI(TAG, "MS5611 initialized successfully at address 0x%02X", device_address_);

    // 打印校准系数
    for (size_t i = 0; i < calib_coeffs_.size(); i++) {
        ESP_LOGI(TAG, "C%zu: 0x%04X (%d)", i + 1, calib_coeffs_[i], calib_coeffs_[i]);
    }

    return ESP_OK;
}

// 软复位
esp_err_t MS5611::reset()
{
    if (!device_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = MS5611_CMD_RESET;
    esp_err_t ret = i2c_bus_.write_reg(device_handle_, cmd, 1, 100);

    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 等待复位完成
        ESP_LOGD(TAG, "MS5611 reset successful");
    }
    else {
        ESP_LOGE(TAG, "MS5611 reset failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

// 读取校准系数
esp_err_t MS5611::read_calibration_coeffs()
{
    if (!device_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Reading MS5611 calibration coefficients...");

    for (size_t i = 0; i < calib_coeffs_.size(); i++) {
        uint8_t cmd = MS5611_CMD_PROM_READ + (i * 2);
        uint8_t buffer[2] = {0};

        esp_err_t ret = i2c_bus_.write_then_read(device_handle_, &cmd, 1, buffer, sizeof(buffer), 200);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read C%zu: %s (cmd: 0x%02X)", i + 1, esp_err_to_name(ret), cmd);
            return ret;
        }

        calib_coeffs_[i] = (buffer[0] << 8) | buffer[1];
        vTaskDelay(pdMS_TO_TICKS(2)); // 短暂延时
    }

    return ESP_OK;
}

// 验证校准系数
bool MS5611::is_calibration_valid() const
{
    // 检查所有系数都不为0
    for (const auto &coeff : calib_coeffs_) {
        if (coeff == 0) {
            return false;
        }
    }

    // 检查系数范围是否合理（根据MS5611数据手册）
    if (calib_coeffs_[0] < 10000 || calib_coeffs_[0] > 20000)
        return false; // C1
    if (calib_coeffs_[1] < 10000 || calib_coeffs_[1] > 20000)
        return false; // C2
    if (calib_coeffs_[2] < 0 || calib_coeffs_[2] > 10000)
        return false; // C3
    if (calib_coeffs_[3] < 0 || calib_coeffs_[3] > 10000)
        return false; // C4
    if (calib_coeffs_[4] < 0 || calib_coeffs_[4] > 10000)
        return false; // C5
    if (calib_coeffs_[5] < 0 || calib_coeffs_[5] > 10000)
        return false; // C6

    return true;
}

// 读取ADC值
esp_err_t MS5611::read_adc(uint8_t convert_cmd, uint32_t &adc_data)
{
    if (!device_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    // 发送转换命令
    esp_err_t ret = i2c_bus_.write_reg(device_handle_, convert_cmd, 1, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send convert command 0x%02X: %s", convert_cmd, esp_err_to_name(ret));
        return ret;
    }

    // 等待转换完成（根据OSR）
    uint8_t osr_index = static_cast<uint8_t>(current_osr_);
    vTaskDelay(pdMS_TO_TICKS(osr_delays_[osr_index]));

    // 读取ADC数据
    uint8_t read_cmd = MS5611_CMD_ADC_READ;
    uint8_t buffer[3] = {0};

    ret = i2c_master_transmit_receive(device_handle_, &read_cmd, 1, buffer, sizeof(buffer), 200);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC data: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_data = (static_cast<uint32_t>(buffer[0]) << 16) | (static_cast<uint32_t>(buffer[1]) << 8) |
               static_cast<uint32_t>(buffer[2]);

    return ESP_OK;
}

// 读取原始D1/D2值
esp_err_t MS5611::read_raw_data(uint32_t &d1, uint32_t &d2)
{
    if (!is_initialized_ || !calibration_loaded_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t osr_index = static_cast<uint8_t>(current_osr_);

    // 读取压力(D1)
    esp_err_t ret = read_adc(d1_convert_cmds_[osr_index], d1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read D1 (pressure)");
        return ret;
    }

    // 读取温度(D2)
    ret = read_adc(d2_convert_cmds_[osr_index], d2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read D2 (temperature)");
        return ret;
    }

    return ESP_OK;
}

// 转换原始数据
void MS5611::convert_raw_data(uint32_t d1, uint32_t d2, Data &data)
{
    // 保存原始值
    data.raw_d1 = d1;
    data.raw_d2 = d2;

    // 提取校准系数
    int32_t C1 = calib_coeffs_[0]; // Pressure sensitivity
    int32_t C2 = calib_coeffs_[1]; // Pressure offset
    int32_t C3 = calib_coeffs_[2]; // Temperature coefficient of pressure sensitivity
    int32_t C4 = calib_coeffs_[3]; // Temperature coefficient of pressure offset
    int32_t C5 = calib_coeffs_[4]; // Reference temperature
    int32_t C6 = calib_coeffs_[5]; // Temperature coefficient of the temperature

    // 使用64位计算避免溢出
    int32_t dT = static_cast<int32_t>(d2) - (static_cast<int32_t>(C5) << 8);
    int32_t TEMP = 2000 + ((dT * static_cast<int32_t>(C6)) >> 23);

    int32_t OFF = (static_cast<int32_t>(C2) << 16) + ((static_cast<int32_t>(C4) * dT) >> 7);
    int32_t SENS = (static_cast<int32_t>(C1) << 15) + ((static_cast<int32_t>(C3) * dT) >> 8);

    // 二阶温度补偿
    if (TEMP < 2000) {
        int32_t T2 = (dT * dT) >> 31;
        int32_t OFF2 = 61 * (TEMP - 2000) * (TEMP - 2000) / 16;
        int32_t SENS2 = 29 * (TEMP - 2000) * (TEMP - 2000) / 16;

        if (TEMP < -1500) {
            OFF2 += 17 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 += 9 * (TEMP + 1500) * (TEMP + 1500);
        }

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int32_t P = ((static_cast<int32_t>(d1) * SENS) >> 21) - OFF;

    // 转换为物理单位
    data.temperature = static_cast<double>(TEMP) / 100.0;
    data.pressure_mbar = static_cast<double>(P) / (100.0 * 1024.0);
    data.pressure_pa = data.pressure_mbar * 100.0;
}

// 读取传感器数据
esp_err_t MS5611::read_data(Data &data)
{
    if (!is_initialized_ || !calibration_loaded_) {
        ESP_LOGE(TAG, "MS5611 not initialized or calibration not loaded");
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t d1 = 0, d2 = 0;
    esp_err_t ret = read_raw_data(d1, d2);

    if (ret == ESP_OK) {
        convert_raw_data(d1, d2, data);
    }

    return ret;
}

// 设置OSR
void MS5611::set_osr(OSR osr)
{
    current_osr_ = osr;
    ESP_LOGI(TAG, "MS5611 OSR set to %d", static_cast<int>(osr));
}

} // namespace sensor