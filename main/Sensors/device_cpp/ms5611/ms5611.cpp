#include "ms5611.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char *TAG = "MS5611";

namespace sensor
{

MS5611::MS5611(i2c::I2CBus &i2c_bus, uint8_t device_address, OSR osr)
    : i2c_bus_(i2c_bus), device_address_(device_address), current_osr_(osr)
{
}

MS5611::~MS5611()
{
    if (device_handle_) {
        i2c_bus_.remove_device(device_handle_);
        device_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "MS5611 device released");
}

esp_err_t MS5611::init()
{
    if (!i2c_bus_.is_initialized()) {
        i2c_bus_.init();
        // ESP_LOGE(TAG, "I2C bus not initialized");
        // return ESP_ERR_INVALID_STATE;
    }

    i2c::DeviceConfig dev_cfg = {
        .device_address = device_address_, .scl_speed_hz = 400000, .addr_len = I2C_ADDR_BIT_LEN_7};

    esp_err_t ret = i2c_bus_.add_device(dev_cfg, device_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MS5611 device at address 0x%02X", device_address_);
        return ret;
    }

    ret = reset();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MS5611 reset failed");
        return ret;
    }

    ret = read_calibration_coeffs();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read calibration coefficients");
        return ret;
    }

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

esp_err_t MS5611::reset()
{
    if (!device_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t cmd = MS5611_CMD_RESET;
    esp_err_t ret = i2c_bus_.write_reg(device_handle_, cmd, 1, 100);

    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGD(TAG, "MS5611 reset successful");
    }
    else {
        ESP_LOGE(TAG, "MS5611 reset failed: %s", esp_err_to_name(ret));
    }

    return ret;
}

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
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    return ESP_OK;
}

bool MS5611::is_calibration_valid() const
{
    for (const auto &coeff : calib_coeffs_) {
        if (coeff == 0) {
            return false;
        }
    }

    if (calib_coeffs_[0] < 40000 || calib_coeffs_[0] > 48000)
        return false; // C1
    if (calib_coeffs_[1] < 30000 || calib_coeffs_[1] > 38000)
        return false; // C2
    if (calib_coeffs_[2] < 2000 || calib_coeffs_[2] > 4000)
        return false; // C3
    if (calib_coeffs_[3] < 2000 || calib_coeffs_[3] > 4000)
        return false; // C4
    if (calib_coeffs_[4] < 20000 || calib_coeffs_[4] > 30000)
        return false; // C5
    if (calib_coeffs_[5] < 20000 || calib_coeffs_[5] > 30000)
        return false; // C6

    return true;
}

esp_err_t MS5611::read_reg(uint8_t convert_cmd, uint32_t &adc_data)
{
    if (!device_handle_) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = i2c_bus_.write_reg(device_handle_, convert_cmd, 1, 10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send convert command 0x%02X: %s", convert_cmd, esp_err_to_name(ret));
        return ret;
    }

    uint8_t osr_index = static_cast<uint8_t>(current_osr_);
    vTaskDelay(pdMS_TO_TICKS(10));
    // vTaskDelay(pdMS_TO_TICKS(osr_delays_[osr_index]));

    uint8_t read_cmd = MS5611_CMD_ADC_READ;
    uint8_t buffer[3] = {0};

    ret = i2c_bus_.write_then_read(device_handle_, &read_cmd, 1, buffer, sizeof(buffer), 200);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC data: %s", esp_err_to_name(ret));
        return ret;
    }

    adc_data = (static_cast<uint32_t>(buffer[0]) << 16) | (static_cast<uint32_t>(buffer[1]) << 8) |
               static_cast<uint32_t>(buffer[2]);

    return ESP_OK;
}

esp_err_t MS5611::read_raw_data(uint32_t &d1, uint32_t &d2)
{
    if (!is_initialized_ || !calibration_loaded_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t osr_index = static_cast<uint8_t>(current_osr_);

    // 读取压力(D1)
    esp_err_t ret = read_reg(d1_convert_cmds_[osr_index], d1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read D1 (pressure)");
        return ret;
    }

    // 读取温度(D2)
    ret = read_reg(d2_convert_cmds_[osr_index], d2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read D2 (temperature)");
        return ret;
    }

    return ESP_OK;
}

void MS5611::convert_raw_data(const RawData &_rawdata, ConvertData &convertData)
{

    int64_t C1 = calib_coeffs_[0];
    int64_t C2 = calib_coeffs_[1];
    int64_t C3 = calib_coeffs_[2];
    int64_t C4 = calib_coeffs_[3];
    int64_t C5 = calib_coeffs_[4];
    int64_t C6 = calib_coeffs_[5];

    int64_t dT = (int64_t)_rawdata.d2 - (C5 << 8);
    int64_t TEMP = 2000 + ((dT * C6) >> 23);

    int64_t OFF = (C2 << 16) + ((C4 * dT) >> 7);
    int64_t SENS = (C1 << 15) + ((C3 * dT) >> 8);

    if (TEMP < 2000) {
        int64_t T2 = (dT * dT) >> 31;
        int64_t OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 1;
        int64_t SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) >> 2;

        TEMP -= T2;
        OFF -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = ((((int64_t)_rawdata.d1 * SENS) >> 21) - OFF) >> 15;

    convertData.temperature = TEMP / 100.0; // ℃
    convertData.pressure_pa = (double)P;    // Pa
    convertData.pressure_mbar = convertData.pressure_pa / 100.0;

    // 只计算相对高度
    static bool initialized = false;
    static double P0 = 0.0;

    if (!initialized) {
        P0 = convertData.pressure_mbar;
        initialized = true;
    }

    convertData.height = -(convertData.pressure_mbar - P0) * 8.43;
}

esp_err_t MS5611::read_data(ConvertData &data)
{
    RawData _rawdata;
    if (!is_initialized_ || !calibration_loaded_) {
        ESP_LOGE(TAG, "MS5611 not initialized or calibration not loaded");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = read_raw_data(_rawdata.d1, _rawdata.d2);

    if (ret == ESP_OK) {
        convert_raw_data(_rawdata, data);
    }

    return ret;
}

void MS5611::set_osr(OSR osr)
{
    current_osr_ = osr;
    ESP_LOGI(TAG, "MS5611 OSR set to %d", static_cast<int>(osr));
}

} // namespace sensor