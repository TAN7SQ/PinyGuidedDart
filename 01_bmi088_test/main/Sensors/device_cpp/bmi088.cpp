#include "bmi088.hpp"
#include "esp_log.h"
#include "freertos/task.h"

static const char *TAG = "BMI088";

namespace sensor
{

BMI088::BMI088(const spi::DeviceConfig &dev_cfg_acc, const spi::DeviceConfig &dev_cfg_gyro)
    : spi_bus_(spi::SPIBus::get_instance())
{
    // 注册加速度计设备
    spi_bus_.add_device(dev_cfg_acc, acc_handle_);
    // 注册陀螺仪设备
    spi_bus_.add_device(dev_cfg_gyro, gyro_handle_);
}

esp_err_t BMI088::init()
{
    // 初始化陀螺仪
    esp_err_t ret = init_gyroscope();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Gyro init failed");
        return ret;
    }

    // 初始化加速度计
    ret = init_accelerometer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Accelerometer init failed");
        return ret;
    }

    ESP_LOGI(TAG, "BMI088 init success");
    return ESP_OK;
}

esp_err_t BMI088::init_accelerometer()
{
    uint8_t chip_id = 0;
    esp_err_t ret = ESP_OK;

    // 软复位
    ret = spi_bus_.write_reg(acc_handle_, ACC_SOFT_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 读取芯片ID（重试3次）
    for (int i = 0; i < 4; i++) {
        ret = spi_bus_.read_reg(acc_handle_, ACC_CHIP_ID, chip_id);
        if (chip_id == 0x1E)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_LOGW(TAG, "Retry ACC ID read (%d/3), current: 0x%02X", i + 1, chip_id);
    }
    if (chip_id != 0x1E) {
        ESP_LOGE(TAG, "ACC chip ID error: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // 配置加速度计
    ret |= spi_bus_.write_reg(acc_handle_, ACC_IF_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(5));
    ret |= spi_bus_.write_reg(acc_handle_, ACC_PWR_CONF, 0x00);
    vTaskDelay(pdMS_TO_TICKS(20));
    ret |= spi_bus_.write_reg(acc_handle_, ACC_PWR_CTRL, 0x04);
    vTaskDelay(pdMS_TO_TICKS(100));
    ret |= spi_bus_.write_reg(acc_handle_, ACC_CONF, 0x8C);  // 1600Hz输出
    ret |= spi_bus_.write_reg(acc_handle_, ACC_RANGE, 0x03); // ±24g量程
    ret |= spi_bus_.write_reg(acc_handle_, ACC_INT_EN_1, 0x01);

    uint8_t pwr_ctrl = 0;
    for (int i = 0; i < 3; i++) {
        spi_bus_.read_reg(acc_handle_, ACC_PWR_CTRL, pwr_ctrl);
        if (pwr_ctrl == 0x04)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (pwr_ctrl != 0x04) {
        ESP_LOGE(TAG, "ACC not in measurement mode!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMI088 Accelerometer init success (ID: 0x%02X)", chip_id);
    return ret;
}

esp_err_t BMI088::init_gyroscope()
{
    uint8_t chip_id = 0;
    esp_err_t ret = ESP_OK;

    vTaskDelay(pdMS_TO_TICKS(10));
    ret = spi_bus_.read_reg(gyro_handle_, GYRO_CHIP_ID, chip_id);
    if (chip_id != 0x0F) {
        ESP_LOGE(TAG, "GYRO chip ID error: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // 软复位
    ret |= spi_bus_.write_reg(gyro_handle_, 0x14, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(20));

    // 配置陀螺仪
    ret |= spi_bus_.write_reg(gyro_handle_, GYRO_BANDWITH, 0x02); // 1000Hz输出
    ret |= spi_bus_.write_reg(gyro_handle_, GYRO_RANGE, 0x00);    // ±2000°/s量程

    ESP_LOGI(TAG, "BMI088 Gyroscope init success (ID: 0x%02X)", chip_id);
    return ret;
}

esp_err_t BMI088::read_accelerometer(Data &data)
{
    uint8_t acc_data[6] = {0};
    uint8_t status = 0;
    esp_err_t ret = ESP_OK;

    // 等待数据就绪
    int retry = 0;
    do {
        ret = spi_bus_.read_reg(acc_handle_, ACC_STATUS, status);
        retry++;
        vTaskDelay(pdMS_TO_TICKS(1));
    } while ((status & 0x80) == 0 && retry < 8);

    // 数据未就绪用上次数据
    // if (retry >= 8) {
    //     ESP_LOGW(TAG, "ACC data not ready");
    //     data.acc_x = last_acc_data_.acc_x;
    //     data.acc_y = last_acc_data_.acc_y;
    //     data.acc_z = last_acc_data_.acc_z;
    //     return ESP_OK;
    // }

    // 读取加速度数据
    ret = spi_bus_.read_regs(acc_handle_, ACC_DATA_X_LSB, 6, acc_data);
    if (ret == ESP_OK) {
        data.acc_x = (int16_t)((acc_data[1] << 8) | acc_data[0]);
        data.acc_y = (int16_t)((acc_data[3] << 8) | acc_data[2]);
        data.acc_z = (int16_t)((acc_data[5] << 8) | acc_data[4]);
        last_acc_data_ = data;
    }

    return ret;
}

esp_err_t BMI088::read_gyroscope(Data &data)
{

    uint8_t gyro_data[6] = {0};
    esp_err_t ret = spi_bus_.read_regs(gyro_handle_, GYRO_RATE_X_LSB, 6, gyro_data);
    if (ret != ESP_OK) {
        return ret;
    }

    // 原始数据
    int16_t gyro_x = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    int16_t gyro_y = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    int16_t gyro_z = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);

    // 滑动平均滤波
    gyro_x_buf_[filter_idx_] = gyro_x;
    gyro_y_buf_[filter_idx_] = gyro_y;
    gyro_z_buf_[filter_idx_] = gyro_z;
    filter_idx_ = (filter_idx_ + 1) % GYRO_FILTER_WINDOW;

    // 求和平均
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for (size_t i = 0; i < GYRO_FILTER_WINDOW; i++) {
        sum_x += gyro_x_buf_[i];
        sum_y += gyro_y_buf_[i];
        sum_z += gyro_z_buf_[i];
    }

    data.gyro_x = sum_x / GYRO_FILTER_WINDOW;
    data.gyro_y = sum_y / GYRO_FILTER_WINDOW;
    data.gyro_z = sum_z / GYRO_FILTER_WINDOW;

    // 转换为 dps
    data.gyro_x = gyro_x;
    data.gyro_y = gyro_y;
    data.gyro_z = gyro_z;

    return ESP_OK;
}

esp_err_t BMI088::read_data(Data &data)
{
    if (acc_handle_ == nullptr || gyro_handle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = read_accelerometer(data);
    if (ret != ESP_OK)
        return ret;

    ret = read_gyroscope(data);
    return ret;
}

std::string BMI088::Data::to_string() const
{
    char buf[128];
    snprintf(buf,
             sizeof(buf),
             "BMI088: ACC(%.2f,%.2f,%.2f)g | GYRO(%.2f,%.2f,%.2f)dps",
             acc_x_g(),
             acc_y_g(),
             acc_z_g(),
             gyro_x_dps(),
             gyro_y_dps(),
             gyro_z_dps());
    return std::string(buf);
}

BMI088::~BMI088()
{
    // 移除SPI设备
    if (acc_handle_) {
        spi_bus_.remove_device(acc_handle_);
        acc_handle_ = nullptr;
    }
    if (gyro_handle_) {
        spi_bus_.remove_device(gyro_handle_);
        gyro_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "BMI088 device released");
}

} // namespace sensor