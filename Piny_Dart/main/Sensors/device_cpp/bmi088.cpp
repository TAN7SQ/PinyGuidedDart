#include "bmi088.hpp"
#include "bmi088_reg.hpp"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMI088";

static portMUX_TYPE acc_mux = portMUX_INITIALIZER_UNLOCKED;

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
    esp_err_t ret;
    ret = init_accelerometer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Accelerometer init failed");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    ret = init_gyroscope();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Gyro init failed");
        return ret;
    }

    ESP_LOGI(TAG, "BMI088 init success");
    return ESP_OK;
}

esp_err_t BMI088::init_accelerometer()
{
    esp_err_t ret;
    uint8_t chip_id = 0;

    ret = spi_bus_.write_reg(acc_handle_, ACC_SOFT_RESET, 0xB6);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(50));

    for (int i = 0; i < 5; i++) {
        ret = spi_bus_.read_reg(acc_handle_, ACC_CHIP_ID, chip_id);
        if (chip_id == 0x1E)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (chip_id != 0x1E) {
        ESP_LOGE(TAG, "ACC chip ID error: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ACC chip detected");

    ret = spi_bus_.write_reg(acc_handle_, BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON); // active
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = spi_bus_.write_reg(acc_handle_, BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t pwr_ctrl = 0;
    for (int i = 0; i < 10; i++) {
        spi_bus_.read_reg(acc_handle_, BMI088_ACC_PWR_CTRL, pwr_ctrl);
        if (pwr_ctrl & 0x04)
            break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if ((pwr_ctrl & 0x04) == 0) {
        ESP_LOGE(TAG, "ACC failed to enter measurement mode");
        return ESP_FAIL;
    }

    // ret = spi_bus_.write_reg(acc_handle_, ACC_CONF, 0xA7);
    ret = spi_bus_.write_reg(acc_handle_, ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(2));

    ret = spi_bus_.write_reg(acc_handle_, ACC_RANGE, BMI088_ACC_RANGE_12G);
    if (ret != ESP_OK)
        return ret;
    vTaskDelay(pdMS_TO_TICKS(2));

    // ret = ensure_acc_range_24g();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "ACC range 24g init failed");
    //     return ret;
    // }

    ESP_LOGI(TAG, "BMI088 Accelerometer init success (±24g)");
    return ESP_OK;
}

esp_err_t BMI088::ensure_acc_range_24g()
{
    uint8_t raw = 0;
    esp_err_t ret;

    /* Read current range */
    ret = spi_bus_.read_reg(acc_handle_, ACC_RANGE, raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ACC_RANGE");
        return ret;
    }

    uint8_t effective = raw & 0x03;

    ESP_LOGI(TAG, "ACC_RANGE current: raw=0x%02X effective=%u", raw, effective);

    if (effective == 0x00) {
        return ESP_OK;
    }

    ESP_LOGW(TAG, "ACC not in ±24g, forcing ±24g...");

    /* Force set ±24g (write 0x00) */
    ret = spi_bus_.write_reg(acc_handle_, ACC_RANGE, 0x00);
    if (ret != ESP_OK)
        return ret;

    vTaskDelay(pdMS_TO_TICKS(2));

    /* Verify again */
    ret = spi_bus_.read_reg(acc_handle_, ACC_RANGE, raw);
    if (ret != ESP_OK)
        return ret;

    effective = raw & 0x03;

    ESP_LOGI(TAG, "ACC_RANGE after set: raw=0x%02X effective=%u", raw, effective);

    if (effective != 0x00) {
        ESP_LOGE(TAG, "ACC failed to enter ±24g mode!");
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t BMI088::init_gyroscope()
{
    uint8_t chip_id = 0;
    esp_err_t ret = ESP_OK;

    // vTaskDelay(pdMS_TO_TICKS(10));
    // ret = spi_bus_.read_reg(gyro_handle_, GYRO_CHIP_ID, chip_id);
    // if (chip_id != 0x0F) {
    //     ESP_LOGE(TAG, "GYRO chip ID error: 0x%02X", chip_id);
    //     return ESP_FAIL;
    // }

    // 软复位
    ESP_ERROR_CHECK(spi_bus_.write_reg(gyro_handle_, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE));
    vTaskDelay(pdMS_TO_TICKS(30));
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(spi_bus_.read_reg(gyro_handle_, GYRO_CHIP_ID, chip_id));
        if (chip_id == 0x0F)
            break;
        esp_rom_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
        ESP_LOGW(TAG, "Retry GYRO ID read (%d/3), current: 0x%02X", i + 1, chip_id);
    }
    if (chip_id != 0x0F) {
        ESP_LOGE(TAG, "GYRO chip ID error: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    // 配置陀螺仪
    ESP_ERROR_CHECK(spi_bus_.write_reg(
        gyro_handle_, BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set)); // 1000Hz输出
    esp_rom_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ESP_ERROR_CHECK(spi_bus_.write_reg(gyro_handle_, BMI088_GYRO_RANGE, BMI088_GYRO_2000)); // ±2000°/s量程
    esp_rom_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    ESP_ERROR_CHECK(spi_bus_.write_reg(gyro_handle_, BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE));
    vTaskDelay(pdMS_TO_TICKS(30));

    ESP_LOGI(TAG, "BMI088 Gyroscope init success (ID: 0x%02X)", chip_id);
    return ret;
}

esp_err_t BMI088::read_accelerometer(Data &data)
{
    esp_err_t ret;
    uint8_t status = 0;
    uint8_t buf[6] = {0};

    constexpr int MAX_RETRY = 3;
    bool ready = false;

    for (int i = 0; i < MAX_RETRY; i++) {
        ret = spi_bus_.read_reg(acc_handle_, ACC_STATUS, status);
        if (ret != ESP_OK) {
            data = last_acc_data_;
            return ret;
        }

        if (status & 0x80) {
            ready = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    if (!ready) {
        data = last_acc_data_;
        return ESP_OK;
    }

    taskENTER_CRITICAL(&acc_mux);
    ret = spi_bus_.read_regs(acc_handle_, ACC_DATA_X_LSB, 6, buf);
    taskEXIT_CRITICAL(&acc_mux);
    if (ret != ESP_OK) {
        data = last_acc_data_;
        return ret;
    }

    Data new_data = data;

    new_data.acc_x = (int16_t)((buf[1] << 8) | buf[0]);
    new_data.acc_y = (int16_t)((buf[3] << 8) | buf[2]);
    new_data.acc_z = (int16_t)((buf[5] << 8) | buf[4]);

    ESP_LOGI(TAG, "ACC: %d, %d, %d", new_data.acc_x, new_data.acc_y, new_data.acc_z);
    uint8_t range;
    spi_bus_.read_reg(acc_handle_, ACC_RANGE, range);
    ESP_LOGI(TAG, "ACC_RANGE = 0x%02X", range);

    last_acc_data_ = new_data;
    data = new_data;

    return ESP_OK;
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
    if (ret != ESP_OK)
        return ret;

    return ESP_OK;
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