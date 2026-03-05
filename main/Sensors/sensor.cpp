#include "sensor.hpp"
#include <cmath>
#include <esp_task.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace sensor
{

// ========================================
std::string SensorConfig::to_string() const
{
    char buffer[128];
    const char *type_str = "UNKNOWN";
    switch (type) {
    case SensorType::BAROMETER:
        type_str = "BAROMETER";
        break;
    case SensorType::IMU_6AXIS:
        type_str = "IMU_6AXIS";
        break;
    case SensorType::MAG_3AXIS:
        type_str = "MAG_3AXIS";
        break;
    case SensorType::HIGH_G_ACCEL:
        type_str = "HIGH_G_ACCEL";
        break;
    default:
        break;
    }

    snprintf(buffer,
             sizeof(buffer),
             "Type: %s | Name: %s | Rate: %u Hz | Enabled: %s",
             type_str,
             name.c_str(),
             update_rate_hz,
             enabled ? "Yes" : "No");
    return std::string(buffer);
}

std::string BarometerConfig::to_string() const
{
    char buffer[256];
    snprintf(buffer,
             sizeof(buffer),
             "%s | I2C: 0x%02X | OSR: %d | AutoCal: %s",
             SensorConfig::to_string().c_str(),
             i2c_address,
             static_cast<int>(osr),
             auto_calibrate ? "Yes" : "No");
    return std::string(buffer);
}

std::string IMUConfig::to_string() const
{
    char buffer[256];
    snprintf(buffer,
             sizeof(buffer),
             "%s | Accel Range: %d | Gyro Range: %d | BW: %d",
             SensorConfig::to_string().c_str(),
             accel_range,
             gyro_range,
             filter_bandwidth);
    return std::string(buffer);
}

std::string BMI088Config::to_string() const
{
    char buffer[384];
    snprintf(buffer,
             sizeof(buffer),
             "%s | Acc CS: %d | Gyro CS: %d",
             IMUConfig::to_string().c_str(),
             spi_acc_cfg.cs_pin,
             spi_gyro_cfg.cs_pin);
    return std::string(buffer);
}

std::string QMI8658Config::to_string() const
{
    char buffer[256];
    snprintf(buffer,
             sizeof(buffer),
             "%s | I2C: 0x%02X | Mag: %s",
             IMUConfig::to_string().c_str(),
             i2c_address,
             enable_magnetometer ? "Enabled" : "Disabled");
    return std::string(buffer);
}

std::string ADXL375Config::to_string() const
{
    char buffer[256];
    snprintf(buffer,
             sizeof(buffer),
             "%s | I2C: 0x%02X | Range: ±%dg | Rate: %u Hz",
             SensorConfig::to_string().c_str(),
             i2c_address,
             range,
             sampling_rate);
    return std::string(buffer);
}

// ==================== 数据类实现 ====================
// BarometerData
std::string BarometerData::to_string() const
{
    char buffer[256];
    snprintf(buffer,
             sizeof(buffer),
             "BARO | Temp: %.2f°C | Press: %.2f Pa (%.2f mbar) | "
             "Alt: %.2f m | Valid: %s",
             temperature,
             pressure_pa,
             pressure_mbar,
             altitude,
             is_valid() ? "Yes" : "No");
    return std::string(buffer);
}

// IMUData::Vector3f
void IMUData::Vector3f::calculate_magnitude()
{
    magnitude = sqrtf(x * x + y * y + z * z);
}

std::string IMUData::Vector3f::to_string() const
{
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "X: %.3f, Y: %.3f, Z: %.3f (Mag: %.3f)", x, y, z, magnitude);
    return std::string(buffer);
}

// IMUData
bool IMUData::is_valid() const
{
    // 简单的有效性检查
    return !(std::isnan(accel_g.x) || std::isnan(gyro_dps.x));
}

void IMUData::calculate_magnitudes()
{
    accel_g.calculate_magnitude();
    gyro_dps.calculate_magnitude();
    if (magnetometer.x != 0 || magnetometer.y != 0 || magnetometer.z != 0) {
        magnetometer.calculate_magnitude();
    }
}

// BMI088Data
std::string BMI088Data::to_string() const
{
    char buffer[512];
    snprintf(buffer,
             sizeof(buffer),
             "BMI088 | Accel: %s g | Gyro: %s dps | Temp: %.1f°C | Valid: %s",
             accel_g.to_string().c_str(),
             gyro_dps.to_string().c_str(),
             temperature,
             is_valid() ? "Yes" : "No");
    return std::string(buffer);
}

// HighGAccelData::HighGData
void HighGAccelData::HighGData::calculate_magnitude()
{
    magnitude = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
}

std::string HighGAccelData::HighGData::to_string() const
{
    char buffer[192];
    snprintf(buffer, sizeof(buffer), "X: %.1f, Y: %.1f, Z: %.1f (Mag: %.1f)g @ ±%dg", x_g, y_g, z_g, magnitude, range);
    return std::string(buffer);
}

// HighGAccelData
std::string HighGAccelData::to_string() const
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "ADXL375 | %s | Valid: %s", accel.to_string().c_str(), is_valid() ? "Yes" : "No");
    return std::string(buffer);
}

// ==================== SensorManager 实现 ====================
SensorManager::SensorManager()
{
}

SensorManager::~SensorManager()
{
    stop_async_read();

    std::lock_guard<std::mutex> lock(mutex_);
    sensors_.clear();
}

esp_err_t SensorManager::init()
{
    reset_statistics();
    return ESP_OK;
}

esp_err_t SensorManager::add_barometer(i2c::I2CBus &i2c_bus, const BarometerConfig &config)
{
    try {
        // 创建MS5611实例
        auto barometer = std::make_shared<MS5611>(i2c_bus, config.i2c_address, config.osr);

        // 初始化
        esp_err_t ret = barometer->init();
        if (ret != ESP_OK) {
            printf("Failed to initialize barometer: %s\n", esp_err_to_name(ret));
            return ret;
        }

        // 包装为ISensor接口
        class BarometerSensor : public ISensor
        {
        private:
            std::shared_ptr<MS5611> sensor_;
            BarometerConfig config_;

        public:
            BarometerSensor(std::shared_ptr<MS5611> sensor, const BarometerConfig &config)
                : sensor_(sensor), config_(config)
            {
            }

            esp_err_t init() override
            {
                return sensor_->init();
            }

            esp_err_t read(std::shared_ptr<ISensorData> &data) override
            {
                auto baro_data = std::make_shared<BarometerData>();
                MS5611::Data raw_data;

                esp_err_t ret = sensor_->read_data(raw_data);
                if (ret == ESP_OK) {
                    baro_data->temperature = raw_data.temperature;
                    baro_data->pressure_pa = raw_data.pressure_pa;
                    baro_data->pressure_mbar = raw_data.pressure_mbar;
                    baro_data->raw_d1 = raw_data.raw_d1;
                    baro_data->raw_d2 = raw_data.raw_d2;
                    baro_data->sea_level_pressure = config_.sea_level_pressure;
                    baro_data->altitude =
                        SensorManager::calculate_altitude(raw_data.pressure_pa, config_.sea_level_pressure);
                    baro_data->timestamp_us = esp_timer_get_time();
                }

                data = baro_data;
                return ret;
            }

            esp_err_t reset() override
            {
                return sensor_->reset();
            }

            SensorType get_type() const override
            {
                return SensorType::BAROMETER;
            }
            std::string get_name() const override
            {
                return config_.name;
            }
            bool is_initialized() const override
            {
                return sensor_->is_calibration_valid();
            }
            uint32_t get_update_rate_hz() const override
            {
                return config_.update_rate_hz;
            }

            std::shared_ptr<SensorConfig> get_config() const override
            {
                return std::make_shared<BarometerConfig>(config_);
            }

            esp_err_t update_config(const SensorConfig &config) override
            {
                if (config.type != SensorType::BAROMETER)
                    return ESP_ERR_INVALID_ARG;

                config_ = static_cast<const BarometerConfig &>(config);
                return ESP_OK;
            }
        };

        auto sensor = std::make_shared<BarometerSensor>(barometer, config);

        // 添加到管理器
        return add_sensor(sensor);
    } catch (const std::exception &e) {
        printf("Exception creating barometer: %s\n", e.what());
        return ESP_FAIL;
    }
}

esp_err_t SensorManager::add_bmi088(spi::SPIBus &spi_bus, const BMI088Config &config)
{
    try {
        // 创建BMI088实例
        auto bmi088 = std::make_shared<BMI088>(config.spi_acc_cfg, config.spi_gyro_cfg);

        // 初始化
        esp_err_t ret = bmi088->init();
        if (ret != ESP_OK) {
            printf("Failed to initialize BMI088: %s\n", esp_err_to_name(ret));
            return ret;
        }

        // 包装为ISensor接口
        class BMI088Sensor : public ISensor
        {
        private:
            std::shared_ptr<BMI088> sensor_;
            BMI088Config config_;

        public:
            BMI088Sensor(std::shared_ptr<BMI088> sensor, const BMI088Config &config) : sensor_(sensor), config_(config)
            {
            }

            esp_err_t init() override
            {
                return sensor_->init();
            }

            esp_err_t read(std::shared_ptr<ISensorData> &data) override
            {
                auto imu_data = std::make_shared<BMI088Data>();
                BMI088::Data raw_data;

                esp_err_t ret = sensor_->read_data(raw_data);
                if (ret == ESP_OK) {
                    // 原始数据
                    imu_data->accel_raw.x = raw_data.acc_x;
                    imu_data->accel_raw.y = raw_data.acc_y;
                    imu_data->accel_raw.z = raw_data.acc_z;
                    imu_data->gyro_raw.x = raw_data.gyro_x;
                    imu_data->gyro_raw.y = raw_data.gyro_y;
                    imu_data->gyro_raw.z = raw_data.gyro_z;

                    // 物理单位数据
                    imu_data->accel_g.x = raw_data.acc_x_g();
                    imu_data->accel_g.y = raw_data.acc_y_g();
                    imu_data->accel_g.z = raw_data.acc_z_g();
                    imu_data->gyro_dps.x = raw_data.gyro_x_dps();
                    imu_data->gyro_dps.y = raw_data.gyro_y_dps();
                    imu_data->gyro_dps.z = raw_data.gyro_z_dps();

                    // 计算幅值
                    imu_data->calculate_magnitudes();

                    imu_data->timestamp_us = esp_timer_get_time();
                }

                data = imu_data;
                return ret;
            }

            esp_err_t reset() override
            {
                // BMI088没有reset接口，重新初始化
                return sensor_->init();
            }

            SensorType get_type() const override
            {
                return SensorType::IMU_6AXIS;
            }
            std::string get_name() const override
            {
                return config_.name;
            }
            bool is_initialized() const override
            {
                return true;
            }
            uint32_t get_update_rate_hz() const override
            {
                return config_.update_rate_hz;
            }

            std::shared_ptr<SensorConfig> get_config() const override
            {
                return std::make_shared<BMI088Config>(config_);
            }

            esp_err_t update_config(const SensorConfig &config) override
            {
                if (config.type != SensorType::IMU_6AXIS)
                    return ESP_ERR_INVALID_ARG;

                config_ = static_cast<const BMI088Config &>(config);
                return ESP_OK;
            }
        };

        auto sensor = std::make_shared<BMI088Sensor>(bmi088, config);

        // 添加到管理器
        return add_sensor(sensor);
    } catch (const std::exception &e) {
        printf("Exception creating BMI088: %s\n", e.what());
        return ESP_FAIL;
    }
}

esp_err_t SensorManager::add_sensor(std::shared_ptr<ISensor> sensor)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 检查是否已存在同类型传感器
    for (const auto &entry : sensors_) {
        if (entry.sensor->get_type() == sensor->get_type() && entry.sensor->get_name() == sensor->get_name()) {
            printf("Sensor %s already exists\n", sensor->get_name().c_str());
            return ESP_ERR_INVALID_STATE;
        }
    }

    // 创建传感器条目
    SensorEntry entry;
    entry.sensor = sensor;
    entry.name = sensor->get_name();
    entry.read_interval_us = sensor->get_update_rate_hz() > 0 ? (1000000 / sensor->get_update_rate_hz()) : 0;

    entry.stats.name = sensor->get_name();
    entry.stats.type = sensor->get_type();
    entry.stats.is_enabled = true;

    sensors_.push_back(entry);

    printf("Added sensor: %s (Type: %d, Rate: %u Hz)\n",
           entry.name.c_str(),
           static_cast<int>(sensor->get_type()),
           sensor->get_update_rate_hz());

    return ESP_OK;
}

esp_err_t SensorManager::remove_sensor(SensorType type)
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
        if (it->sensor->get_type() == type) {
            printf("Removed sensor: %s\n", it->name.c_str());
            sensors_.erase(it);
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t SensorManager::remove_sensor(const std::string &name)
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto it = sensors_.begin(); it != sensors_.end(); ++it) {
        if (it->name == name) {
            printf("Removed sensor: %s\n", it->name.c_str());
            sensors_.erase(it);
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t SensorManager::read_all()
{
    std::vector<std::shared_ptr<ISensorData>> all_data;
    uint64_t current_time = esp_timer_get_time();

    std::lock_guard<std::mutex> lock(mutex_);

    for (auto &entry : sensors_) {
        if (!entry.stats.is_enabled)
            continue;

        if (entry.should_read(current_time)) {
            std::shared_ptr<ISensorData> data;
            esp_err_t ret = read_sensor_internal(entry);

            if (ret == ESP_OK && entry.latest_data) {
                all_data.push_back(entry.latest_data);

                // 调用传感器特定回调
                if (entry.callback) {
                    entry.callback(entry.sensor->get_type(), entry.latest_data);
                }

                // 调用通用数据回调
                if (data_callback_) {
                    data_callback_(entry.sensor->get_type(), entry.latest_data);
                }
            }
        }
    }

    // 调用所有数据回调
    if (!all_data.empty() && all_data_callback_) {
        all_data_callback_(all_data);
    }

    return ESP_OK;
}

esp_err_t SensorManager::read_sensor(SensorType type, std::shared_ptr<ISensorData> &data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto entry = find_sensor_entry(type);
    if (!entry)
        return ESP_ERR_NOT_FOUND;

    return read_sensor_internal(*entry);
}

esp_err_t SensorManager::read_sensor(const std::string &name, std::shared_ptr<ISensorData> &data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto entry = find_sensor_entry(name);
    if (!entry)
        return ESP_ERR_NOT_FOUND;

    return read_sensor_internal(*entry);
}

esp_err_t SensorManager::start_async_read(uint32_t interval_ms)
{
    if (async_mode_)
        return ESP_ERR_INVALID_STATE;

    stop_async_ = false;
    async_mode_ = true;

    // 创建异步读取任务
    BaseType_t result = xTaskCreatePinnedToCore(async_task_wrapper,
                                                "sensor_async_read",
                                                4096,
                                                this,
                                                configMAX_PRIORITIES - 2,
                                                &async_task_handle_,
                                                0 // 通常放在PRO核心
    );

    if (result != pdPASS) {
        async_mode_ = false;
        return ESP_ERR_NO_MEM;
    }

    printf("Started async sensor reading with interval %u ms\n", interval_ms);
    return ESP_OK;
}

esp_err_t SensorManager::stop_async_read()
{
    if (!async_mode_)
        return ESP_OK;

    stop_async_ = true;
    async_mode_ = false;

    if (async_task_handle_ != nullptr) {
        vTaskDelete(async_task_handle_);
        async_task_handle_ = nullptr;
    }

    printf("Stopped async sensor reading\n");
    return ESP_OK;
}

void SensorManager::register_data_callback(DataCallback callback)
{
    std::lock_guard<std::mutex> lock(mutex_);
    data_callback_ = std::move(callback);
}

void SensorManager::register_all_data_callback(AllDataCallback callback)
{
    std::lock_guard<std::mutex> lock(mutex_);
    all_data_callback_ = std::move(callback);
}

void SensorManager::register_sensor_callback(SensorType type, DataCallback callback)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto entry = find_sensor_entry(type);
    if (entry) {
        entry->callback = std::move(callback);
    }
}

std::shared_ptr<ISensorData> SensorManager::get_latest_data(SensorType type) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto entry = find_sensor_entry(type);
    if (entry) {
        return entry->latest_data;
    }

    return nullptr;
}

std::shared_ptr<ISensorData> SensorManager::get_latest_data(const std::string &name) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto entry = find_sensor_entry(name);
    if (entry) {
        return entry->latest_data;
    }

    return nullptr;
}

std::vector<std::shared_ptr<ISensorData>> SensorManager::get_all_latest_data() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::shared_ptr<ISensorData>> all_data;
    for (const auto &entry : sensors_) {
        if (entry.latest_data) {
            all_data.push_back(entry.latest_data);
        }
    }

    return all_data;
}

std::shared_ptr<ISensor> SensorManager::get_sensor(SensorType type) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto entry = find_sensor_entry(type);
    if (entry) {
        return entry->sensor;
    }

    return nullptr;
}

std::shared_ptr<ISensor> SensorManager::get_sensor(const std::string &name) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto entry = find_sensor_entry(name);
    if (entry) {
        return entry->sensor;
    }

    return nullptr;
}

std::vector<std::shared_ptr<ISensor>> SensorManager::get_all_sensors() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<std::shared_ptr<ISensor>> all_sensors;
    for (const auto &entry : sensors_) {
        all_sensors.push_back(entry.sensor);
    }

    return all_sensors;
}

bool SensorManager::is_sensor_present(SensorType type) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return find_sensor_entry(type) != nullptr;
}

bool SensorManager::is_sensor_initialized(SensorType type) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    const auto entry = find_sensor_entry(type);
    if (entry) {
        return entry->sensor->is_initialized();
    }

    return false;
}

SensorManager::Statistics SensorManager::get_statistics() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    Statistics stats;
    stats.start_time_us = esp_timer_get_time();
    stats.total_read_cycles = 0; // 这需要额外跟踪

    for (const auto &entry : sensors_) {
        stats.sensor_stats.push_back(entry.stats);
    }

    return stats;
}

void SensorManager::reset_statistics()
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto &entry : sensors_) {
        entry.stats.total_reads = 0;
        entry.stats.successful_reads = 0;
        entry.stats.error_count = 0;
        entry.stats.average_read_time_us = 0.0f;
    }
}

esp_err_t SensorManager::update_sensor_config(SensorType type, const SensorConfig &config)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto entry = find_sensor_entry(type);
    if (!entry)
        return ESP_ERR_NOT_FOUND;

    return entry->sensor->update_config(config);
}

esp_err_t SensorManager::enable_sensor(SensorType type, bool enable)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto entry = find_sensor_entry(type);
    if (!entry)
        return ESP_ERR_NOT_FOUND;

    entry->stats.is_enabled = enable;
    return ESP_OK;
}

bool SensorManager::SensorEntry::should_read(uint64_t current_time_us) const
{
    if (!stats.is_enabled || read_interval_us == 0)
        return false;

    return (current_time_us - last_read_time_us) >= read_interval_us;
}

esp_err_t SensorManager::read_sensor_internal(SensorEntry &entry)
{
    uint64_t start_time = esp_timer_get_time();
    std::shared_ptr<ISensorData> data;

    esp_err_t ret = entry.sensor->read(data);
    uint64_t read_time = esp_timer_get_time() - start_time;

    if (ret == ESP_OK && data) {
        entry.latest_data = data;
        entry.last_read_time_us = esp_timer_get_time();
        update_sensor_stats(entry, true, read_time);
    }
    else {
        update_sensor_stats(entry, false, read_time);
    }

    return ret;
}

void SensorManager::async_read_task(void *arg)
{
    SensorManager *manager = static_cast<SensorManager *>(arg);

    while (!manager->stop_async_) {
        manager->read_all();
        vTaskDelay(pdMS_TO_TICKS(1)); // 1ms最小间隔
    }

    vTaskDelete(nullptr);
}

void SensorManager::async_task_wrapper(void *arg)
{
    static_cast<SensorManager *>(arg)->async_read_task(arg);
}

SensorManager::SensorEntry *SensorManager::find_sensor_entry(SensorType type)
{
    for (auto &entry : sensors_) {
        if (entry.sensor->get_type() == type)
            return &entry;
    }
    return nullptr;
}

SensorManager::SensorEntry *SensorManager::find_sensor_entry(const std::string &name)
{
    for (auto &entry : sensors_) {
        if (entry.name == name)
            return &entry;
    }
    return nullptr;
}

const SensorManager::SensorEntry *SensorManager::find_sensor_entry(SensorType type) const
{
    for (const auto &entry : sensors_) {
        if (entry.sensor->get_type() == type)
            return &entry;
    }
    return nullptr;
}

const SensorManager::SensorEntry *SensorManager::find_sensor_entry(const std::string &name) const
{
    for (const auto &entry : sensors_) {
        if (entry.name == name)
            return &entry;
    }
    return nullptr;
}

void SensorManager::update_sensor_stats(SensorEntry &entry, bool success, uint64_t read_time_us)
{
    entry.stats.total_reads++;

    if (success) {
        entry.stats.successful_reads++;

        // 更新平均读取时间（指数移动平均）
        if (entry.stats.average_read_time_us == 0.0f) {
            entry.stats.average_read_time_us = static_cast<float>(read_time_us);
        }
        else {
            entry.stats.average_read_time_us =
                0.9f * entry.stats.average_read_time_us + 0.1f * static_cast<float>(read_time_us);
        }
    }
    else {
        entry.stats.error_count++;
    }

    entry.stats.last_read_time_us = esp_timer_get_time();
}

std::string SensorManager::Statistics::to_string() const
{
    std::string result = "Sensor Statistics:\n";

    for (const auto &stat : sensor_stats) {
        char buffer[256];
        const char *type_str = "UNKNOWN";
        switch (stat.type) {
        case SensorType::BAROMETER:
            type_str = "BARO";
            break;
        case SensorType::IMU_6AXIS:
            type_str = "IMU6";
            break;
        case SensorType::IMU_9AXIS:
            type_str = "IMU9";
            break;
        case SensorType::HIGH_G_ACCEL:
            type_str = "HGAC";
            break;
        default:
            break;
        }

        float success_rate = stat.total_reads > 0 ? (100.0f * stat.successful_reads / stat.total_reads) : 0.0f;

        snprintf(buffer,
                 sizeof(buffer),
                 "  %s [%s]: Reads: %u, Success: %.1f%%, "
                 "Errors: %u, AvgTime: %.1f us, Enabled: %s\n",
                 stat.name.c_str(),
                 type_str,
                 stat.total_reads,
                 success_rate,
                 stat.error_count,
                 stat.average_read_time_us,
                 stat.is_enabled ? "Yes" : "No");

        result += buffer;
    }

    return result;
}

} // namespace sensor