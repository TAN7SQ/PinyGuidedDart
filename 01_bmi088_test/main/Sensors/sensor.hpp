#pragma once

#include "bmi088.hpp"
#include "ms5611.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace sensor
{

// 传感器类型枚举
enum class SensorType : uint8_t
{
    BAROMETER = 0,    // 气压计 (MS5611)
    IMU_6AXIS = 1,    // 6轴IMU (BMI088)
    MAG_3AXIS = 2,    // 3轴磁力计 (QMI5883)
    HIGH_G_ACCEL = 3, // 高g值加速度计 (ADXL375BCCZ)
    UNKNOWN = 255
};

// 传感器配置基类
struct SensorConfig
{
    SensorType type = SensorType::UNKNOWN;
    uint32_t update_rate_hz = 0;    // 数据更新频率 (Hz)
    bool enabled = true;            // 是否启用
    uint32_t read_timeout_ms = 100; // 读取超时时间
    std::string name = "";          // 传感器名称

    virtual ~SensorConfig() = default;
    virtual std::string to_string() const;
};

// 气压计配置
struct BarometerConfig : public SensorConfig
{
    uint8_t i2c_address = MS5611_ADDR;    // I2C地址
    OSR osr = OSR::OSR_2048;              // 过采样率
    bool auto_calibrate = true;           // 自动校准
    float sea_level_pressure = 101325.0f; // 海平面气压 (Pa)

    std::string to_string() const override;
};

// IMU配置基类
struct IMUConfig : public SensorConfig
{
    uint8_t accel_range = 0;      // 加速度计量程
    uint8_t gyro_range = 0;       // 陀螺仪量程
    uint8_t filter_bandwidth = 0; // 滤波器带宽

    std::string to_string() const override;
};

// BMI088配置
struct BMI088Config : public IMUConfig
{
    spi::DeviceConfig spi_acc_cfg;
    spi::DeviceConfig spi_gyro_cfg;

    std::string to_string() const override;
};

// TODO:QMI8658配置 (待实现)
struct QMI8658Config : public IMUConfig
{
    uint8_t i2c_address = 0x6B;      // 默认I2C地址
    bool enable_magnetometer = true; // 启用磁力计

    std::string to_string() const;
};

// TODO:ADXL375配置 (待实现)
struct ADXL375Config : public SensorConfig
{
    uint8_t i2c_address = 0x53;    // 默认I2C地址
    uint8_t range = 200;           // 量程 (±200g)
    uint32_t sampling_rate = 3200; // 采样率 (Hz)

    std::string to_string() const;
};

// 传感器数据接口
class ISensorData
{
public:
    virtual ~ISensorData() = default;
    virtual SensorType get_type() const = 0;
    virtual uint64_t get_timestamp_us() const = 0;
    virtual bool is_valid() const = 0;
    virtual std::string to_string() const = 0;
};

// 气压计数据
class BarometerData : public ISensorData
{
public:
    double temperature = 0.0;   // 温度 (°C)
    double pressure_pa = 0.0;   // 压力 (Pa)
    double pressure_mbar = 0.0; // 压力 (mbar)
    double altitude = 0.0;      // 计算高度 (m)
    double sea_level_pressure = 101325.0;
    uint32_t raw_d1 = 0;
    uint32_t raw_d2 = 0;
    uint64_t timestamp_us = 0;

    SensorType get_type() const override
    {
        return SensorType::BAROMETER;
    }
    uint64_t get_timestamp_us() const override
    {
        return timestamp_us;
    }
    bool is_valid() const override
    {
        return pressure_pa > 0 && pressure_pa < 150000;
    }

    std::string to_string() const override;
};

// IMU数据基类
class IMUData : public ISensorData
{
public:
    struct Vector3f
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        float magnitude = 0.0f;

        void calculate_magnitude(); // 计算幅值，用于归一化
        std::string to_string() const;
    };

    struct Vector3i
    {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;
    };

    Vector3i accel_raw;       // 原始加速度数据
    Vector3i gyro_raw;        // 原始陀螺仪数据
    Vector3f accel_g;         // 加速度 (g)
    Vector3f gyro_dps;        // 角速度 (dps)
    Vector3f magnetometer;    // 磁力计数据 (uT) - 3轴磁力计
    float temperature = 0.0f; // 温度
    uint64_t timestamp_us = 0;

    bool is_valid() const override;
    uint64_t get_timestamp_us() const override
    {
        return timestamp_us;
    }

    // 计算所有幅值
    void calculate_magnitudes();
};

// BMI088数据
class BMI088Data : public IMUData
{
public:
    SensorType get_type() const override
    {
        return SensorType::IMU_6AXIS;
    }
    std::string to_string() const override;
};

// 高g值加速度计数据
class HighGAccelData : public ISensorData
{
public:
    struct HighGData
    {
        int16_t raw_x = 0;
        int16_t raw_y = 0;
        int16_t raw_z = 0;
        float x_g = 0.0f;
        float y_g = 0.0f;
        float z_g = 0.0f;
        float magnitude = 0.0f;
        uint8_t range = 0; // 量程 (±g)

        void calculate_magnitude();
        std::string to_string() const;
    };

    HighGData accel;
    uint64_t timestamp_us = 0;

    SensorType get_type() const override
    {
        return SensorType::HIGH_G_ACCEL;
    }
    uint64_t get_timestamp_us() const override
    {
        return timestamp_us;
    }
    bool is_valid() const override
    {
        return true;
    }

    std::string to_string() const override;
};

// 传感器接口
class ISensor
{
public:
    virtual ~ISensor() = default;

    virtual esp_err_t init() = 0;
    virtual esp_err_t read(std::shared_ptr<ISensorData> &data) = 0;
    virtual esp_err_t reset() = 0;

    virtual SensorType get_type() const = 0;
    virtual std::string get_name() const = 0;
    virtual bool is_initialized() const = 0;
    virtual uint32_t get_update_rate_hz() const = 0;

    virtual std::shared_ptr<SensorConfig> get_config() const = 0;
    virtual esp_err_t update_config(const SensorConfig &config) = 0;
};

// 传感器管理器
class SensorManager
{
public:
    // 构造函数
    SensorManager();
    virtual ~SensorManager();

    // 禁止拷贝
    SensorManager(const SensorManager &) = delete;
    SensorManager &operator=(const SensorManager &) = delete;

    esp_err_t init();

    esp_err_t add_barometer(i2c::I2CBus &i2c_bus, const BarometerConfig &config);
    esp_err_t add_bmi088(spi::SPIBus &spi_bus, const BMI088Config &config);
    esp_err_t add_sensor(std::shared_ptr<ISensor> sensor);

    esp_err_t remove_sensor(SensorType type);
    esp_err_t remove_sensor(const std::string &name);

    esp_err_t read_all();
    esp_err_t read_sensor(SensorType type, std::shared_ptr<ISensorData> &data);
    esp_err_t read_sensor(const std::string &name, std::shared_ptr<ISensorData> &data);

    esp_err_t start_async_read(uint32_t interval_ms = 10);
    esp_err_t stop_async_read();

    using DataCallback = std::function<void(SensorType type, const std::shared_ptr<ISensorData> &data)>;
    using AllDataCallback = std::function<void(const std::vector<std::shared_ptr<ISensorData>> &data)>;

    void register_data_callback(DataCallback callback);
    void register_all_data_callback(AllDataCallback callback);
    void register_sensor_callback(SensorType type, DataCallback callback);

    std::shared_ptr<ISensorData> get_latest_data(SensorType type) const;
    std::shared_ptr<ISensorData> get_latest_data(const std::string &name) const;
    std::vector<std::shared_ptr<ISensorData>> get_all_latest_data() const;

    std::shared_ptr<ISensor> get_sensor(SensorType type) const;
    std::shared_ptr<ISensor> get_sensor(const std::string &name) const;
    std::vector<std::shared_ptr<ISensor>> get_all_sensors() const;

    bool is_sensor_present(SensorType type) const;
    bool is_sensor_initialized(SensorType type) const;
    bool is_async_mode() const
    {
        return async_mode_;
    }

    struct Statistics
    {
        struct SensorStats
        {
            std::string name;
            SensorType type;
            uint32_t total_reads = 0;
            uint32_t successful_reads = 0;
            uint32_t error_count = 0;
            float average_read_time_us = 0.0f;
            uint64_t last_read_time_us = 0;
            bool is_enabled = true;
        };

        std::vector<SensorStats> sensor_stats;
        uint64_t start_time_us = 0;
        uint32_t total_read_cycles = 0;

        std::string to_string() const;
    };

    Statistics get_statistics() const;
    void reset_statistics();

    esp_err_t update_sensor_config(SensorType type, const SensorConfig &config);
    esp_err_t enable_sensor(SensorType type, bool enable);

private:
    struct SensorEntry
    {
        std::shared_ptr<ISensor> sensor;
        std::string name;
        uint32_t last_read_time_us = 0;
        uint32_t read_interval_us = 0; // 根据更新频率计算
        std::shared_ptr<ISensorData> latest_data;
        Statistics::SensorStats stats;
        DataCallback callback = nullptr;

        bool should_read(uint64_t current_time_us) const;
    };

    std::vector<SensorEntry> sensors_;
    mutable std::mutex mutex_;

    // 异步模式
    bool async_mode_ = false;
    bool stop_async_ = false;
    TaskHandle_t async_task_handle_ = nullptr;

    // 回调函数
    DataCallback data_callback_ = nullptr;
    AllDataCallback all_data_callback_ = nullptr;

    esp_err_t read_sensor_internal(SensorEntry &entry);
    void async_read_task(void *arg);
    static void async_task_wrapper(void *arg);

    // 工具方法
    SensorEntry *find_sensor_entry(SensorType type);
    SensorEntry *find_sensor_entry(const std::string &name);
    const SensorEntry *find_sensor_entry(SensorType type) const;
    const SensorEntry *find_sensor_entry(const std::string &name) const;

    // 更新统计信息
    void update_sensor_stats(SensorEntry &entry, bool success, uint64_t read_time_us);
};

} // namespace sensor

#pragma once
