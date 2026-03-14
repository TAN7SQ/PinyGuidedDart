#include "application.hpp"

#include <chrono>
#include <functional>

#include "bmi088.hpp"
#include "spi_bus.hpp"

#include "i2c_bus.hpp"
#include "ms5611.hpp"

#include "freertos/FreeRTOS.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "beeper.hpp"
#include "mlog.hpp"

#include "tfcard.hpp"
#include "ws2812.hpp"

#include "kalman6asix.hpp"

//************************************************************ */
#include "basicInclude.hpp"
#include "control.hpp"
#include "hostpc.hpp"
//************************************************************ */

Beeper *Application::sBeeper = nullptr;

WifiUdpClient *Application::sClient = nullptr;
//************************************************************ */

void LedTask(void *pvParameters)
{

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << GPIO_NUM_38,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t gpio_ret = gpio_config(&gpio_cfg);
    if (gpio_ret != ESP_OK) {
        ESP_LOGE("led_task", "GPIO config failed, err code: %d", gpio_ret);
        vTaskDelete(NULL); // 配置失败则删除当前任务
        return;
    }
    led_strip_handle_t led_strip = configure_led();
    if (led_strip == NULL) {
        ESP_LOGE(Application::TAG, "LED strip configuration failed");
        vTaskDelete(NULL);
        return;
    }
    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI("Led", "\tLedTask start");
    //========================================================
    while (1) {
        gpio_set_level(GPIO_NUM_38, 0);
        ws2812b_RGBOn(led_strip, 0, 20, 20, 20);
        ws2812b_RGBOn(led_strip, 1, 20, 20, 20);
        vTaskDelay(pdMS_TO_TICKS(500));
        ws2812b_Off(led_strip, 0);
        ws2812b_Off(led_strip, 1);
        gpio_set_level(GPIO_NUM_38, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
void KeyTask(void *pvParameters)
{
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1ULL << GPIO_NUM_35,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t gpio_ret = gpio_config(&gpio_cfg);
    if (gpio_ret != ESP_OK) {
        ESP_LOGE("key_task", "GPIO config failed, err code: %d", gpio_ret);
        vTaskDelete(NULL);
        return;
    }
    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI("Key", "\tKeyTask start");
    //========================================================
    while (1) {
        if (gpio_get_level(GPIO_NUM_35) == 1) {
            ESP_LOGI("key_task", "key pressed");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

#include "kalman1D.hpp"
#include "kalmanHeightVelocity.hpp"
void SensorIIcTask(void *pvParameters)
{
    i2c::BusConfig i2c_bus_config = {
        .port = I2C_NUM_1,      //
        .scl_pin = GPIO_NUM_10, //
        .sda_pin = GPIO_NUM_11, //
        .clk_speed_hz = 400000, //
    };
    i2c::I2CBus &i2c_bus = i2c::I2CBus::get_instance(i2c_bus_config);

    // i2c_bus.scan_devices(0x01, 0xdd);
    sensor::MS5611 ms5611(i2c_bus, 0x77, sensor::OSR::OSR_4096);

    esp_err_t ret = ms5611.init();

    if (ret != ESP_OK) {
        ESP_LOGE(Application::TAG, "MS5611 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    // kalmanHeight kfHeight;
    Kalman1D kfHeight;
    kfHeight.init(0.0);

    const uint16_t DT = 0.02;
    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI("IIC", "SensorIIcTask start");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(DT * 1000));
        sensor::MS5611::ConvertData data;
        ret = ms5611.read_data(data);
        if (ret != ESP_OK) {
            ESP_LOGE(Application::TAG, "MS5611 read data failed: %s", esp_err_to_name(ret));
            continue;
        }

        kfHeight.update(data.height);
        // double height = kfHeight.getHeight();
        // ESP_LOGI(Application::TAG, "%.4f", height);

        BaseType_t ret = xQueueSend(rtoshandler.BaroQueue, &data, 0);
        if (ret != pdPASS) {
            ESP_LOGE(Application::TAG, "BaroQueue send failed");
        }
    }
}

#include "AuxiliaryMath.hpp"
#include "calibrate.hpp"
#include "kalman6asix.hpp"

#include "adxl375.hpp"
void SensorSpiTask(void *pvParameters)
{
    spi::BusConfig spi_bus_config = {
        .host_num = SPI2_HOST,
        .sclk_pin = GPIO_NUM_33, //
        .mosi_pin = GPIO_NUM_34, //
        .miso_pin = GPIO_NUM_48, //
    };
    spi::SPIBus::get_instance(spi_bus_config);

    spi::DeviceConfig bmi088_acc_cfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .cs_pin = GPIO_NUM_47,
    };
    spi::DeviceConfig bmi088_gyro_cfg = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .cs_pin = GPIO_NUM_37,
    };
    sensor::BMI088 bmi088(bmi088_acc_cfg, bmi088_gyro_cfg);
    esp_err_t ret = bmi088.init();
    if (ret != ESP_OK) {
        ESP_LOGE(Application::TAG, "BMI088 initialization failed: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    //========================================================
    // spi::DeviceConfig adxl375_cfg = {
    //     .clock_speed_hz = 2 * 1000 * 1000,
    //     .cs_pin = GPIO_NUM_40,
    //     .mode = 3,
    // };
    // sensor::ADXL375 adxl375(adxl375_cfg);
    // ret = adxl375.init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE(Application::TAG, "ADXL375 initialization failed: %s", esp_err_to_name(ret));
    //     vTaskDelete(NULL);
    //     return;
    // }
    // uint8_t id = 0;
    // ret = adxl375.read_device_id(id);
    // ESP_LOGI(Application::TAG, "ADXL375 device id: 0x%02x", id);

    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI("Spi", "\tSensorSpiTask start");
    //========================================================
    const float IMU_UPDATE_DT = 0.01f;

    sensor::BMI088::Data data;
    ret = bmi088.read_data(data);
    //========================================================
    AttitudeEKF ekf;
    AuxMath::Vec3 accVec3(data.acc_x_g(), data.acc_y_g(), data.acc_z_g());
    ekf.Init(accVec3);
    //========================================================

    IMUCalibration imu_calibration;
    imu_calibration.init(ACC_CALI, GYRO_CALI);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(IMU_UPDATE_DT * 1000));
        ret = bmi088.read_data(data);
        if (ret != ESP_OK) {
            ESP_LOGE(Application::TAG, "BMI088 read data failed: %s", esp_err_to_name(ret));
            continue;
        }

        imu_calibration.correctA(data.acc_x, data.acc_y, data.acc_z);
        CaliOutput_s out = imu_calibration.getOutput();
        sensor::BMI088::Data corrected_data = data;
        corrected_data.acc_x = out.ax;
        corrected_data.acc_y = out.ay;
        corrected_data.acc_z = out.az;

        AuxMath::Quat q;
        AuxMath::Vec3 accVec3(corrected_data.acc_x_g(), corrected_data.acc_y_g(), corrected_data.acc_z_g());
        AuxMath::Vec3 gyroVec3(data.gyro_x_rads(), data.gyro_y_rads(), data.gyro_z_rads());
        ekf.CalculateAccelOnlyEuler(accVec3);
        ekf.StaticDetect(gyroVec3, accVec3);
        ekf.Update(accVec3);
        ekf.Predict(gyroVec3, IMU_UPDATE_DT);
        ekf.GetAttitude(q);
        AuxMath::Vec3 euler;
        AuxMath::QuatToEuler(q, euler);
        xAxisIMU::IMUAttitude imu_attitude;
        imu_attitude.euler = euler;
        imu_attitude.quat = q;

        xAxisIMU::IMURawData imu_raw_data;
        imu_raw_data.acc = accVec3;
        imu_raw_data.gyro = gyroVec3;
        BaseType_t ret = xQueueSend(rtoshandler.imuQueueRaw, &imu_raw_data, pdMS_TO_TICKS(1));
        ret |= xQueueSend(rtoshandler.imuQueueFiltered, &imu_attitude, pdMS_TO_TICKS(1));
        if (ret != pdPASS) {
            continue; // It doesn't happen in theory
        }
    }
}

// 异步记录日志，同时使用串口发送、http发送
void LogTask(void *pvParameters)
{
    xAxisIMU::IMUAttitude imuAttitude;

    // TF_Card *tfCard = (TF_Card *)pvParameters;
    //========================================================
    xSemaphoreGive(rtoshandler.InitCountSem);
    xEventGroupWaitBits(rtoshandler.StartSyncGroup, START_SYNC_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    //========================================================
    ESP_LOGI("LOG", "\tLogTask started");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        // TODO: 设置队列，并异步地写入TF卡，句柄应该通过参数来传递

        BaseType_t ret = xQueuePeek(rtoshandler.imuQueueFiltered, &imuAttitude, 0);
        if (ret != pdPASS) {
            continue;
        }
        else {
            char imu_attitude_str[256];
            sprintf(imu_attitude_str,
                    "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                    imuAttitude.quat.w,
                    imuAttitude.quat.x,
                    imuAttitude.quat.y,
                    imuAttitude.quat.z,
                    imuAttitude.euler.x,
                    imuAttitude.euler.y,
                    imuAttitude.euler.z);

            Application::sClient->sendData((const uint8_t *)imu_attitude_str, strlen(imu_attitude_str));
        }
    }
}

void Application::Initialize()
{
    ESP_ERROR_CHECK(rtosHandlerInit());

    Control::GetInstance();
    HostPC::GetInstance();

    vTaskSuspendAll();

    /************************ LOG SYSTEM INITIALIZE ************************/

    // esp_err_t tf_ret = this->tfCard.Initialize();
    // if (tf_ret != ESP_OK) {
    //     ESP_LOGE(Application::TAG, "TF card initialization failed: %s", esp_err_to_name(tf_ret));
    //     return;
    // }
    // ESP_LOGI(Application::TAG, "TF card initialized successfully");

    // LogTaskParams_t log_task_params = {
    //     .tf_card_ptr = &this->tfCard, //
    //     .log_queue = this->xLogQueue  //
    // };
    // Tools::taskCreate(LogTask, "log_task", 4096, &log_task_params, tskIDLE_PRIORITY + 1, NULL, 1);

    /************************  ************************/
    Tools::taskCreate(LedTask, "led_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 1);
    Tools::taskCreate(KeyTask, "key_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL, 1);

    Tools::taskCreate(SensorIIcTask, "SensorIIcTask", 8192, NULL, tskIDLE_PRIORITY + 2, NULL, 0);
    Tools::taskCreate(SensorSpiTask, "SensorSpiTask", 10096, &this->xSpiSensorQueue, tskIDLE_PRIORITY + 2, NULL, 0);

    Control::GetInstance().start();
    HostPC::GetInstance().start();

    /************************  ************************/
    xTaskCreatePinnedToCore(Tools::AppManagerTask, "app_manager_task", 4096, NULL, tskIDLE_PRIORITY + 3, NULL, 0);
    xTaskResumeAll();

    /************************ 无线开关 ************************/
    // auto beeper_cb = []() {
    //     Application::sBeeper->play_run_music();
    // };
    // esp_err_t ret = this->client.init(client_config, beeper_cb);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(Application::TAG, "WifiUdpClient init failed: %s", esp_err_to_name(ret));
    //     return;
    // }
}

void Application::Run()
{
    // this->beeper.play_run_music();

    ESP_LOGI(Application::TAG, "App run");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

Application::Application()
    : beeper(GPIO_NUM_21), //
      client(WifiUdpClient::getInstance())
{
    Application::sBeeper = &this->beeper;
    this->beeper.play_boot_music();

    Application::sClient = &this->client;
    client_config = {
        .wifi_ssid = "TAN",
        .wifi_pass = "11111111",

        .udp_server_ip = "192.168.137.1",
        .udp_server_port = 8080,

        .static_ip = "192.168.137.99", // 除了最后一位，其他必须和服务器网关相同
        .static_gw = "192.168.137.1",
        .static_netmask = "255.255.255.0",
    };
    /*********************************************************** */

    ESP_LOGI(Application::TAG, "App start");
}

Application::~Application()
{
}
