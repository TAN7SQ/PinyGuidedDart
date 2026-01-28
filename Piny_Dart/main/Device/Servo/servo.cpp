#include "servo.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include <cstdint>

Servo::Servo()
{
    ESP_LOGI(TAG, "Servo init");
    servo_channel_t servo_handle = {
        .servo_pin =
            {
                GPIO_NUM_13, // LF0
                GPIO_NUM_14, // LF1
                GPIO_NUM_15, // RF0
                GPIO_NUM_16, // RF1

                // GPIO_NUM_17, // LB0
                // GPIO_NUM_18, // LB1
                // GPIO_NUM_3,  // RB0
                // GPIO_NUM_41, // RB1
            },
        .ch =
            {
                LEDC_CHANNEL_0,
                LEDC_CHANNEL_1,
                LEDC_CHANNEL_2,
                LEDC_CHANNEL_3,
                // LEDC_CHANNEL_4,
                // LEDC_CHANNEL_5,
                // LEDC_CHANNEL_6,
                // LEDC_CHANNEL_7,
            },
    };
    MaxChNum = 4;

    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = LEDC_TIMER_0,
        .channels = servo_handle,
        .channel_number = 4,
    };

    esp_err_t err = iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init low speed timer 2 failed %s", esp_err_to_name(err));
    }
    else {
        ESP_LOGI(TAG, "init low speed timer 2 success");
    }
}

esp_err_t Servo::Initialize()
{
    esp_err_t ret = ESP_OK;
    uint16_t calibration_value_0 = 30;
    uint16_t calibration_value_180 = 195;
    for (int i = calibration_value_0; i <= calibration_value_180; i += 1) {
        for (uint8_t ch = 0; ch < MaxChNum; ch++)
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ch, i);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    // Return to the initial position
    for (uint8_t ch = 0; ch < MaxChNum; ch++)
        ret |= iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ch, calibration_value_0);
    return ret;
}

esp_err_t Servo::SetAngle(ServoCH_e Servo, int angle)
{
    if (Servo >= MaxChNum || Servo >= ALL)
        return ESP_ERR_INVALID_ARG;
    if (Servo == ALL) {
        for (uint8_t ch = 0; ch < MaxChNum; ch++)
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ch, angle);
    }
    else
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, Servo, angle);
    return ESP_OK;
}

esp_err_t Servo::ReadAngle(ServoCH_e Servo, float &angle)
{
    if (Servo >= MaxChNum || Servo >= ALL)
        return ESP_ERR_INVALID_ARG;
    return iot_servo_read_angle(LEDC_LOW_SPEED_MODE, Servo, &angle);
}