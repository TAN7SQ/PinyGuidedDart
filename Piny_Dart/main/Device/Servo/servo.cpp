#include "servo.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"

Servo::Servo()
{
    ledc_timer_config_t timer = {
        .speed_mode = MODE,
        .duty_resolution = RES,
        .timer_num = TIMER,
        .freq_hz = SERVO_FREQ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < NUM_SERVO; i++) {
        ledc_channel_config_t ch = {
            .gpio_num = PINS[i],
            .speed_mode = MODE,
            .channel = static_cast<ledc_channel_t>(i),
            .timer_sel = TIMER,
            .duty = us_to_duty(1500),
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
    }

    ESP_LOGI(TAG, "Servo driver initialized");
}

uint32_t Servo::us_to_duty(uint32_t us)
{
    uint32_t period_us = 1000000 / SERVO_FREQ;
    uint32_t max_duty = (1 << RES) - 1;
    return (us * max_duty) / period_us;
}

esp_err_t Servo::SetAngle(ServoCH_e ch, int angle)
{
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    auto apply = [&](int i) {
        const ServoLimit &lim = limits[i];

        uint32_t pulse = lim.min_us + (lim.max_us - lim.min_us) * angle / 180;

        /* deadband */
        if (last_pulse_us[i] != 0 && abs((int)pulse - (int)last_pulse_us[i]) < lim.deadband_us) {
            return;
        }

        last_pulse_us[i] = pulse;

        uint32_t duty = us_to_duty(pulse);
        ledc_set_duty(MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(MODE, (ledc_channel_t)i);
    };

    if (ch == ALL) {
        for (int i = 0; i < NUM_SERVO; i++)
            apply(i);
    }
    else {
        apply(ch);
    }
    return ESP_OK;
}

esp_err_t Servo::Initialize()
{
    for (int a = 30; a <= 180; a++) {
        SetAngle(ALL, a);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    SetAngle(ALL, 30);
    return ESP_OK;
}
