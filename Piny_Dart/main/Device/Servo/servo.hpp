#ifndef __SERVO_HPP__
#define __SERVO_HPP__

#include <iostream>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include <cstdint>

/*

servo_channel_t servo_handle = {
    .servo_pin = {
        GPIO_NUM_18,
        GPIO_NUM_19,
        GPIO_NUM_21,
        GPIO_NUM_22,
        GPIO_NUM_23,
        GPIO_NUM_25,
        GPIO_NUM_26,
        GPIO_NUM_27,
    },
    .ch = {
        LEDC_CHANNEL_0,
        LEDC_CHANNEL_1,
        LEDC_CHANNEL_2,
        LEDC_CHANNEL_3,
        LEDC_CHANNEL_4,
        LEDC_CHANNEL_5,
        LEDC_CHANNEL_6,
        LEDC_CHANNEL_7,
    },
};

*/

class Servo
{
public:
    enum ServoCH_e
    {
        LF0 = 0,
        LF1,
        RF0,
        RF1,
        ALL
    };

    Servo();
    esp_err_t Initialize();
    esp_err_t SetAngle(ServoCH_e ch, int angle);

private:
    static constexpr const char *TAG = "SERVO";

    static constexpr ledc_timer_t TIMER = LEDC_TIMER_2;
    static constexpr ledc_mode_t MODE = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_bit_t RES = LEDC_TIMER_10_BIT;

    static constexpr uint32_t SERVO_FREQ = 50;
    static constexpr uint32_t MIN_US = 500;
    static constexpr uint32_t MAX_US = 2500;

    static constexpr gpio_num_t PINS[4] = {
        GPIO_NUM_13,
        GPIO_NUM_14,
        GPIO_NUM_15,
        GPIO_NUM_16,
    };

    uint32_t us_to_duty(uint32_t us);
};

#endif
