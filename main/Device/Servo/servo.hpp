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

struct ServoLimit
{
    uint16_t min_us;
    uint16_t max_us;
    uint16_t deadband_us;
};

class Servo
{
public:
    enum ServoCH_e
    {
        CH1 = 0,
        // LF1,
        CH2,
        // RF1,

        CH3,
        CH4,
        CH5,
        CH6,

        FAN,

        ALL
    };

    Servo();
    esp_err_t Initialize();
    esp_err_t SetAngle(ServoCH_e ch, uint8_t angle);
    esp_err_t SetAngles(uint16_t angles[ALL]);
    esp_err_t SetDelta(ServoCH_e ch, float delta);

private:
    static constexpr const char *TAG = "SERVO";

    static constexpr ledc_timer_t TIMER = LEDC_TIMER_2;
    static constexpr ledc_mode_t MODE = LEDC_LOW_SPEED_MODE;
    static constexpr ledc_timer_bit_t RES = LEDC_TIMER_10_BIT;

    static constexpr uint32_t SERVO_FREQ = 50;
    static constexpr uint32_t MIN_US = 500;
    static constexpr uint32_t MAX_US = 2500;

    static constexpr gpio_num_t PINS[] = {
        GPIO_NUM_13,
        GPIO_NUM_14,
        GPIO_NUM_3,
        GPIO_NUM_16,
        GPIO_NUM_15,
        GPIO_NUM_17,
        GPIO_NUM_18,
        // GPIO_NUM_41,
    };

    static constexpr size_t NUM_SERVO = sizeof(PINS) / sizeof(PINS[0]);

    static constexpr ServoLimit limits[] = {
        {600, 2400, 10}, // LF0
        {600, 2400, 10}, // LF1
        {550, 2450, 10}, // RF0
        {550, 2450, 10}, // RF1
        {500, 2500, 10}, // LB0
        {500, 2500, 10}, // LB1
        {500, 2500, 10}, // RB0
        // {500, 2500, 15}, // RB1  // FIXME：发现好像如果使用8个，RMT会出问题
    };
    uint32_t last_pulse_us[NUM_SERVO] = {0};

    uint32_t us_to_duty(uint32_t us);
};

#endif
