#ifndef __SERVO_HPP__
#define __SERVO_HPP__

#include <iostream>

#include "iot_servo.h"
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

        LB0,
        LB1,
        RB0,
        RB1,

        ALL = 8,
    };

    static constexpr const char *TAG = "PWM";
    Servo();
    ~Servo();

    esp_err_t Initialize(); // 舵机运动到初始位置
    esp_err_t SetAngle(ServoCH_e Servo, int angle);
    esp_err_t ReadAngle(ServoCH_e Servo, float &angle);

private:
    ledc_timer_t timerCh;
    servo_channel_t channels;
    uint8_t MaxChNum;
};

#endif
