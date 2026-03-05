#ifndef WS2812B_HPP
#define WS2812B_HPP

#include "led_strip.h"

#define WS2812B_GPIO_PIN 39 // 数据线连接的 GPIO 引脚

#define WS2812B_LED_NUM 2 // LED 数量

#define WS2812b_RMT_RES_HZ (10 * 1000 * 1000)

#define REFRESH_INTERVAL_STEP 10

extern "C" {

led_strip_handle_t configure_led(void);

void ws2812b_RGBOn(led_strip_handle_t led_strip, uint32_t index, uint8_t r, uint8_t g, uint8_t b);

void ws2812b_HSVOn(led_strip_handle_t led_strip, uint32_t index, uint8_t h, uint8_t s, uint8_t v);

void ws2812b_Off(led_strip_handle_t led_strip, uint32_t index);

void ws2812b_breath_HSV(led_strip_handle_t led_strip, int h, uint8_t s, int v, const int step);

void ws2812b_rainbow(led_strip_handle_t led_strip);
}

#endif