#ifndef __BEEPER_HPP__
#define __BEEPER_HPP__

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <cstdint>

/* ===== 音符表 ===== */
enum BeepNote
{
    NO_NOTE = 0,
    NOTE_A0,
    NOTE_AS0,
    NOTE_B0,
    NOTE_C1,
    NOTE_CS1,
    NOTE_D1,
    NOTE_DS1,
    NOTE_E1,
    NOTE_F1,
    NOTE_FS1,
    NOTE_G1,
    NOTE_GS1,
    NOTE_A1,
    NOTE_AS1,
    NOTE_B1,
    NOTE_C2,
    NOTE_CS2,
    NOTE_D2,
    NOTE_DS2,
    NOTE_E2,
    NOTE_F2,
    NOTE_FS2,
    NOTE_G2,
    NOTE_GS2,
    NOTE_A2,
    NOTE_AS2,
    NOTE_B2,
    NOTE_C3,
    NOTE_CS3,
    NOTE_D3,
    NOTE_DS3,
    NOTE_E3,
    NOTE_F3,
    NOTE_FS3,
    NOTE_G3,
    NOTE_GS3,
    NOTE_A3,
    NOTE_AS3,
    NOTE_B3,
    NOTE_C4,
    NOTE_CS4,
    NOTE_D4,
    NOTE_DS4,
    NOTE_E4,
    NOTE_F4,
    NOTE_FS4,
    NOTE_G4,
    NOTE_GS4,
    NOTE_A4,
    NOTE_AS4,
    NOTE_B4,
    NOTE_C5,
    NOTE_CS5,
    NOTE_D5,
    NOTE_DS5,
    NOTE_E5,
    NOTE_F5,
    NOTE_FS5,
    NOTE_G5,
    NOTE_GS5,
    NOTE_A5,
    NOTE_AS5,
    NOTE_B5,
    NOTE_C6,
    NOTE_CS6,
    NOTE_D6,
    NOTE_DS6,
    NOTE_E6,
    NOTE_F6,
    NOTE_FS6,
    NOTE_G6,
    NOTE_GS6,
    NOTE_A6,
    NOTE_AS6,
    NOTE_B6,
    NOTE_C7,
    NOTE_CS7,
    NOTE_D7,
    NOTE_DS7,
    NOTE_E7,
    NOTE_F7,
    NOTE_FS7,
    NOTE_G7,
    NOTE_GS7,
    NOTE_A7,
    NOTE_AS7,
    NOTE_B7,
    NOTE_C8,
    // 可选：添加数组长度宏，方便遍历/校验
    BEEP_NOTE_TOTAL
};

struct BeepCmd
{
    float freq;
    uint32_t duration_ms;
};

class Beeper
{
public:
    explicit Beeper(gpio_num_t pin);
    ~Beeper();

    /* 非阻塞接口 */
    void play(BeepNote note, uint32_t duration_ms);
    void play(float freq_hz, uint32_t duration_ms);
    void play_boot_music();
    void play_run_music();
    void play_wifi_music();

private:
    static constexpr const char *TAG = "BEEPER";

    rmt_channel_handle_t tx_channel = nullptr;
    rmt_encoder_handle_t encoder = nullptr;

    QueueHandle_t beep_queue = nullptr;
    TaskHandle_t task_handle = nullptr;

    static void beeper_task(void *arg);
    void play_blocking(float freq_hz, uint32_t duration_ms);
};

#endif
