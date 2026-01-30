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
    NOTE_SILENT = 0,
    NOTE_C4,
    NOTE_D4,
    NOTE_E4,
    NOTE_F4,
    NOTE_G4,
    NOTE_A4,
    NOTE_B4,
    NOTE_C5,
    NOTE_D5,
    NOTE_E5,
    NOTE_F5,
    NOTE_G5,
    NOTE_A5,
    NOTE_B5,
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
