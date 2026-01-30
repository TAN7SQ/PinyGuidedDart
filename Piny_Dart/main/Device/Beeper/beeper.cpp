#include "beeper.hpp"
#include "esp_err.h"
#include <cmath>

/* ===== 音符频率表 ===== */
static float note_freq_table[] = {
    0.0f,
    261.63f,
    293.66f,
    329.63f,
    349.23f,
    392.00f,
    440.00f,
    493.88f,
    523.25f,
    587.33f,
    659.25f,
    698.46f,
    783.99f,
    880.00f,
    987.77f,
};

Beeper::Beeper(gpio_num_t pin)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags =
            {
                .invert_out = false,
                .with_dma = false,
            },
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &tx_channel));

    rmt_copy_encoder_config_t encoder_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_cfg, &encoder));

    ESP_ERROR_CHECK(rmt_enable(tx_channel));

    beep_queue = xQueueCreate(8, sizeof(BeepCmd));
    xTaskCreatePinnedToCore(beeper_task, "beeper_task", 2048, this, 5, &task_handle, 0);

    ESP_LOGI(TAG, "Non-blocking RMT beeper initialized");
}

Beeper::~Beeper()
{
    if (task_handle)
        vTaskDelete(task_handle);
    if (beep_queue)
        vQueueDelete(beep_queue);
    if (tx_channel) {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
    }
    if (encoder)
        rmt_del_encoder(encoder);
}

void Beeper::play(BeepNote note, uint32_t duration_ms)
{
    if (note >= sizeof(note_freq_table) / sizeof(note_freq_table[0]))
        return;
    play(note_freq_table[note], duration_ms);
}

void Beeper::play(float freq_hz, uint32_t duration_ms)
{
    BeepCmd cmd = {
        .freq = freq_hz,
        .duration_ms = duration_ms,
    };
    xQueueSend(beep_queue, &cmd, 0);
}

void Beeper::beeper_task(void *arg)
{
    auto *self = static_cast<Beeper *>(arg);
    BeepCmd cmd;

    while (true) {
        if (xQueueReceive(self->beep_queue, &cmd, portMAX_DELAY)) {
            self->play_blocking(cmd.freq, cmd.duration_ms);
        }
    }
}

void Beeper::play_blocking(float freq_hz, uint32_t duration_ms)
{
    if (freq_hz <= 0 || duration_ms == 0) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        return;
    }

    uint16_t period_us = static_cast<uint32_t>(1000000.0f / freq_hz);
    uint16_t half = period_us / 2;

    rmt_symbol_word_t symbol = {
        .duration0 = half,
        .level0 = 1,
        .duration1 = half,
        .level1 = 0,
    };

    int loop = static_cast<int>(freq_hz * duration_ms / 1000.0f);

    rmt_transmit_config_t tx_cfg = {
        .loop_count = loop,
        .flags = {.eot_level = 0},
    };

    rmt_transmit(tx_channel, encoder, &symbol, sizeof(symbol), &tx_cfg);
    rmt_tx_wait_all_done(tx_channel, portMAX_DELAY);
}

void Beeper::play_boot_music()
{
    play(NOTE_C5, 350);
    play(NOTE_E5, 250);
    play(NOTE_G5, 500);
}
