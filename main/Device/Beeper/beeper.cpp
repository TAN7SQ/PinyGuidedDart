#include "beeper.hpp"
#include "esp_err.h"
#include <cmath>

/* ===== 音符频率表 ===== */
static float note_freq_table[] = {
    0.00f,    // 0: NO_NOTE（静音）
    27.50f,   // 1: NOTE_A0
    29.14f,   // 2: NOTE_AS0
    30.87f,   // 3: NOTE_B0
    32.70f,   // 4: NOTE_C1
    34.65f,   // 5: NOTE_CS1
    36.71f,   // 6: NOTE_D1
    38.89f,   // 7: NOTE_DS1
    41.20f,   // 8: NOTE_E1
    43.65f,   // 9: NOTE_F1
    46.25f,   // 10: NOTE_FS1
    49.00f,   // 11: NOTE_G1
    51.91f,   // 12: NOTE_GS1
    55.00f,   // 13: NOTE_A1
    58.27f,   // 14: NOTE_AS1
    61.74f,   // 15: NOTE_B1
    65.41f,   // 16: NOTE_C2
    69.30f,   // 17: NOTE_CS2
    73.42f,   // 18: NOTE_D2
    77.78f,   // 19: NOTE_DS2
    82.41f,   // 20: NOTE_E2
    87.31f,   // 21: NOTE_F2
    92.50f,   // 22: NOTE_FS2
    98.00f,   // 23: NOTE_G2
    103.83f,  // 24: NOTE_GS2
    110.00f,  // 25: NOTE_A2
    116.54f,  // 26: NOTE_AS2
    123.47f,  // 27: NOTE_B2
    130.81f,  // 28: NOTE_C3
    138.59f,  // 29: NOTE_CS3
    146.83f,  // 30: NOTE_D3
    155.56f,  // 31: NOTE_DS3
    164.81f,  // 32: NOTE_E3
    174.61f,  // 33: NOTE_F3
    185.00f,  // 34: NOTE_FS3
    196.00f,  // 35: NOTE_G3
    207.65f,  // 36: NOTE_GS3
    220.00f,  // 37: NOTE_A3
    233.08f,  // 38: NOTE_AS3
    246.94f,  // 39: NOTE_B3
    261.63f,  // 40: NOTE_C4（中央C）
    277.18f,  // 41: NOTE_CS4
    293.66f,  // 42: NOTE_D4
    311.13f,  // 43: NOTE_DS4
    329.63f,  // 44: NOTE_E4
    349.23f,  // 45: NOTE_F4
    369.99f,  // 46: NOTE_FS4
    392.00f,  // 47: NOTE_G4
    415.30f,  // 48: NOTE_GS4
    440.00f,  // 49: NOTE_A4（基准音440Hz）
    466.16f,  // 50: NOTE_AS4
    493.88f,  // 51: NOTE_B4
    523.25f,  // 52: NOTE_C5
    554.37f,  // 53: NOTE_CS5
    587.33f,  // 54: NOTE_D5
    622.25f,  // 55: NOTE_DS5
    659.25f,  // 56: NOTE_E5
    698.46f,  // 57: NOTE_F5
    739.99f,  // 58: NOTE_FS5
    783.99f,  // 59: NOTE_G5
    830.61f,  // 60: NOTE_GS5
    880.00f,  // 61: NOTE_A5
    932.33f,  // 62: NOTE_AS5
    987.77f,  // 63: NOTE_B5
    1046.50f, // 64: NOTE_C6
    1108.73f, // 65: NOTE_CS6
    1174.66f, // 66: NOTE_D6
    1244.51f, // 67: NOTE_DS6
    1318.51f, // 68: NOTE_E6
    1396.91f, // 69: NOTE_F6
    1479.98f, // 70: NOTE_FS6
    1567.98f, // 71: NOTE_G6
    1661.22f, // 72: NOTE_GS6
    1760.00f, // 73: NOTE_A6
    1864.66f, // 74: NOTE_AS6
    1975.53f, // 75: NOTE_B6
    2093.00f, // 76: NOTE_C7
    2217.46f, // 77: NOTE_CS7
    2349.32f, // 78: NOTE_D7
    2489.02f, // 79: NOTE_DS7
    2637.02f, // 80: NOTE_E7
    2793.83f, // 81: NOTE_F7
    2959.96f, // 82: NOTE_FS7
    3135.96f, // 83: NOTE_G7
    3322.44f, // 84: NOTE_GS7
    3520.00f, // 85: NOTE_A7
    3729.31f, // 86: NOTE_AS7
    3951.07f, // 87: NOTE_B7
    4186.01f  // 88: NOTE_C8
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
    xTaskCreatePinnedToCore(beeper_task, "beeper_task", 2048, this, 5, &task_handle, 1);

    ESP_LOGI(TAG, "RMT beeper initialized");
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
    play(NOTE_E5, 250);
    play(NOTE_C5, 150);
    play(NOTE_G5, 350);
}

void Beeper::play_run_music()
{
    play(NOTE_C5, 250);
    play(NOTE_E5, 150);
    play(NOTE_G5, 150);
    play(NOTE_C6, 150);
}
