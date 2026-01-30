#include "beeper.hpp"
#include "esp_err.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

Beeper::Beeper(gpio_num_t pin)
{
    rmt_tx_channel_config_t tx_cfg = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1 MHz → 1 tick = 1 us
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

    ESP_LOGI(TAG, "RMT beeper initialized");
}

Beeper::~Beeper()
{
    if (tx_channel) {
        rmt_disable(tx_channel);
        rmt_del_channel(tx_channel);
    }
    if (encoder) {
        rmt_del_encoder(encoder);
    }
}

void Beeper::play(float freq_hz, uint32_t duration_ms)
{
    if (freq_hz <= 0 || duration_ms == 0)
        return;

    uint16_t period_us = static_cast<uint32_t>(1'000'000.0f / freq_hz);
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
        .flags =
            {
                .eot_level = 0,
            },
    };

    ESP_ERROR_CHECK(rmt_transmit(tx_channel, encoder, &symbol, sizeof(symbol), &tx_cfg));

    ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channel, portMAX_DELAY));
}

void Beeper::play_boot_music()
{
    play(523.25f, 320); // C5
    play(659.25f, 220); // E5
    play(783.99f, 450); // G5
}