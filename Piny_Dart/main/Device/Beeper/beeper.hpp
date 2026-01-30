#ifndef __BEEPER_HPP__
#define __BEEPER_HPP__

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

class Beeper
{
public:
    explicit Beeper(gpio_num_t pin);
    ~Beeper();

    void play(float freq_hz, uint32_t duration_ms);
    void play_boot_music();

private:
    static constexpr const char *TAG = "BEEPER";

    rmt_channel_handle_t tx_channel = nullptr;
    rmt_encoder_handle_t encoder = nullptr;
};

#endif
