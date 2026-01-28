#ifndef __BEEPER_HPP__
#define __BEEPER_HPP__
#include "buzzer.h"
class Beeper
{
public:
    Beeper(gpio_num_t pin)
    {
        buzzer_init(pin);
        playmusic();
        ESP_LOGI("Beeper", "Beeper initialized");
    }
    void play(piano_note_t note, uint32_t duration)
    {
        buzzer(note, duration, 1, 1, 5);
    }
    void playmusic(void)
    {
        piano_note_t dji_boot_notes[] = {
            NOTE_C5, //
            NOTE_E5, //
            NOTE_G5  //
        };

        float note_loud_time[] = {
            0.35, 
            0.22, 
            0.45  
        };

        uint8_t melody_len = sizeof(dji_boot_notes) / sizeof(dji_boot_notes[0]);

        for (uint8_t i = 0; i < melody_len; i++) {
            buzzer(dji_boot_notes[i], 7000, note_loud_time[i], 0.02, 1);
        }

        ledc_set_duty(BUZZER_TIMER_SPEED_MODE, BUZZER_CHANNEL_NUM, 0);
        ledc_update_duty(BUZZER_TIMER_SPEED_MODE, BUZZER_CHANNEL_NUM);
    }
};

#endif