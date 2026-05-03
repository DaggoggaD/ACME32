#include "buzzer.h"

void init_buzzer(int buzzPin) {

    ledc_timer_config_t timer_conf = {
        .speed_mode       = BUZZER_LEDC_MODE,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .timer_num        = BUZZER_LEDC_TIMER,
        .freq_hz          = 2000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t channel_conf = {
        .gpio_num       = buzzPin,
        .speed_mode     = BUZZER_LEDC_MODE,
        .channel        = BUZZER_LEDC_CHANNEL,
        .timer_sel      = BUZZER_LEDC_TIMER,
        .duty           = 0, // Minimum volume at intitialization
        .hpoint         = 0
    };
    ledc_channel_config(&channel_conf);

    ESP_LOGI("Buzzer", "Buzzer correctly intialized");
}

void buzzer_set_tone(uint32_t freq, uint8_t volume) {

    uint16_t duty = (uint16_t)((4095 * volume) / 100);
    if (freq == 0) {
        ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
        ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    } else {
        ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq);
        ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, duty);
        ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    }
}