#ifndef BUZZER_H
#define BUZZER_H

#include <driver/ledc.h>
#include "esp_log.h"

#define BUZZER_LEDC_TIMER       LEDC_TIMER_0
#define BUZZER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_CHANNEL     LEDC_CHANNEL_0

// Configures timers and channel
void init_buzzer(int buzzPin);

// Set buzzers frequency, volume on a scale of 0 to 100
void buzzer_set_tone(uint32_t freq, uint8_t volume);

#endif 