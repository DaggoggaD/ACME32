#ifndef SERVOS_H
#define SERVOS_H

#include <stdint.h>
#include "esp_err.h"

// Creates and initializes servo control groups and timers
// Implementation modified from: 
// https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_servo_control/main/mcpwm_servo_control_example_main.c
void init_servos(const int pins[4]);

// Apply fin rotation
esp_err_t set_fin_angle(uint8_t finIndex, float angle_deg);

#endif