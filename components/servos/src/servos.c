#include "servos.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

#define NUM_SERVO 4

// Min and max rotation
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2500

static mcpwm_cmpr_handle_t comparators[NUM_SERVO];
static const char *TAG = "SERVOS";

static inline uint32_t degrees_to_us(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;
    return (uint32_t)(SERVO_MIN_PULSEWIDTH_US + (angle * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / 180.0f));
}

void init_servos(const int pins[NUM_SERVO]) {
    // Function created using ESP-IDF V5 API reference:
    // https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_servo_control/main/mcpwm_servo_control_example_main.c
    // scaled to control 4 fins.
    // Each group can control up to 3 servos, wich means 2 groups
    // are needed for 4 fin controls. Each group has it's own timer,
    // and each servo has it's operator, comparator and generator
    // to control the current flow.


    ESP_LOGI(TAG, "Fins initialization");

    mcpwm_timer_handle_t timers[2] = {NULL, NULL};
    
    // Groups timer cration (pitch and yaw)
    for (int g = 0; g < 2; g++) {
        mcpwm_timer_config_t timer_config = {
            .group_id = g,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = 1000000, 
            .period_ticks = 20000,    
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        };

        // TODO: Use of ESP_ERROR_CHECK should be also used in main file.
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timers[g]));
    }

    for (int i = 0; i < NUM_SERVO; i++) {
        int group = (i < 2) ? 0 : 1; 

        mcpwm_oper_handle_t oper = NULL;
        mcpwm_operator_config_t operator_config = { .group_id = group };
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timers[group]));

        // Comparator and generator to control HIGH and LOW voltage
        mcpwm_comparator_config_t comparator_config = { .flags.update_cmp_on_tez = true };
        ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparators[i]));

        mcpwm_gen_handle_t generator = NULL;
        
        mcpwm_generator_config_t generator_config = { .gen_gpio_num = pins[i] };
        ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparators[i], MCPWM_GEN_ACTION_LOW)));

        mcpwm_comparator_set_compare_value(comparators[i], degrees_to_us(90.0f));
    }

    for (int g = 0; g < 2; g++) {
        ESP_ERROR_CHECK(mcpwm_timer_enable(timers[g]));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(timers[g], MCPWM_TIMER_START_NO_STOP));
    }
}

esp_err_t set_fin_angle(uint8_t fin_index, float angle_degrees) {
    if (fin_index < NUM_SERVO) {
        mcpwm_comparator_set_compare_value(comparators[fin_index], degrees_to_us(angle_degrees));
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}