#include "IR.h"
#include "Bot_config.h"
#include "Bot_state.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static gpio_num_t s_pins[3] = { IR_LEFT_PIN, IR_CENTER_PIN, IR_RIGHT_PIN };

void line_sensor_init() {
    for (int i = 0; i < 3; i++) {
        gpio_config_t conf = {};
        conf.pin_bit_mask = (1ULL << s_pins[i]);
        conf.mode = GPIO_MODE_INPUT;
        conf.pull_up_en = GPIO_PULLUP_DISABLE;
        conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&conf);
    }
}

static bool read_detected(gpio_num_t pin) {
    int level = gpio_get_level(pin);
    return IR_ACTIVE_LOW ? (level == 0) : (level == 1);
}

void line_sensor_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        // Store as 0/1 in shared state so debug_task's existing %d
        // printf format still works without changes.
        robot_state_set_line_raw(0, read_detected(s_pins[0]) ? 1 : 0);
        robot_state_set_line_raw(1, read_detected(s_pins[1]) ? 1 : 0);
        robot_state_set_line_raw(2, read_detected(s_pins[2]) ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

LinePattern read_line_pattern() {
    LinePattern p;
    p.left   = read_detected(s_pins[0]);
    p.center = read_detected(s_pins[1]);
    p.right  = read_detected(s_pins[2]);
    return p;
}