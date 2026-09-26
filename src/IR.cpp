#include "IR.h"
#include "Bot_config.h"
#include "driver/gpio.h"

void LineSensor(){
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << LeftPin) | (1ULL << CenterPin) | (1ULL << RightPin);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);

}

LinePattern read_line_pattern() {
    LinePattern p;
    p.left   = (gpio_get_level(LeftPin) == LineLevel);
    p.center = (gpio_get_level(LeftPin) == LineLevel);
    p.right  = (gpio_get_level(LeftPin) == LineLevel);
    return p;
}