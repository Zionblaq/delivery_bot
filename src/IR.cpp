#include "IR.h"
#include "Bot_config.h"
#include "driver/gpio.h"

void lineSensor(){
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << rightSensor ) | (1ULL << midSensor) | (1ULL << leftSensor);
    config.mode = GPIO_MODE_INPUT;
    config.pull_up_en = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&config);

}

int ir_read_right(void) { 
    return gpio_get_level(rightSensor);
 }
int ir_read_left(void)  { 
    return gpio_get_level(leftSensor); 
 }
int ir_read_mid(void)   { 
    return gpio_get_level(midSensor);          
}