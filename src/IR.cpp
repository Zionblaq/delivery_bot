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

<<<<<<< HEAD
bool ir_line_detected(void)
{
    return ir_read_left() || ir_read_mid() || ir_read_right();
=======
void line_sensor_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
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
>>>>>>> 99be0c6 (Hopefully Works)
}