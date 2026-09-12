#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Bot_config.h"
#include "motor.h"
#include "IR.h"

static const char *TAG = "SMOOTH_STEER";

/* Differential turn speeds */
#define SPEED_OUTER_WHEEL   SPEED_FORWARD        /* Full forward speed */
#define SPEED_INNER_WHEEL  (SPEED_FORWARD / 4)   /* Reduced speed for gentle arc */

extern "C" void app_main(void) {
    motor_control_init();
    line_sensor_init();

    ESP_LOGI(TAG, "System starting with differential steering...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Sensor stabilization delay

    while (1) {
        LinePattern p = read_line_pattern();

        if (p.left && !p.right) {
            /* Drifted Right -> Gentle Arc Left */
            ESP_LOGI(TAG, "Arcing Left");
            motor_set_left_speed(SPEED_INNER_WHEEL);
            motor_set_right_speed(SPEED_OUTER_WHEEL);
        } 
        else if (p.right && !p.left) {
            /* Drifted Left -> Gentle Arc Right */
            ESP_LOGI(TAG, "Arcing Right");
            motor_set_left_speed(SPEED_OUTER_WHEEL);
            motor_set_right_speed(SPEED_INNER_WHEEL);
        } 
        else if (p.center || (p.left && p.right)) {
            /* Center or full line -> Drive Straight */
            ESP_LOGI(TAG, "Moving Straight");
            motor_forward(SPEED_FORWARD);
        } 
        else {
            /* All sensors see white -> Stop */
            ESP_LOGI(TAG, "All White -> Stopped");
            motor_stop();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}