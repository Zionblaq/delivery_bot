#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Bot_config.h"
#include "motor.h"
#include "IR.h"

static const char *TAG = "LINEFOLLOW";

extern "C" void app_main(void) {
    motor_control_init();
    line_sensor_init();

    ESP_LOGI(TAG, "Line follower starting...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // let sensors settle before moving

    while (1) {
        LinePattern p = read_line_pattern();

        if (p.center) {
            motor_forward(SPEED_FORWARD);
        } else if (p.left) {
            motor_turn_left(SPEED_TURN);
        } else if (p.right) {
            motor_turn_right(SPEED_TURN);
        } else {
            // No sensor sees the line - stop rather than guess.
            motor_stop();
        }

        ESP_LOGI(TAG, "IR[L=%d C=%d R=%d]", p.left, p.center, p.right);

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}