#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Bot_config.h"
#include "motor.h"
#include "IR.h"

static const char *TAG = "LINE_FOLLOWER";

typedef enum {
    LAST_DIR_NONE,
    LAST_DIR_LEFT,
    LAST_DIR_RIGHT
} LastDirection;

extern "C" void app_main(void) {
    motor_control_init();
    LineSensor();

    ESP_LOGI(TAG, "Starting motors...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Stabilization delay

    // Ensure motor direction pins are set to FORWARD at boot
    motor_forward(SPEED_FORWARD);

    LastDirection last_dir = LAST_DIR_NONE;

    while (1) {
        LinePattern p = read_line_pattern();

        // 1. CENTER ON LINE (0 1 0 or 1 1 1) -> Drive straight full speed
        if (p.center || (p.left && p.right)) {
            ESP_LOGI(TAG, "Straight");
            motor_forward(SPEED_FORWARD); // Reinforces forward direction
            motor_set_left_speed(SPEED_FORWARD);
            motor_set_right_speed(SPEED_FORWARD);
            last_dir = LAST_DIR_NONE;
        }
        // 2. DRIFTED RIGHT (Left sees line) -> Arc Left
        else if (p.left && !p.right) {
            ESP_LOGI(TAG, "Turning Left");
            motor_forward(SPEED_FORWARD);
            motor_set_left_speed(SPEED_SLOW);      // Slow left wheel
            motor_set_right_speed(SPEED_FORWARD);  // Fast right wheel
            last_dir = LAST_DIR_LEFT;
        }
        // 3. DRIFTED LEFT (Right sees line) -> Arc Right
        else if (p.right && !p.left) {
            ESP_LOGI(TAG, "Turning Right");
            motor_forward(SPEED_FORWARD);
            motor_set_left_speed(SPEED_FORWARD);   // Fast left wheel
            motor_set_right_speed(SPEED_SLOW);     // Slow right wheel
            last_dir = LAST_DIR_RIGHT;
        }
        // 4. ALL WHITE (Line lost) -> Search in last known direction
        else {
            if (last_dir == LAST_DIR_LEFT) {
                ESP_LOGI(TAG, "Searching Left...");
                motor_forward(SPEED_FORWARD);
                motor_set_left_speed(0);
                motor_set_right_speed(SPEED_SLOW);
            }
            else if (last_dir == LAST_DIR_RIGHT) {
                ESP_LOGI(TAG, "Searching Right...");
                motor_forward(SPEED_FORWARD);
                motor_set_left_speed(SPEED_SLOW);
                motor_set_right_speed(0);
            }
            else {
                // Initial boot on white ground: creep forward to find line
                ESP_LOGI(TAG, "Creeping forward to find line...");
                motor_forward(SPEED_SLOW);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(loop_delay_MS));
    }
}