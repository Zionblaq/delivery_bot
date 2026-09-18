#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Bot_config.h"
#include "motor.h"
#include "IR.h"

static const char *TAG = "LINE_FOLLOWER";

/* --- SPEED CONFIGURATION --- */
#define SPEED_MAX   SPEED_FORWARD   /* Max forward speed */
#define SPEED_SLOW  SPEED_TURN             /* Custom speed (250 PWM) for turns */

typedef enum {
    LAST_DIR_NONE,
    LAST_DIR_LEFT,
    LAST_DIR_RIGHT
} LastDirection;

extern "C" void app_main(void) {
    motor_control_init();
    line_sensor_init();

    ESP_LOGI(TAG, "Starting line follower with SPEED_SLOW = 250...");
    vTaskDelay(pdMS_TO_TICKS(2000));

    LastDirection last_dir = LAST_DIR_NONE;

    while (1) {
        LinePattern p = read_line_pattern();

        // 1. CENTER ON LINE -> Drive Straight at full speed
        if (p.center) {
            ESP_LOGI(TAG, "Moving Straight");
            motor_forward(SPEED_MAX);
            last_dir = LAST_DIR_NONE;
        }
        // 2. BEND LEFT -> Turn Left using SPEED_SLOW (250)
        else if (p.left) {
            ESP_LOGI(TAG, "Turning Left (250)");
            motor_turn_left(SPEED_SLOW);
            last_dir = LAST_DIR_LEFT;
        }
        // 3. BEND RIGHT -> Turn Right using SPEED_SLOW (250)
        else if (p.right) {
            ESP_LOGI(TAG, "Turning Right (250)");
            motor_turn_right(SPEED_SLOW);
            last_dir = LAST_DIR_RIGHT;
        }
        // 4. LINE LOST (All white) -> Search at SPEED_SLOW (250)
        else {
            if (last_dir == LAST_DIR_LEFT) {
                ESP_LOGI(TAG, "Searching Left (250)");
                motor_turn_left(SPEED_SLOW);
            } 
            else if (last_dir == LAST_DIR_RIGHT) {
                ESP_LOGI(TAG, "Searching Right (250)");
                motor_turn_right(SPEED_SLOW);
            } 
            else {
                motor_stop();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}