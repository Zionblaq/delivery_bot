#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "Bot_config.h"
#include "motor.h"
#include "IR.h"

static const char *TAG = "LINEFOLLOW";

static bool pattern_has_any(LinePattern p) {
    return p.left || p.center || p.right;
}

extern "C" void app_main(void) {
    motor_control_init();
    line_sensor_init();

    ESP_LOGI(TAG, "Starting...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // Sensor stabilization delay

    while (1) {
        LinePattern p = read_line_pattern();
        bool line_seen = pattern_has_any(p);

        if (line_seen) {
            /* At least one sensor sees the black line */
            motor_forward(SPEED_FORWARD);
        } else {
            /* All sensors see white (line lost) -> STOP */
            motor_stop();
        }

        ESP_LOGI(TAG, "IR[L=%d C=%d R=%d] status: %s",
                 p.left, p.center, p.right, line_seen ? "FORWARD" : "STOPPED");

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}