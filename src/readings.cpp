#include "readings.h"
#include "Bot_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SENSORS";

void debug_task(void *pvParameters) {
    (void)pvParameters;
    float distances[4];
    int line_raw[3];

    while (1) {
        robot_state_get_distances(distances);
        robot_state_get_line_raw(line_raw);

        ESP_LOGI(TAG, "US[FL=%.1f FC=%.1f FR=%.1f R=%.1f]cm  IR[L=%d C=%d R=%d]",
                 distances[0], distances[1], distances[2], distances[3],
                 line_raw[0], line_raw[1], line_raw[2]);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}