#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bot_config.h"
#include "IR.h"
#include "movement.h"

static const char *TAG = "LINE_CAR";

enum class CarState {
    None,
    Forward,
    Left,
    Right,
    Stop
};

// ESP-IDF looks for a C symbol named app_main, so it needs C linkage in a .cpp file
extern "C" void app_main(void)
{
    lineSensor();
    movement();

    CarState last_state = CarState::None;

    while (true) {
        int right_ir = ir_read_right();
        int left_ir  = ir_read_left();
        int mid_ir   = ir_read_mid();

        CarState state;

        if (right_ir == 0 && left_ir == 0 && mid_ir == 1) {
            state = CarState::Forward;
        } else if (right_ir == 1) {
            state = CarState::Right;
        } else if (left_ir == 1) {
            state = CarState::Left;
        } else {
            state = CarState::Stop;
        }

        // Act and log only when the state changes
        if (state != last_state) {
            switch (state) {
                case CarState::Forward: move_forward(); ESP_LOGI(TAG, "Waiting for line"); break;
                case CarState::Right:   turn_right();   ESP_LOGI(TAG, "Turning Right");    break;
                case CarState::Left:    turn_left();    ESP_LOGI(TAG, "Turning Left");     break;
                default:                stop_car();     ESP_LOGI(TAG, "Stopping");         break;
            }
            last_state = state;
        }

        vTaskDelay(1);  // yield to the idle task and watchdog
    }
}