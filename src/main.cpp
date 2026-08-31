#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "Bot_state.h"
#include "motor.h"
#include "Ultrasonic.h"
#include "IR.h"
#include "servo.h"
#include "Task.h"
#include "web_server.h"
#include "readings.h"

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }

    robot_state_init();
    control_task_init();

    motor_control_init();
    ultrasonic_init();
    line_sensor_init();
    servo_control_init();

    wifi_ap_start();
    web_server_start();

    xTaskCreate(ultrasonic_task,  "ultrasonic_task",  4096, NULL, 4, NULL);
    xTaskCreate(line_sensor_task, "line_sensor_task", 2048, NULL, 4, NULL);
    xTaskCreate(control_task,     "control_task",     4096, NULL, 5, NULL);
    xTaskCreate(debug_task,       "debug_task",        4096, NULL, 3, NULL);
}