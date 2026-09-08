#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define TRIG_PIN GPIO_NUM_19
#define ECHO_PIN GPIO_NUM_20
#define TIMEOUT_US 30000 // 30ms timeout (~5m max)

static const char *TAG = "ULTRASONIC";

void init_ultrasonic(void) {
    // Configure Trigger Pin (Output)
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);

    // Configure Echo Pin (Input)
    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&echo_conf);

    gpio_set_level(TRIG_PIN, 0);
}

float measure_distance_cm(void) {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - wait_start) > TIMEOUT_US) return -1.0f;
    }
    int64_t echo_start = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - echo_start) > TIMEOUT_US) return -1.0f;
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t pulse_duration = echo_end - echo_start;

    // Calibrated divisor for Wokwi / standard air temperature (340 m/s sound speed)
    return (float)pulse_duration / 58.75f;
}

void app_main(void) {
    init_ultrasonic();
    
    // Print initial boot message directly
    printf("\n=== ESP32-S3 Ultrasonic Test Started ===\n");
    fflush(stdout);

    while (1) {
        float distance = measure_distance_cm();

        if (distance < 0) {
            printf("[WARNING] Distance: Out of range or timeout\n");
        } else {
            printf("[DATA] Distance: %.2f cm\n", distance);
        }
        
        fflush(stdout); // Force text output to Wokwi Serial Monitor
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}