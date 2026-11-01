#include "ultrasonic.h"
#include "bot_config.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

void ultrasonic_init(void)
{
    gpio_config_t trig_conf = {};
    trig_conf.pin_bit_mask = (1ULL << MID_TRIG) | (1ULL << LF_TRIG) | (1ULL << RG_TRIG);
    trig_conf.mode         = GPIO_MODE_OUTPUT;
    trig_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    trig_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    trig_conf.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&trig_conf));

    gpio_config_t echo_conf = {};
    echo_conf.pin_bit_mask = (1ULL << MID_ECHO) | (1ULL << LF_ECHO) | (1ULL << RG_ECHO);
    echo_conf.mode         = GPIO_MODE_INPUT;
    echo_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    echo_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    echo_conf.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&echo_conf));

    gpio_set_level(MID_TRIG, 0);
    gpio_set_level(LF_TRIG, 0);
    gpio_set_level(RG_TRIG, 0);
}

int ultrasonic_measure_cm(gpio_num_t trig, gpio_num_t echo)
{
    int64_t low_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if (esp_timer_get_time() - low_start >= ULTRASONIC_TIMEOUT_US) {
            return 999;
        }
    }

    gpio_set_level(trig, 0);
    esp_rom_delay_us(3);
    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig, 0);

    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 0) {
        if (esp_timer_get_time() - wait_start >= ULTRASONIC_TIMEOUT_US) {
            return 999;
        }
    }

    int64_t pulse_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if (esp_timer_get_time() - pulse_start >= ULTRASONIC_TIMEOUT_US) {
            return 999;
        }
    }

    int64_t duration = esp_timer_get_time() - pulse_start;
    if (duration == 0) {
        return 999;
    }
    
    return (int)(duration * 0.0343 / 2);
}

void ultrasonic_read_all(distances_t *out)
{
    out->front = ultrasonic_measure_cm(MID_TRIG, MID_ECHO);
    DELAY_MS(ULTRASONIC_GAP_MS);

    out->left = ultrasonic_measure_cm(LF_TRIG, LF_ECHO);
    DELAY_MS(ULTRASONIC_GAP_MS);

    out->right = ultrasonic_measure_cm(RG_TRIG, RG_ECHO);
    DELAY_MS(ULTRASONIC_GAP_MS);
}