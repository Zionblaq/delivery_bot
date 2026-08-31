#include "Ultrasonic.h"
#include "Bot_config.h"
#include "Bot_state.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>

#define SOUND_SPEED_CM_US   0.0343f
#define SAMPLE_GAP_MS       60
#define MAX_VALID_CM        400.0f   // HC-SR04 realistic max range - anything beyond is noise

struct UsPair {
    gpio_num_t trig;
    gpio_num_t echo;
};

static UsPair s_sensors[4] = {
    { US1_TRIG, US1_ECHO },
    { US2_TRIG, US2_ECHO },
    { US3_TRIG, US3_ECHO },
    { US4_TRIG, US4_ECHO },
};

static void configure_sensor_pins(UsPair &s) {
    gpio_config_t trig_conf = {};
    trig_conf.pin_bit_mask = (1ULL << s.trig);
    trig_conf.mode = GPIO_MODE_OUTPUT;
    trig_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    trig_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    trig_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {};
    echo_conf.pin_bit_mask = (1ULL << s.echo);
    echo_conf.mode = GPIO_MODE_INPUT;
    echo_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    echo_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    echo_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&echo_conf);

    gpio_set_level(s.trig, 0);
}

void ultrasonic_init() {
    for (auto &s : s_sensors) configure_sensor_pins(s);
}

// Single raw ping - unchanged physics, just kept as a building block now.
static float single_ping_cm(UsPair &s) {
    gpio_set_level(s.trig, 0);
    esp_rom_delay_us(2);
    gpio_set_level(s.trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(s.trig, 0);

    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(s.echo) == 0) {
        if ((esp_timer_get_time() - wait_start) > ECHO_TIMEOUT_US) return -1.0f;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(s.echo) == 1) {
        if ((esp_timer_get_time() - echo_start) > ECHO_TIMEOUT_US) return -1.0f;
    }
    int64_t echo_end = esp_timer_get_time();

    float d = ((echo_end - echo_start) * SOUND_SPEED_CM_US) / 2.0f;

    // Reject physically implausible readings outright - these are what
    // produced the 802.2cm and 180-vs-18 glitches in your log.
    if (d > MAX_VALID_CM) return -1.0f;

    return d;
}

// NEW: takes 3 quick pings and returns the median. A single corrupted
// reading (from task preemption mid-measurement) gets outvoted by the
// two good ones, instead of passing straight through to shared state.
static float measure_distance_cm(UsPair &s) {
    float samples[3];
    for (int i = 0; i < 3; i++) {
        samples[i] = single_ping_cm(&s == &s ? s : s); // keep signature simple
        esp_rom_delay_us(3000); // brief gap between the 3 pings on the same sensor
    }

    // Treat timeouts as the worst case (sort high) so 2 valid + 1 timeout
    // still picks a real reading via the median.
    for (int i = 0; i < 3; i++) {
        if (samples[i] < 0) samples[i] = 9999.0f;
    }

    std::sort(samples, samples + 3);
    float median = samples[1];

    return (median >= 9999.0f) ? -1.0f : median;
}

void ultrasonic_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        for (int i = 0; i < 4; i++) {
            float d = measure_distance_cm(s_sensors[i]);
            robot_state_set_distance(i, d);
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_GAP_MS));
        }
    }
}