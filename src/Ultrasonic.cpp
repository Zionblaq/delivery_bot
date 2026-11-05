#include "ultrasonic.h"
#include "bot_config.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

<<<<<<< HEAD
void ultrasonic_init(void)
{
=======
#define SOUND_SPEED_CM_US   0.0343f
#define SAMPLE_GAP_MS       30      // pause after each sensor (echoes beyond 5 m are gone by then)
#define PING_TIMEOUT_US     9000    // give up on a ping after ~150 cm; the navigator does not need more
#define MAX_VALID_CM        400.0f   // HC-SR04 realistic max range - anything beyond is noise

struct UsPair {
    gpio_num_t trig;
    gpio_num_t echo;
};

// Order must match the US_* indices in Bot_config.h (these are what robot_state_set_distance() uses)
static UsPair s_sensors[4] = {
    { LF_TRIG,  LF_ECHO  },   // US_FRONT_LEFT   = 0
    { MID_TRIG, MID_ECHO },   // US_FRONT_CENTER = 1
    { RG_TRIG,  RG_ECHO  },   // US_FRONT_RIGHT  = 2
    { BK_TRIG,  BK_ECHO  },   // US_REAR         = 3
};

static void configure_sensor_pins(UsPair &s) {
>>>>>>> 99be0c6 (Hopefully Works)
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

<<<<<<< HEAD
int ultrasonic_measure_cm(gpio_num_t trig, gpio_num_t echo)
{
    int64_t low_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if (esp_timer_get_time() - low_start >= ULTRASONIC_TIMEOUT_US) {
            return 999;
=======
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
        if ((esp_timer_get_time() - wait_start) > PING_TIMEOUT_US) return -1.0f;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(s.echo) == 1) {
        if ((esp_timer_get_time() - echo_start) > PING_TIMEOUT_US) return -1.0f;
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
        samples[i] = single_ping_cm(s);
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

// Measure one sensor, publish the result, then pause so its echo cannot disturb the next sensor
static void update_sensor(int i) {
    float d = measure_distance_cm(s_sensors[i]);
    robot_state_set_distance(i, d);
    vTaskDelay(pdMS_TO_TICKS(SAMPLE_GAP_MS));
}

// The front-centre sensor is re-read after every other sensor, so the obstacle check and the
// line-following navigator see fresh front readings several times per cycle.
void ultrasonic_task(void *pvParameters) {
    (void)pvParameters;
    while (1) {
        for (int i = 0; i < 4; i++) {
            update_sensor(i);
            if (i != US_FRONT_CENTER) update_sensor(US_FRONT_CENTER);
>>>>>>> 99be0c6 (Hopefully Works)
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