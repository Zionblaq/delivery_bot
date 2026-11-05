#include "movement.h"
#include "Bot_config.h"
#include "esp_err.h"

static void set_pwm(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

<<<<<<< HEAD
static void set_direction(int r1, int r2, int l1, int l2)
{
    gpio_set_level(RIGHT_MOTOR_PIN1, r1);
    gpio_set_level(RIGHT_MOTOR_PIN2, r2);
    gpio_set_level(LEFT_MOTOR_PIN1,  l1);
    gpio_set_level(LEFT_MOTOR_PIN2,  l2);
}

static void pwm_init(void)
{
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode      = PWM_MODE;
    timer_conf.duty_resolution = PWM_RESOLUTION;
    timer_conf.timer_num       = PWM_TIMER;
    timer_conf.freq_hz         = PWM_FREQ_HZ;
    timer_conf.clk_cfg         = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t right_ch = {};
    right_ch.gpio_num   = ENABLE_RIGHT_MOTOR;
    right_ch.speed_mode = PWM_MODE;
    right_ch.channel    = RIGHT_PWM_CHANNEL;
    right_ch.timer_sel  = PWM_TIMER;
    right_ch.duty       = 0;
    right_ch.hpoint     = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&right_ch));

    ledc_channel_config_t left_ch = {};
    left_ch.gpio_num   = ENABLE_LEFT_MOTOR;
    left_ch.speed_mode = PWM_MODE;
    left_ch.channel    = LEFT_PWM_CHANNEL;
    left_ch.timer_sel  = PWM_TIMER;
    left_ch.duty       = 0;
    left_ch.hpoint     = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&left_ch));
}

static void gpio_init_motors(void)
{
    gpio_config_t out_conf = {};
    out_conf.pin_bit_mask = (1ULL << RIGHT_MOTOR_PIN1) | (1ULL << RIGHT_MOTOR_PIN2) |
                            (1ULL << LEFT_MOTOR_PIN1)  | (1ULL << LEFT_MOTOR_PIN2);
    out_conf.mode         = GPIO_MODE_OUTPUT;
    out_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    out_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_conf.intr_type    = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&out_conf));
}


void movement_init(void)
{
    gpio_init_motors();
    pwm_init();
    stop_car();
}

=======
void movement_log_record(MoveDir dir, uint32_t duration_ms) {
    if (duration_ms == 0) return;
    if (s_log_mutex == NULL) movement_log_init();
    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    if (s_log.size() >= MAX_LOG_ENTRIES) s_log.erase(s_log.begin());
    s_log.push_back({dir, duration_ms});
    xSemaphoreGive(s_log_mutex);
}

void movement_log_reset() {
    if (s_log_mutex == NULL) {
        movement_log_init();
    }
    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    s_log.clear();
    xSemaphoreGive(s_log_mutex);
}

static MoveDir opposite(MoveDir d) {
    switch (d) {
        case MoveDir::FORWARD:  return MoveDir::BACKWARD;
        case MoveDir::BACKWARD: return MoveDir::FORWARD;
        case MoveDir::LEFT:     return MoveDir::RIGHT;
        case MoveDir::RIGHT:    return MoveDir::LEFT;
        case MoveDir::PIVOT_LEFT:  return MoveDir::PIVOT_RIGHT;
        case MoveDir::PIVOT_RIGHT: return MoveDir::PIVOT_LEFT;
    }
    return MoveDir::FORWARD;
}

static void drive(MoveDir d, int speed) {
    switch (d) {
        case MoveDir::FORWARD:  motor_forward(speed); break;
        case MoveDir::BACKWARD: motor_backward(speed); break;
        case MoveDir::LEFT:     motor_turn_left(speed); break;
        case MoveDir::RIGHT:    motor_turn_right(speed); break;
        case MoveDir::PIVOT_LEFT:  motor_turn_left(speed); break;
        case MoveDir::PIVOT_RIGHT: motor_turn_right(speed); break;
    }
}

void movement_log_retrace() {
    if (s_log_mutex == NULL) {
        movement_log_init();
    }
    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    std::vector<MoveEntry> snapshot = s_log;
    s_log.clear();
    xSemaphoreGive(s_log_mutex);
>>>>>>> 99be0c6 (Hopefully Works)

static uint32_t trimmed(int speed, int pct)
{
    int v = speed * pct / 100;
    if (v > 255) v = 255;
    if (v < 0)   v = 0;
    return (uint32_t)v;
}

void move_forward_speed(int speed)
{
    set_direction(1, 0, 1, 0);
    set_pwm(RIGHT_PWM_CHANNEL, speed);
    set_pwm(LEFT_PWM_CHANNEL,  speed);
}

void move_forward(void)
{
    move_forward_speed(MOTOR_SPEED);
}

void move_backward(void)
{
    set_direction(0, 1, 0, 1);
    set_pwm(RIGHT_PWM_CHANNEL, MOTOR_SPEED);
    set_pwm(LEFT_PWM_CHANNEL,  MOTOR_SPEED);
}

void turn_left(void)
{
    // Right motor forward, left motor backward
    set_direction(1, 0, 0, 1);
    set_pwm(RIGHT_PWM_CHANNEL, MOTOR_SPEED);
    set_pwm(LEFT_PWM_CHANNEL,  TURN_SPEED);
}

void turn_right(void)
{
    // Right motor backward, left motor forward
    set_direction(0, 1, 1, 0);
    set_pwm(RIGHT_PWM_CHANNEL, TURN_SPEED);
    set_pwm(LEFT_PWM_CHANNEL,  MOTOR_SPEED);
}

void stop_car(void)
{
    // Same as the original: only the enable (PWM) pins are zeroed
    set_pwm(RIGHT_PWM_CHANNEL, 0);
    set_pwm(LEFT_PWM_CHANNEL,  0);
}