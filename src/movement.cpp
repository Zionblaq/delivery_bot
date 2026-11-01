#include "movement.h"
#include "Bot_config.h"
#include "esp_err.h"

static void set_pwm(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

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