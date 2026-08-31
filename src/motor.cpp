#include "motor.h"
#include "Bot_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ    5000
#define LEDC_RES        LEDC_TIMER_8_BIT
#define LEDC_CH_LEFT    LEDC_CHANNEL_0
#define LEDC_CH_RIGHT   LEDC_CHANNEL_1

static void configure_direction_pin(gpio_num_t pin) {
    gpio_config_t conf = {};
    conf.pin_bit_mask = (1ULL << pin);
    conf.mode = GPIO_MODE_OUTPUT;
    conf.pull_up_en = GPIO_PULLUP_DISABLE;
    conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&conf);
    gpio_set_level(pin, 0);
}

void motor_control_init() {
    configure_direction_pin(PIN_MOTOR_IN1);
    configure_direction_pin(PIN_MOTOR_IN2);
    configure_direction_pin(PIN_MOTOR_IN3);
    configure_direction_pin(PIN_MOTOR_IN4);

    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = LEDC_MODE;
    timer_conf.timer_num = LEDC_TIMER;
    timer_conf.duty_resolution = LEDC_RES;
    timer_conf.freq_hz = LEDC_FREQ_HZ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t left_ch = {};
    left_ch.gpio_num = PIN_MOTOR_ENA;
    left_ch.speed_mode = LEDC_MODE;
    left_ch.channel = LEDC_CH_LEFT;
    left_ch.timer_sel = LEDC_TIMER;
    left_ch.duty = 0;
    left_ch.hpoint = 0;
    ledc_channel_config(&left_ch);

    ledc_channel_config_t right_ch = {};
    right_ch.gpio_num = PIN_MOTOR_ENB;
    right_ch.speed_mode = LEDC_MODE;
    right_ch.channel = LEDC_CH_RIGHT;
    right_ch.timer_sel = LEDC_TIMER;
    right_ch.duty = 0;
    right_ch.hpoint = 0;
    ledc_channel_config(&right_ch);
}

static int clamp_speed(int speed) {
    if (speed < 0) return 0;
    if (speed > 255) return 255;
    return speed;
}

void motor_set_left_speed(int speed) {
    ledc_set_duty(LEDC_MODE, LEDC_CH_LEFT, clamp_speed(speed));
    ledc_update_duty(LEDC_MODE, LEDC_CH_LEFT);
}

void motor_set_right_speed(int speed) {
    ledc_set_duty(LEDC_MODE, LEDC_CH_RIGHT, clamp_speed(speed));
    ledc_update_duty(LEDC_MODE, LEDC_CH_RIGHT);
}

void motor_forward(int speed) {
    gpio_set_level(PIN_MOTOR_IN1, 1);
    gpio_set_level(PIN_MOTOR_IN2, 0);
    gpio_set_level(PIN_MOTOR_IN3, 1);
    gpio_set_level(PIN_MOTOR_IN4, 0);
    motor_set_left_speed(speed);
    motor_set_right_speed(speed);
}

void motor_backward(int speed) {
    gpio_set_level(PIN_MOTOR_IN1, 0);
    gpio_set_level(PIN_MOTOR_IN2, 1);
    gpio_set_level(PIN_MOTOR_IN3, 0);
    gpio_set_level(PIN_MOTOR_IN4, 1);
    motor_set_left_speed(speed);
    motor_set_right_speed(speed);
}

// FIX: both sides now drive FORWARD (was: left backward, right forward
// - a pivot spin, not a turn). "speed" is treated as the outer
// (faster) wheel's speed; the inner wheel is slowed proportionally,
// so the robot curves instead of spinning or stopping.
#define TURN_INNER_RATIO   40   // inner wheel runs at this % of outer wheel speed - tune on the real robot

void motor_turn_left(int speed) {

    gpio_set_level(PIN_MOTOR_IN1, 1);   // left = FORWARD (inner wheel)
    gpio_set_level(PIN_MOTOR_IN2, 0);
    gpio_set_level(PIN_MOTOR_IN3, 0);   // right = FORWARD (outer wheel)
    gpio_set_level(PIN_MOTOR_IN4, 0);

    motor_set_left_speed(speed);
    motor_set_right_speed(speed);
}

void motor_turn_right(int speed) {

    gpio_set_level(PIN_MOTOR_IN1, 0);   // left = FORWARD (outer wheel)
    gpio_set_level(PIN_MOTOR_IN2, 0);
    gpio_set_level(PIN_MOTOR_IN3, 1);   // right = FORWARD (inner wheel)
    gpio_set_level(PIN_MOTOR_IN4, 0);

    motor_set_left_speed(speed);
    motor_set_right_speed(speed);
}

void motor_stop() {
    gpio_set_level(PIN_MOTOR_IN1, 0);
    gpio_set_level(PIN_MOTOR_IN2, 0);
    gpio_set_level(PIN_MOTOR_IN3, 0);
    gpio_set_level(PIN_MOTOR_IN4, 0);
    motor_set_left_speed(0);
    motor_set_right_speed(0);
}