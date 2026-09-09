#include "servo.h"
#include "Bot_config.h"
#include "Bot_state.h"
#include "movement.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SERVO_LEDC_TIMER    LEDC_TIMER_1
#define SERVO_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL  LEDC_CHANNEL_2
#define SERVO_FREQ_HZ       50
#define SERVO_RES           LEDC_TIMER_14_BIT

static uint32_t angle_to_duty(float angle_deg) {
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > SERVO_MAX_ANGLE) angle_deg = SERVO_MAX_ANGLE;

    float pulse_us = SERVO_MIN_PULSE_US +
        (angle_deg / (float)SERVO_MAX_ANGLE) * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);

    return (uint32_t)((pulse_us / 20000.0f) * 16383.0f);
}

void servo_control_init() {
    ledc_timer_config_t timer_conf = {};
    timer_conf.speed_mode = SERVO_LEDC_MODE;
    timer_conf.timer_num = SERVO_LEDC_TIMER;
    timer_conf.duty_resolution = SERVO_RES;
    timer_conf.freq_hz = SERVO_FREQ_HZ;
    timer_conf.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {};
    ch_conf.gpio_num = PIN_SERVO;
    ch_conf.speed_mode = SERVO_LEDC_MODE;
    ch_conf.channel = SERVO_LEDC_CHANNEL;
    ch_conf.timer_sel = SERVO_LEDC_TIMER;
    ch_conf.duty = angle_to_duty(SERVO_CLOSED_ANGLE);
    ch_conf.hpoint = 0;
    ledc_channel_config(&ch_conf);
}

void servo_open() {
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, angle_to_duty(SERVO_OPEN_ANGLE));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

void servo_close() {
    ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, angle_to_duty(SERVO_CLOSED_ANGLE));
    ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL);
}

void servo_unlock_task(void *pvParameters) {
    (void)pvParameters;

    robot_state_set_unlock_active(true);
    servo_open();

    vTaskDelay(pdMS_TO_TICKS(UNLOCK_HOLD_MS));

    servo_close();
    movement_log_retrace();

    robot_state_set_unlock_active(false);
    robot_state_set_mode(RobotMode::MANUAL);

    vTaskDelete(NULL);
}