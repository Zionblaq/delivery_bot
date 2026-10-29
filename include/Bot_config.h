#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"


#define rightSensor     GPIO_NUM_34
#define leftSensor      GPIO_NUM_35
#define midSensor       GPIO_NUM_32

#define MOTOR_SPEED      200
#define slow_speed        150


#define ENABLE_RIGHT_MOTOR  GPIO_NUM_33
#define RIGHT_MOTOR_PIN1    GPIO_NUM_26
#define RIGHT_MOTOR_PIN2    GPIO_NUM_27
#define ENABLE_LEFT_MOTOR   GPIO_NUM_13
#define LEFT_MOTOR_PIN1     GPIO_NUM_14
#define LEFT_MOTOR_PIN2     GPIO_NUM_12

#define PWM_FREQ_HZ         1000
#define PWM_MODE            LEDC_LOW_SPEED_MODE
#define PWM_TIMER           LEDC_TIMER_0
#define PWM_RESOLUTION      LEDC_TIMER_8_BIT
#define RIGHT_PWM_CHANNEL   LEDC_CHANNEL_0
#define LEFT_PWM_CHANNEL    LEDC_CHANNEL_1
