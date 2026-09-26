#pragma once
#include "driver/gpio.h"

/* ===================== MOTOR PINS (L298N) ===================== */
#define PIN_MOTOR_ENA   GPIO_NUM_14
#define PIN_MOTOR_ENB   GPIO_NUM_9
#define PIN_MOTOR_IN1   GPIO_NUM_13
#define PIN_MOTOR_IN2   GPIO_NUM_12
#define PIN_MOTOR_IN3   GPIO_NUM_11
#define PIN_MOTOR_IN4   GPIO_NUM_10

/* ===================== IR LINE SENSORS (digital) ===================== */
#define LeftPin    GPIO_NUM_7
#define CenterPin  GPIO_NUM_5
#define RightPin  GPIO_NUM_6
#define LineLevel 0 

#define SPEED_FORWARD   750
#define SPEED_TURN      750
#define SPEED_SLOW      350
#define SPEED_STOP      0
#define loop_delay_MS   20