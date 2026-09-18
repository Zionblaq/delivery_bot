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
#define IR_LEFT_PIN    GPIO_NUM_7
#define IR_CENTER_PIN  GPIO_NUM_5
#define IR_RIGHT_PIN   GPIO_NUM_6

#define IR_ACTIVE_LOW  false 

/* ===================== MOTOR SPEEDS (0-255 PWM range) ===================== */
#define SPEED_FORWARD   350
#define SPEED_TURN      350
#define SPEED_STOP      0