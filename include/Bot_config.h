#pragma once
#include "driver/gpio.h"

/* ===================== MOTOR PINS (L298N) ===================== */
#define PIN_MOTOR_ENA   GPIO_NUM_14
#define PIN_MOTOR_ENB   GPIO_NUM_9
#define PIN_MOTOR_IN1   GPIO_NUM_13
#define PIN_MOTOR_IN2   GPIO_NUM_12
#define PIN_MOTOR_IN3   GPIO_NUM_11
#define PIN_MOTOR_IN4   GPIO_NUM_10

#define LF_TRIG  GPIO_NUM_35
#define LF_ECHO  GPIO_NUM_36
#define MID_TRIG  GPIO_NUM_1
#define MID_ECHO  GPIO_NUM_2
#define RG_TRIG  GPIO_NUM_16
#define RG_ECHO  GPIO_NUM_15
#define BK_TRIG  GPIO_NUM_37
#define BK_ECHO  GPIO_NUM_38

#define OBSTACLE_DISTANCE_CM      25.0f
#define ECHO_TIMEOUT_US           50000
#define OBSTACLE_DEBOUNCE_COUNT   3
#define STARTUP_SETTLE_MS         2000

#define US_FRONT_LEFT     0
#define US_FRONT_CENTER   1
#define US_FRONT_RIGHT    2
#define US_REAR           3

#define IR_LEFT_PIN    GPIO_NUM_7
#define IR_CENTER_PIN  GPIO_NUM_5
#define IR_RIGHT_PIN   GPIO_NUM_6

#define LOCATION_DEBOUNCE_COUNT   3

#define IR_ACTIVE_LOW  true


#define LINE_DETECT_THRESHOLD   2000

#define LINE_LOST_GRACE_MS      500
#define LINE_SEARCH_TIMEOUT_MS  3000


#define PIN_SERVO           GPIO_NUM_8
#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_MAX_ANGLE     270
#define SERVO_CLOSED_ANGLE  270
#define SERVO_OPEN_ANGLE    90
#define UNLOCK_HOLD_MS      (2 * 30 * 1000)

#define SPEED_FORWARD     300
#define SPEED_TURN        500
#define SPEED_STOP        0
#define SPEED_SLOW        250 

#define WIFI_AP_SSID      "DeliveryBot"
#define WIFI_AP_PASS      "bot12345"
#define WIFI_AP_CHANNEL   1
#define WIFI_AP_MAX_CONN  4
