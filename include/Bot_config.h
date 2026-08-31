#pragma once
#include "driver/gpio.h"

/* ===================== MOTOR PINS (L298N) ===================== */
#define PIN_MOTOR_ENA   GPIO_NUM_14
#define PIN_MOTOR_ENB   GPIO_NUM_9
#define PIN_MOTOR_IN1   GPIO_NUM_13
#define PIN_MOTOR_IN2   GPIO_NUM_12
#define PIN_MOTOR_IN3   GPIO_NUM_11
#define PIN_MOTOR_IN4   GPIO_NUM_10

/* ===================== ULTRASONIC SENSORS ===================== */
#define US1_TRIG  GPIO_NUM_19
#define US1_ECHO  GPIO_NUM_20
#define US2_TRIG  GPIO_NUM_21
#define US2_ECHO  GPIO_NUM_47
#define US3_TRIG  GPIO_NUM_48
#define US3_ECHO  GPIO_NUM_36
#define US4_TRIG  GPIO_NUM_37
#define US4_ECHO  GPIO_NUM_35

#define OBSTACLE_DISTANCE_CM      25.0f
#define ECHO_TIMEOUT_US           50000
#define OBSTACLE_DEBOUNCE_COUNT   3
#define STARTUP_SETTLE_MS         2000

#define US_FRONT_LEFT     0
#define US_FRONT_CENTER   1
#define US_FRONT_RIGHT    2
#define US_REAR           3

/* ===================== IR LINE SENSORS (analog) =====================
 * Each sensor is treated as a simple detected/not-detected boolean.
 * The PATTERN of which sensors see black identifies location markers:
 *   L+C+R           -> Location A
 *   C+R (no L)      -> Location B
 *   L+C (no R)      -> Location C
 * A single sensor alone (just L, just C, or just R) is normal
 * steering, not a location marker.
 */

#define IR_LEFT_PIN    GPIO_NUM_7
#define IR_CENTER_PIN  GPIO_NUM_5
#define IR_RIGHT_PIN   GPIO_NUM_6

#define LOCATION_DEBOUNCE_COUNT   3

#define IR_ACTIVE_LOW  true

// PLACEHOLDER - calibrate against real readings from debug_task.
// A raw value above this = "line detected" for that sensor.
#define LINE_DETECT_THRESHOLD   2000

#define LINE_LOST_GRACE_MS      500
#define LINE_SEARCH_TIMEOUT_MS  3000

/* ===================== SERVO (delivery lock) ===================== */
#define PIN_SERVO           GPIO_NUM_8
#define SERVO_MIN_PULSE_US  500
#define SERVO_MAX_PULSE_US  2500
#define SERVO_MAX_ANGLE     270
#define SERVO_CLOSED_ANGLE  270
#define SERVO_OPEN_ANGLE    90
#define UNLOCK_HOLD_MS      (2 * 60 * 1000)

/* ===================== MOTOR SPEEDS ===================== */
#define SPEED_FORWARD     300
#define SPEED_TURN        500
#define SPEED_STOP        0

/* ===================== WIFI SOFT AP ===================== */
#define WIFI_AP_SSID      "DeliveryBot"
#define WIFI_AP_PASS      "bot12345"
#define WIFI_AP_CHANNEL   1
#define WIFI_AP_MAX_CONN  4