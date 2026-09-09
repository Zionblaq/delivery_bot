#include "Bot_state.h"
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "esp_timer.h"

static RobotMode s_mode = RobotMode::MANUAL;
static ManualCmd s_manual_cmd = ManualCmd::NONE;
static int64_t s_manual_cmd_time_us = 0;
static float s_distances[4] = { -1.0f, -1.0f, -1.0f, -1.0f };
static int   s_line_raw[3]  = { 0, 0, 0 };
static std::string s_current_pin = "";
static bool s_order_active = false;
static bool s_unlock_active = false;
static bool s_powered_off = false;
static DeliveryLocation s_target_location = DeliveryLocation::NONE;
static bool s_priority_override = false;
static SemaphoreHandle_t s_mutex = NULL;

void robot_state_init() {
    s_mutex = xSemaphoreCreateMutex();
}

void robot_state_set_mode(RobotMode m) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_mode = m;
    xSemaphoreGive(s_mutex);
}

RobotMode robot_state_get_mode() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    RobotMode m = s_mode;
    xSemaphoreGive(s_mutex);
    return m;
}

void robot_state_set_manual_cmd(ManualCmd c) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_manual_cmd = c;
    s_manual_cmd_time_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
}

ManualCmd robot_state_get_manual_cmd() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    ManualCmd c = s_manual_cmd;
    xSemaphoreGive(s_mutex);
    return c;
}

bool robot_state_manual_cmd_is_stale(uint32_t max_age_ms) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int64_t age_us = esp_timer_get_time() - s_manual_cmd_time_us;
    xSemaphoreGive(s_mutex);
    return (age_us / 1000) > max_age_ms;
}

void robot_state_set_distance(int index, float cm) {
    if (index < 0 || index > 3) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_distances[index] = cm;
    xSemaphoreGive(s_mutex);
}

void robot_state_get_distances(float out[4]) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, s_distances, sizeof(s_distances));
    xSemaphoreGive(s_mutex);
}

void robot_state_set_line_raw(int index, int raw) {
    if (index < 0 || index > 2) return;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_line_raw[index] = raw;
    xSemaphoreGive(s_mutex);
}

void robot_state_get_line_raw(int out[3]) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, s_line_raw, sizeof(s_line_raw));
    xSemaphoreGive(s_mutex);
}

std::string robot_state_new_order() {
    char buf[8];
    int pin_val = rand() % 10000;
    snprintf(buf, sizeof(buf), "%04d", pin_val);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_current_pin = buf;
    s_order_active = true;
    xSemaphoreGive(s_mutex);

    return s_current_pin;
}

bool robot_state_check_pin(const std::string &pin) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool match = s_order_active && (pin == s_current_pin);
    if (match) s_order_active = false;
    xSemaphoreGive(s_mutex);
    return match;
}

void robot_state_set_unlock_active(bool active) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_unlock_active = active;
    xSemaphoreGive(s_mutex);
}

bool robot_state_get_unlock_active() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool a = s_unlock_active;
    xSemaphoreGive(s_mutex);
    return a;
}

void robot_state_set_powered_off(bool off) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_powered_off = off;
    xSemaphoreGive(s_mutex);
}

bool robot_state_is_powered_off() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool o = s_powered_off;
    xSemaphoreGive(s_mutex);
    return o;
}

void robot_state_set_target_location(DeliveryLocation loc) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_target_location = loc;
    xSemaphoreGive(s_mutex);
}

DeliveryLocation robot_state_get_target_location() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    DeliveryLocation l = s_target_location;
    xSemaphoreGive(s_mutex);
    return l;
}

void robot_state_request_priority_override() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_priority_override = true;
    xSemaphoreGive(s_mutex);
}

bool robot_state_consume_priority_override() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool v = s_priority_override;
    s_priority_override = false;
    xSemaphoreGive(s_mutex);
    return v;
}