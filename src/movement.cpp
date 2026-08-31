#include "movement.h"
#include "Bot_config.h"
#include "motor.h"
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

struct MoveEntry {
    MoveDir dir;
    uint32_t duration_ms;
};

static std::vector<MoveEntry> s_log;
static SemaphoreHandle_t s_log_mutex = NULL;
#define MAX_LOG_ENTRIES 500

void movement_log_init() {
    s_log_mutex = xSemaphoreCreateMutex();
    s_log.reserve(64);
}

void movement_log_record(MoveDir dir, uint32_t duration_ms) {
    if (duration_ms == 0) return;
    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    if (s_log.size() >= MAX_LOG_ENTRIES) s_log.erase(s_log.begin());
    s_log.push_back({dir, duration_ms});
    xSemaphoreGive(s_log_mutex);
}

void movement_log_reset() {
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
    }
    return MoveDir::FORWARD;
}

static void drive(MoveDir d, int speed) {
    switch (d) {
        case MoveDir::FORWARD:  motor_forward(speed); break;
        case MoveDir::BACKWARD: motor_backward(speed); break;
        case MoveDir::LEFT:     motor_turn_left(speed); break;
        case MoveDir::RIGHT:    motor_turn_right(speed); break;
    }
}

void movement_log_retrace() {
    xSemaphoreTake(s_log_mutex, portMAX_DELAY);
    std::vector<MoveEntry> snapshot = s_log;
    s_log.clear();
    xSemaphoreGive(s_log_mutex);

    for (auto it = snapshot.rbegin(); it != snapshot.rend(); ++it) {
        drive(opposite(it->dir), SPEED_FORWARD);
        vTaskDelay(pdMS_TO_TICKS(it->duration_ms));
    }
    motor_stop();
}