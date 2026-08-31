#include "Task.h"
#include "Bot_config.h"
#include "Bot_state.h"
#include "motor.h"
#include "IR.h"
#include "movement.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "CONTROL";

static MoveDir s_current_dir = MoveDir::FORWARD;
static bool s_moving = false;
static int64_t s_segment_start_us = 0;
static int s_obstacle_hit_count = 0;

// Brief debounce so a single-frame sensor glitch doesn't stop the robot
// prematurely - real "reached destination" no-line will hold steady.
static int64_t s_line_lost_since_us = 0;
static bool s_line_was_lost = false;
#define LINE_LOST_STOP_DEBOUNCE_MS 200

static void set_active_direction(MoveDir dir, int speed) {
    int64_t now = esp_timer_get_time();
    if (s_moving && dir == s_current_dir) return;

    if (s_moving) {
        uint32_t elapsed_ms = (uint32_t)((now - s_segment_start_us) / 1000);
        movement_log_record(s_current_dir, elapsed_ms);
    }

    switch (dir) {
        case MoveDir::FORWARD:  motor_forward(speed); break;
        case MoveDir::BACKWARD: motor_backward(speed); break;
        case MoveDir::LEFT:     motor_turn_left(speed); break;
        case MoveDir::RIGHT:    motor_turn_right(speed); break;
    }

    s_current_dir = dir;
    s_segment_start_us = now;
    s_moving = true;
}

static void stop_and_flush() {
    if (s_moving) {
        int64_t now = esp_timer_get_time();
        uint32_t elapsed_ms = (uint32_t)((now - s_segment_start_us) / 1000);
        movement_log_record(s_current_dir, elapsed_ms);
        s_moving = false;
    }
    motor_stop();
}

static bool is_front_blocked(float d[4]) {
    return (d[US_FRONT_CENTER] > 0 && d[US_FRONT_CENTER] < OBSTACLE_DISTANCE_CM) ||
           (d[US_FRONT_LEFT]   > 0 && d[US_FRONT_LEFT]   < OBSTACLE_DISTANCE_CM) ||
           (d[US_FRONT_RIGHT]  > 0 && d[US_FRONT_RIGHT]  < OBSTACLE_DISTANCE_CM);
}

static bool pattern_has_any(LinePattern p) {
    return p.left || p.center || p.right;
}

static bool should_abort(RobotMode mode_at_start) {
    return robot_state_get_mode() != mode_at_start || robot_state_is_powered_off();
}

static bool execute_timed_step(MoveDir dir, int speed, uint32_t duration_ms, RobotMode mode_at_start) {
    stop_and_flush();
    int64_t start = esp_timer_get_time();
    set_active_direction(dir, speed);

    while ((esp_timer_get_time() - start) / 1000 < duration_ms) {
        if (should_abort(mode_at_start)) {
            ESP_LOGW(TAG, "Maneuver aborted - mode/power changed via web UI");
            stop_and_flush();
            return false;
        }

        float distances[4];
        robot_state_get_distances(distances);
        if (dir == MoveDir::FORWARD && is_front_blocked(distances)) {
            ESP_LOGW(TAG, "Secondary obstacle detected mid-maneuver!");
            stop_and_flush();
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
    return true;
}

// Obstacle detected: stop, detour around it, then find the line again
// and resume normal following.
static void navigate_around_obstacle(float distances[4]) {
    ESP_LOGW(TAG, "Starting autonomous obstacle avoidance sequence");
    RobotMode mode_at_start = robot_state_get_mode();

    stop_and_flush();
    vTaskDelay(pdMS_TO_TICKS(100));

    float fl = (distances[US_FRONT_LEFT] > 0)  ? distances[US_FRONT_LEFT]  : 999.0f;
    float fr = (distances[US_FRONT_RIGHT] > 0) ? distances[US_FRONT_RIGHT] : 999.0f;

    MoveDir detour_side = (fl >= fr) ? MoveDir::LEFT : MoveDir::RIGHT;
    MoveDir return_side = (detour_side == MoveDir::LEFT) ? MoveDir::RIGHT : MoveDir::LEFT;

    if (!execute_timed_step(detour_side, SPEED_TURN, 450, mode_at_start)) return;
    if (!execute_timed_step(MoveDir::FORWARD, SPEED_FORWARD, 750, mode_at_start)) return;
    if (!execute_timed_step(return_side, SPEED_TURN, 450, mode_at_start)) return;

    ESP_LOGI(TAG, "Bypass complete. Searching for black line...");
    set_active_direction(MoveDir::FORWARD, SPEED_FORWARD);

    int64_t search_start = esp_timer_get_time();
    bool line_reacquired = false;

    while ((esp_timer_get_time() - search_start) / 1000 < 2500) {
        if (should_abort(mode_at_start)) {
            ESP_LOGW(TAG, "Line search canceled - mode/power changed via web UI");
            break;
        }
        if (pattern_has_any(read_line_pattern())) {
            line_reacquired = true;
            ESP_LOGI(TAG, "Line re-acquired!");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(15));
    }

    stop_and_flush();
    s_line_was_lost = false; // reset so follow_line() doesn't immediately think it's arrived

    if (!line_reacquired) {
        // No obstacle right now (we're past it) and no line found -
        // per your spec, that means arrived. Just stop here; don't
        // force a mode switch, so the web UI stays in control.
        ESP_LOGI(TAG, "No line found after bypass - treating as arrived");
    }
}

// Normal line-following. No line AND no obstacle (obstacle already
// ruled out by the caller) means the destination has been reached.
static void follow_line() {
    LinePattern p = read_line_pattern();
    bool line_seen = pattern_has_any(p);

    if (line_seen) {
        s_line_was_lost = false;

        if (p.center) {
            set_active_direction(MoveDir::FORWARD, SPEED_FORWARD);
        } else if (p.left) {
            set_active_direction(MoveDir::LEFT, SPEED_TURN);
        } else if (p.right) {
            set_active_direction(MoveDir::RIGHT, SPEED_TURN);
        } else {
            // Should not happen given line_seen is true, but keep moving
            // straight as a safe default.
            set_active_direction(MoveDir::FORWARD, SPEED_FORWARD);
        }
        return;
    }

    // No line detected right now.
    if (!s_line_was_lost) {
        s_line_was_lost = true;
        s_line_lost_since_us = esp_timer_get_time();
        // Keep moving straight during the debounce window rather than
        // stopping abruptly on a single glitched frame.
        set_active_direction(MoveDir::FORWARD, SPEED_FORWARD);
        return;
    }

    uint32_t lost_ms = (uint32_t)((esp_timer_get_time() - s_line_lost_since_us) / 1000);

    if (lost_ms < LINE_LOST_STOP_DEBOUNCE_MS) {
        set_active_direction(MoveDir::FORWARD, SPEED_FORWARD);
    } else {
        // No line, confirmed for LINE_LOST_STOP_DEBOUNCE_MS, and no
        // obstacle blocking (control_task already checked that before
        // calling this) - per your spec, this means arrived.
        stop_and_flush();
        ESP_LOGI(TAG, "No line, no obstacle - reached destination, stopping");
    }
}

#define MANUAL_CMD_TIMEOUT_MS 400

static void run_manual() {
    if (robot_state_manual_cmd_is_stale(MANUAL_CMD_TIMEOUT_MS)) {
        stop_and_flush();
        return;
    }

    ManualCmd cmd = robot_state_get_manual_cmd();
    switch (cmd) {
        case ManualCmd::FORWARD:  set_active_direction(MoveDir::FORWARD, SPEED_FORWARD); break;
        case ManualCmd::BACKWARD: set_active_direction(MoveDir::BACKWARD, SPEED_FORWARD); break;
        case ManualCmd::LEFT:     set_active_direction(MoveDir::LEFT, SPEED_TURN); break;
        case ManualCmd::RIGHT:    set_active_direction(MoveDir::RIGHT, SPEED_TURN); break;
        case ManualCmd::STOP:
        case ManualCmd::NONE:
        default:
            stop_and_flush();
            break;
    }
}

void control_task_init() {
    movement_log_init();
}

void control_task(void *pvParameters) {
    (void)pvParameters;
    float distances[4];

    ESP_LOGI(TAG, "Settling sensors for %dms before control starts...", STARTUP_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_SETTLE_MS));
    ESP_LOGI(TAG, "Control loop starting");

    while (1) {
        if (robot_state_get_unlock_active() || robot_state_is_powered_off()) {
            stop_and_flush();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        RobotMode mode = robot_state_get_mode();

        if (mode == RobotMode::MANUAL) {
            run_manual();
        } else {
            robot_state_get_distances(distances);

            if (is_front_blocked(distances)) s_obstacle_hit_count++;
            else s_obstacle_hit_count = 0;

            if (s_obstacle_hit_count >= OBSTACLE_DEBOUNCE_COUNT) {
                navigate_around_obstacle(distances);
                s_obstacle_hit_count = 0;
                vTaskDelay(pdMS_TO_TICKS(100));
            } else {
                follow_line();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }
}