
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

// =====================================================
// NAVIGATION SETTINGS
// =====================================================

namespace {

constexpr uint32_t NAV_PERIOD_MS = 10;

// Junction
constexpr uint32_t JUNCTION_CONFIRM_MS = 30;
constexpr uint32_t JUNCTION_ROLL_MS = 300;
constexpr uint32_t JUNCTION_TURN_MIN_MS = 200;
constexpr uint32_t JUNCTION_TURN_TIMEOUT_MS = 1500;

// Startup
constexpr uint32_t START_IGNORE_MS = 500;

// Destination verification
constexpr uint32_t LINE_LOST_CONFIRM_MS = 200;
constexpr uint32_t DEST_CREEP_MS = 800;
constexpr uint32_t DEST_CONFIRM_MS = 300;

// Obstacle
constexpr float OBSTACLE_STOP_CM = 25.0f;
constexpr uint32_t OBSTACLE_CONFIRM_MS = 120;
constexpr uint32_t RESUME_GRACE_MS = 700;

// Motor tuning
constexpr int FOLLOW_SPEED = SPEED_FORWARD;
constexpr int GENTLE_TURN_SPEED = SPEED_SLOW;
constexpr int SHARP_TURN_SPEED = SPEED_TURN;

enum class NavState {
    FOLLOW_LINE,
    JUNCTION_ROLL,
    JUNCTION_TURN,
    DEST_CREEP,
    DEST_CONFIRM,
    ARRIVED
};

} // namespace


// =====================================================
// SHARED NAVIGATION STATE
// =====================================================

static NavState s_nav_state = NavState::FOLLOW_LINE;

static bool s_branch_taken = false;
static bool s_turn_left = false;
static bool s_center_left_old_line = false;
static bool s_arrived = false;

static int64_t s_start_time_ms = 0;
static int64_t s_junction_time_ms = 0;
static int64_t s_turn_time_ms = 0;
static int64_t s_line_lost_time_ms = 0;
static int64_t s_dest_time_ms = 0;
static int64_t s_obstacle_time_ms = 0;
static int64_t s_resume_ignore_until_ms = 0;

static DeliveryLocation s_last_target = DeliveryLocation::NONE;


// =====================================================
// MOVEMENT LOGGING
// =====================================================

static MoveDir s_current_dir = MoveDir::FORWARD;
static bool s_moving = false;
static int64_t s_segment_start_us = 0;

static void flush_segment()
{
    if (s_moving) {
        int64_t now = esp_timer_get_time();

        uint32_t elapsed_ms =
            (uint32_t)((now - s_segment_start_us) / 1000);

        movement_log_record(s_current_dir, elapsed_ms);

        s_moving = false;
    }
}

static void command_motion(MoveDir direction, int speed)
{
    if (!s_moving || direction != s_current_dir) {
        flush_segment();

        s_current_dir = direction;
        s_segment_start_us = esp_timer_get_time();
        s_moving = true;
    }

    switch (direction) {
        case MoveDir::FORWARD:
            motor_forward(speed);
            break;

        case MoveDir::BACKWARD:
            motor_backward(speed);
            break;

        case MoveDir::LEFT:
        case MoveDir::PIVOT_LEFT:
            motor_turn_left(speed);
            break;

        case MoveDir::RIGHT:
        case MoveDir::PIVOT_RIGHT:
            motor_turn_right(speed);
            break;
    }
}

static void stop_movement()
{
    motor_stop();
    flush_segment();
}


// =====================================================
// IR PATTERN HELPERS
// =====================================================

static bool all_black(const LinePattern &p)
{
    return p.left && p.center && p.right;
}

static bool all_white(const LinePattern &p)
{
    return !p.left && !p.center && !p.right;
}


// =====================================================
// OBSTACLE CHECK
// =====================================================

static bool obstacle_detected()
{
    float distances[4];

    robot_state_get_distances(distances);

    bool left_blocked =
        distances[US_FRONT_LEFT] > 0 &&
        distances[US_FRONT_LEFT] <= OBSTACLE_STOP_CM;

    bool center_blocked =
        distances[US_FRONT_CENTER] > 0 &&
        distances[US_FRONT_CENTER] <= OBSTACLE_STOP_CM;

    bool right_blocked =
        distances[US_FRONT_RIGHT] > 0 &&
        distances[US_FRONT_RIGHT] <= OBSTACLE_STOP_CM;

    return left_blocked || center_blocked || right_blocked;
}


// =====================================================
// LINE FOLLOWING
// =====================================================

static void nav_follow_line(const LinePattern &p)
{
    /*
       Logical IR readings:

       0 = WHITE
       1 = BLACK

       Left sensor detects line  -> steer left
       Centre sensor detects line -> forward
       Right sensor detects line  -> steer right
    */

    // Centre only: line is correctly under the robot.
    if (!p.left && p.center && !p.right) {
        command_motion(MoveDir::FORWARD, FOLLOW_SPEED);
    }

    // Left and centre: correct towards the left.
    else if (p.left && p.center && !p.right) {
        command_motion(MoveDir::LEFT, GENTLE_TURN_SPEED);
    }

    // Left only: stronger left correction.
    else if (p.left && !p.center && !p.right) {
        command_motion(MoveDir::LEFT, SHARP_TURN_SPEED);
    }

    // Centre and right: correct towards the right.
    else if (!p.left && p.center && p.right) {
        command_motion(MoveDir::RIGHT, GENTLE_TURN_SPEED);
    }

    // Right only: stronger right correction.
    else if (!p.left && !p.center && p.right) {
        command_motion(MoveDir::RIGHT, SHARP_TURN_SPEED);
    }

    // Both outer sensors black, centre white.
    else if (p.left && !p.center && p.right) {
        command_motion(MoveDir::FORWARD, SPEED_SLOW);
    }
}


// =====================================================
// JUNCTION HANDLING
// =====================================================

static void nav_handle_junction(
    const LinePattern &p,
    DeliveryLocation target,
    int64_t now)
{
    // A goes straight and ignores junctions.
    if (target == DeliveryLocation::LOC_A) {
        command_motion(MoveDir::FORWARD, FOLLOW_SPEED);
        return;
    }

    // B and C only take one selected branch.
    if (s_branch_taken) {
        command_motion(MoveDir::FORWARD, FOLLOW_SPEED);
        return;
    }

    if (s_nav_state == NavState::FOLLOW_LINE) {

        if (all_black(p)) {

            if (s_junction_time_ms == 0) {
                s_junction_time_ms = now;
            }

            if (now - s_junction_time_ms >= JUNCTION_CONFIRM_MS) {

                s_nav_state = NavState::JUNCTION_ROLL;
                s_dest_time_ms = now;

                s_turn_left =
                    (target == DeliveryLocation::LOC_C);

                ESP_LOGI(TAG, "Junction confirmed");
            }
        }
        else {
            s_junction_time_ms = 0;
        }

        command_motion(MoveDir::FORWARD, FOLLOW_SPEED);
    }
}


// =====================================================
// JUNCTION TURNING
// =====================================================

static void nav_execute_junction_turn(const LinePattern &p, int64_t now)
{
    // Roll forward onto the junction.
    if (s_nav_state == NavState::JUNCTION_ROLL) {

        command_motion(MoveDir::FORWARD, FOLLOW_SPEED);

        if (now - s_dest_time_ms >= JUNCTION_ROLL_MS) {

            s_nav_state = NavState::JUNCTION_TURN;

            s_turn_time_ms = now;
            s_center_left_old_line = false;

            ESP_LOGI(TAG, "Starting junction turn");
        }

        return;
    }

    if (s_nav_state == NavState::JUNCTION_TURN) {

        if (s_turn_left) {
            command_motion(MoveDir::LEFT, SHARP_TURN_SPEED);
        }
        else {
            command_motion(MoveDir::RIGHT, SHARP_TURN_SPEED);
        }

        // Wait until the centre sensor leaves the old junction.
        if (!p.center) {
            s_center_left_old_line = true;
        }

        bool branch_found =
            s_center_left_old_line && p.center;

        bool minimum_turn_complete =
            now - s_turn_time_ms >= JUNCTION_TURN_MIN_MS;

        bool turn_timeout =
            now - s_turn_time_ms >= JUNCTION_TURN_TIMEOUT_MS;

        if ((branch_found && minimum_turn_complete) || turn_timeout) {

            s_branch_taken = true;
            s_nav_state = NavState::FOLLOW_LINE;

            s_junction_time_ms = 0;

            ESP_LOGI(TAG, "Junction branch selected");
        }
    }
}


// =====================================================
// DESTINATION VERIFICATION
// =====================================================

static void nav_verify_destination(const LinePattern &p, int64_t now)
{
    if (s_nav_state == NavState::FOLLOW_LINE) {

        if (all_white(p)) {

            if (s_line_lost_time_ms == 0) {
                s_line_lost_time_ms = now;
            }

            if (now - s_line_lost_time_ms >= LINE_LOST_CONFIRM_MS) {

                s_nav_state = NavState::DEST_CREEP;
                s_dest_time_ms = now;

                ESP_LOGI(TAG, "Verifying destination");
            }
        }
        else {
            s_line_lost_time_ms = 0;
        }
    }

    else if (s_nav_state == NavState::DEST_CREEP) {

        // If the line returns, resume normal line following.
        if (!all_white(p)) {

            s_nav_state = NavState::FOLLOW_LINE;
            s_line_lost_time_ms = 0;

            return;
        }

        command_motion(MoveDir::FORWARD, SPEED_SLOW);

        if (now - s_dest_time_ms >= DEST_CREEP_MS) {

            s_nav_state = NavState::DEST_CONFIRM;
            s_dest_time_ms = now;

            stop_movement();
        }
    }

    else if (s_nav_state == NavState::DEST_CONFIRM) {

        if (!all_white(p)) {

            s_nav_state = NavState::FOLLOW_LINE;
            s_line_lost_time_ms = 0;

            ESP_LOGI(TAG, "Line found again; resuming");

            return;
        }

        stop_movement();

        if (now - s_dest_time_ms >= DEST_CONFIRM_MS) {

            s_nav_state = NavState::ARRIVED;
            s_arrived = true;

            stop_movement();

            robot_state_set_powered_off(true);

            ESP_LOGI(TAG, "Destination reached");
        }
    }

    else if (s_nav_state == NavState::ARRIVED) {
        stop_movement();
    }
}


// =====================================================
// INITIALIZATION
// =====================================================

void control_task_init()
{
    s_current_dir = MoveDir::FORWARD;
    s_moving = false;

    s_segment_start_us = esp_timer_get_time();

    movement_log_reset();

    s_nav_state = NavState::FOLLOW_LINE;

    s_branch_taken = false;
    s_turn_left = false;
    s_center_left_old_line = false;
    s_arrived = false;

    s_start_time_ms = esp_timer_get_time() / 1000;

    s_junction_time_ms = 0;
    s_turn_time_ms = 0;
    s_line_lost_time_ms = 0;
    s_dest_time_ms = 0;
    s_obstacle_time_ms = 0;

    s_resume_ignore_until_ms = 0;

    s_last_target = DeliveryLocation::NONE;

    robot_state_set_mode(RobotMode::MANUAL);
    robot_state_set_manual_cmd(ManualCmd::STOP);
    robot_state_set_powered_off(false);
    robot_state_set_target_location(DeliveryLocation::NONE);
}


// =====================================================
// MAIN CONTROL TASK
// =====================================================

void control_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Control task started");

    while (true) {

        int64_t now = esp_timer_get_time() / 1000;

        RobotMode mode = robot_state_get_mode();

        // =============================================
        // MANUAL MODE
        // =============================================

        if (mode == RobotMode::MANUAL) {

            switch (robot_state_get_manual_cmd()) {

                case ManualCmd::FORWARD:
                    command_motion(MoveDir::FORWARD, SPEED_FORWARD);
                    break;

                case ManualCmd::BACKWARD:
                    command_motion(MoveDir::BACKWARD, SPEED_FORWARD);
                    break;

                case ManualCmd::LEFT:
                    command_motion(MoveDir::LEFT, SPEED_TURN);
                    break;

                case ManualCmd::RIGHT:
                    command_motion(MoveDir::RIGHT, SPEED_TURN);
                    break;

                case ManualCmd::STOP:
                case ManualCmd::NONE:
                default:
                    stop_movement();
                    break;
            }

            vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
            continue;
        }

        // =============================================
        // POWER / PAUSE CHECK
        // =============================================

        if (robot_state_is_powered_off()) {
            stop_movement();

            vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
            continue;
        }

        // =============================================
        // READ CURRENT DESTINATION
        // =============================================

        DeliveryLocation target =
            robot_state_get_target_location();

        // Reset route state if a new destination is selected.
        if (target != s_last_target) {

            s_last_target = target;

            s_branch_taken = false;
            s_nav_state = NavState::FOLLOW_LINE;

            s_junction_time_ms = 0;
            s_line_lost_time_ms = 0;

            ESP_LOGI(TAG, "Destination selection changed");
        }

        // =============================================
        // READ IR SENSORS
        // =============================================

        LinePattern p = read_line_pattern();

        // =============================================
        // OBSTACLE DETECTION
        // =============================================

        if (now >= s_resume_ignore_until_ms) {

            if (obstacle_detected()) {

                if (s_obstacle_time_ms == 0) {
                    s_obstacle_time_ms = now;
                }

                if (now - s_obstacle_time_ms >= OBSTACLE_CONFIRM_MS) {

                    stop_movement();

                    robot_state_set_powered_off(true);

                    ESP_LOGW(TAG, "Obstacle detected. Journey paused.");
                }
            }
            else {
                s_obstacle_time_ms = 0;
            }
        }

        if (robot_state_is_powered_off()) {
            vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
            continue;
        }

        // =============================================
        // STARTUP IGNORE
        // =============================================

        if (now - s_start_time_ms < START_IGNORE_MS) {

            command_motion(MoveDir::FORWARD, FOLLOW_SPEED);

            vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
            continue;
        }

        // =============================================
        // DESTINATION VERIFICATION
        // =============================================

        if (s_nav_state == NavState::DEST_CREEP ||
            s_nav_state == NavState::DEST_CONFIRM ||
            s_nav_state == NavState::ARRIVED) {

            nav_verify_destination(p, now);

            vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
            continue;
        }

        // =============================================
        // JUNCTION TURN STATES
        // =============================================

        if (s_nav_state == NavState::JUNCTION_ROLL ||
            s_nav_state == NavState::JUNCTION_TURN) {

            nav_execute_junction_turn(p, now);

            vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
            continue;
        }

        // =============================================
        // JUNCTION DETECTION
        // =============================================

        if (all_black(p)) {

            nav_handle_junction(p, target, now);

            vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
            continue;
        }
        else {
            s_junction_time_ms = 0;
        }

        // =============================================
        // NORMAL CONTINUOUS LINE FOLLOWING
        // =============================================

        nav_follow_line(p);

        vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
    }
}


// =====================================================
// WEBPAGE PLAY / PAUSE
// =====================================================

void control_request_play()
{
    s_arrived = false;

    if (s_nav_state == NavState::ARRIVED) {
        s_nav_state = NavState::FOLLOW_LINE;
        s_branch_taken = false;
    }

    robot_state_set_powered_off(false);
    robot_state_set_mode(RobotMode::LINE_FOLLOW);
    robot_state_set_manual_cmd(ManualCmd::STOP);

    s_resume_ignore_until_ms =
        (esp_timer_get_time() / 1000) + RESUME_GRACE_MS;

    s_obstacle_time_ms = 0;

    ESP_LOGI(TAG, "Journey resumed");
}


void control_request_pause()
{
    robot_state_set_mode(RobotMode::MANUAL);
    robot_state_set_manual_cmd(ManualCmd::STOP);

    stop_movement();

    ESP_LOGI(TAG, "Journey paused");
}


// =====================================================
// JOURNEY STATUS
// =====================================================

const char *control_journey_status()
{
    if (s_arrived) {
        return "arrived";
    }

    if (robot_state_get_mode() == RobotMode::LINE_FOLLOW) {

        return robot_state_is_powered_off()
            ? "paused"
            : "running";
    }

    return robot_state_get_target_location() == DeliveryLocation::NONE
        ? "idle"
        : "waiting";
}
