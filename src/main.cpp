#include <cstdint>
#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "Bot_config.h"
#include "IR.h"
#include "movement.h"
#include "Ultrasonic.h"


#ifndef DEBUG_PRINT_DISTANCES
#define DEBUG_PRINT_DISTANCES 1    
#endif
#ifndef ULTRASONIC_GAP_MS
#define ULTRASONIC_GAP_MS 35        
#endif

static const char *TAG = "ROBOT";


enum class RobotState {
    FollowLine,      
    CheckSides,      
    TurnOut,       
    MoveOut,         
    TurnParallel,  
    MoveAlong,       
    TurnToLine,     
    ReturnToLine,    
    SearchForLine,  
    AlignWithLine, 
    Stopped
};

enum class Direction { Left, Right };
enum class Leg { Running, Done, Failed };

static RobotState  current_state         = RobotState::FollowLine;
static Direction   bypass_direction      = Direction::Right;
static uint32_t    state_start_ms        = 0;
static uint32_t    last_front_check_ms   = 0;
static uint32_t    out_leg_ms            = 0;
static bool        side_seen             = false;
static int         free_count            = 0;
static int         front_hits            = 0; 
static int         side_recheck_count    = 0;
static int         search_round          = 0;
static int         search_phase          = 0;      
static uint32_t    search_phase_start_ms = 0;

static distances_t dist                  = {999, 999, 999};

#if DEBUG_PRINT_DISTANCES
static uint32_t    last_print_ms         = 0;
static uint32_t    last_side_read_ms     = 0;
#endif

// ---------------- Helpers ----------------
static uint32_t millis_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static const char *state_name(RobotState s)
{
    switch (s) {
        case RobotState::FollowLine:    return "FOLLOW_LINE";
        case RobotState::CheckSides:    return "CHECK_SIDES";
        case RobotState::TurnOut:       return "TURN_OUT";
        case RobotState::MoveOut:       return "MOVE_OUT";
        case RobotState::TurnParallel:  return "TURN_PARALLEL";
        case RobotState::MoveAlong:     return "MOVE_ALONG";
        case RobotState::TurnToLine:    return "TURN_TO_LINE";
        case RobotState::ReturnToLine:  return "RETURN_TO_LINE";
        case RobotState::SearchForLine: return "SEARCH_FOR_LINE";
        case RobotState::AlignWithLine: return "ALIGN_WITH_LINE";
        default:                        return "STOPPED";
    }
}

static void change_state(RobotState new_state)
{
    current_state  = new_state;
    state_start_ms = millis_now();
    ESP_LOGI(TAG, "State -> %s", state_name(new_state));
}

// Stop and say exactly why, with the latest distances
static void enter_stopped(const char *reason)
{
    stop_car();
    ESP_LOGE(TAG, "STOPPED: %s (front %d, left %d, right %d cm)",
             reason, dist.front, dist.left, dist.right);
    change_state(RobotState::Stopped);
}

static Direction opposite(Direction d)
{
    return (d == Direction::Left) ? Direction::Right : Direction::Left;
}

static void turn_dir(Direction d)
{
    if (d == Direction::Left) {
        turn_left();
    } else {
        turn_right();
    }
}

// Pivot time for a turn in the given direction
static uint32_t turn_time_ms(Direction d)
{
    return (d == Direction::Left) ? TURN_LEFT_MS : TURN_RIGHT_MS;
}


static int read_front(bool gap)
{
    dist.front = ultrasonic_measure_cm(MID_TRIG, MID_ECHO);
    if (gap) DELAY_MS(ULTRASONIC_GAP_MS);
    return dist.front;
}

static int read_left(bool gap)
{
    dist.left = ultrasonic_measure_cm(LF_TRIG, LF_ECHO);
    if (gap) DELAY_MS(ULTRASONIC_GAP_MS);
    return dist.left;
}

static int read_right(bool gap)
{
    dist.right = ultrasonic_measure_cm(RG_TRIG, RG_ECHO);
    if (gap) DELAY_MS(ULTRASONIC_GAP_MS);
    return dist.right;
}

#if DEBUG_PRINT_DISTANCES
static void fmt_cm(char *buf, size_t n, int cm)
{
    if (cm >= 999) {
        snprintf(buf, n, "---");   // no echo
    } else {
        snprintf(buf, n, "%d", cm);
    }
}

// Throttled status line, e.g.  [FOLLOW_LINE    ] Front:   42 cm | Left:   31 cm | Right:  --- cm
static void print_distances(void)
{
    uint32_t now = millis_now();
    if (now - last_print_ms < DEBUG_PRINT_INTERVAL_MS) {
        return;
    }
    last_print_ms = now;

    char f[12], l[12], r[12];
    fmt_cm(f, sizeof(f), dist.front);
    fmt_cm(l, sizeof(l), dist.left);
    fmt_cm(r, sizeof(r), dist.right);

    printf("[%-15s] Front: %4s cm | Left: %4s cm | Right: %4s cm\n",
           state_name(current_state), f, l, r);
}
#endif

static bool path_blocked(void)
{
    return read_front(true) <= BLOCKED_DISTANCE;
}

static void reset_free_detection(void)
{
    side_seen  = false;
    free_count = 0;
}

// The obstacle is on the opposite side to the bypass direction. It counts as
// "free" once that side sensor has seen it and then reads clear several times in a row.
// If the side sensor never sees it (small or off-axis obstacle), assume it is passed after SEEN_TIMEOUT_MS.
static bool obstacle_side_is_free(void)
{
    int d = (bypass_direction == Direction::Right) ? read_left(true) : read_right(true);

    if (d <= CLEAR_DISTANCE) {
        side_seen  = true;
        free_count = 0;
    } else if (side_seen) {
        free_count++;
    }

    bool never_seen = !side_seen && (millis_now() - state_start_ms) > SEEN_TIMEOUT_MS;
    if (never_seen) {
        ESP_LOGW(TAG, "Side sensor never saw the obstacle, assuming it is passed");
    }

    return (side_seen && free_count >= FREE_READINGS) || never_seen;
}

// One step of "drive forward until the obstacle is beside-free"
static Leg step_leg_until_free(void)
{
    move_forward();

    if (path_blocked()) {
        enter_stopped("path blocked during bypass");
        return Leg::Failed;
    }

    if ((millis_now() - state_start_ms) > MAX_LEG_MS) {
        enter_stopped("bypass leg timed out");
        return Leg::Failed;
    }

    if (obstacle_side_is_free()) {
        DELAY_MS(CLEAR_MARGIN_MS);   // keep rolling so the whole body clears the obstacle's edge
        return Leg::Done;
    }

    return Leg::Running;
}

// One step of normal line following based on the three IR sensors
static void follow_line(void)
{
    bool left   = ir_read_left();
    bool middle = ir_read_mid();
    bool right  = ir_read_right();

    if (middle && !left && !right) {
        move_forward();
    } else if (left && middle) {
        turn_left();
    } else if (right && middle) {
        turn_right();
    } else if (left) {
        turn_left();
    } else if (right) {
        turn_right();
    } else {
        stop_car();
    }
}

// Gated front safety check while driving toward the line.
// Returns true if it stopped the car.
static bool front_blocked_check(const char *reason)
{
    uint32_t now = millis_now();
    if (now - last_front_check_ms < FRONT_CHECK_INTERVAL_MS) {
        return false;
    }
    last_front_check_ms = now;

    if (read_front(false) <= BLOCKED_DISTANCE) {
        enter_stopped(reason);
        return true;
    }
    return false;
}

// An IR sensor just saw the line: roll on until the wheels are over it, then align
static void line_found(void)
{
    move_forward_speed(SLOW_SPEED);
    DELAY_MS(AXLE_TO_LINE_MS);
    stop_car();
    change_state(RobotState::AlignWithLine);
}

static void start_search(void)
{
    search_round          = 1;
    search_phase          = 0;
    search_phase_start_ms = millis_now();
    change_state(RobotState::SearchForLine);
}

// Pick the clearer side to go around the obstacle.
// If both sides look blocked, look again a few times before giving up.
static void choose_bypass_direction(void)
{
    read_left(true);
    read_right(true);
    ESP_LOGI(TAG, "Sides: left %d cm, right %d cm", dist.left, dist.right);

    if (dist.left >= SIDE_CLEARANCE && dist.right >= SIDE_CLEARANCE) {
        bypass_direction = (dist.left > dist.right) ? Direction::Left : Direction::Right;
    } else if (dist.left >= SIDE_CLEARANCE) {
        bypass_direction = Direction::Left;
    } else if (dist.right >= SIDE_CLEARANCE) {
        bypass_direction = Direction::Right;
    } else {
        side_recheck_count++;

        if (side_recheck_count > SIDE_RECHECKS) {
            enter_stopped("both sides blocked");
        } else {
            ESP_LOGW(TAG, "Both sides blocked, looking again (%d/%d)", side_recheck_count, SIDE_RECHECKS);
            DELAY_MS(SIDE_RECHECK_DELAY_MS);   // stay in CheckSides and re-read
        }
        return;
    }

    ESP_LOGI(TAG, "Bypassing to the %s", bypass_direction == Direction::Left ? "left" : "right");
    change_state(RobotState::TurnOut);
}

// ---------------- Main ---------------
extern "C" void app_main(void)
{
    lineSensor();
    movement_init();
    ultrasonic_init();

#if DEBUG_PRINT_DISTANCES
    printf("\n=== Obstacle bot: ultrasonic debug on (--- = no echo) ===\n");
#endif

    DELAY_MS(1000);

    while (true) {
        uint32_t elapsed = millis_now() - state_start_ms;

        switch (current_state) {

        // 1. Follow the line; poll the front sensor every FRONT_CHECK_INTERVAL_MS
        case RobotState::FollowLine: {
            uint32_t now = millis_now();

            if (now - last_front_check_ms >= FRONT_CHECK_INTERVAL_MS) {
                last_front_check_ms = now;
                int front = read_front(false);

                // Must read close several times in a row, so one noisy reading can't trigger a bypass
                front_hits = (front <= OBSTACLE_DISTANCE) ? front_hits + 1 : 0;

                if (front_hits >= OBSTACLE_CONFIRM_READINGS) {
                    front_hits = 0;
                    stop_car();
                    ESP_LOGI(TAG, "Obstacle at %d cm", front);
                    side_recheck_count = 0;
                    change_state(RobotState::CheckSides);
                    break;
                }

#if DEBUG_PRINT_DISTANCES
                // Debug view: refresh the side sensors too (costs ~70 ms of line-following time)
                if (now - last_side_read_ms >= DEBUG_PRINT_INTERVAL_MS) {
                    last_side_read_ms = now;
                    DELAY_MS(ULTRASONIC_GAP_MS);
                    read_left(true);
                    read_right(true);
                }
#endif
            }

            follow_line();
            break;
        }

        // 2. Look left and right, choose the freer side
        case RobotState::CheckSides:
            stop_car();
            choose_bypass_direction();
            break;

        // 3. Turn toward the free side
        case RobotState::TurnOut:
            turn_dir(bypass_direction);

            if (elapsed >= turn_time_ms(bypass_direction)) {
                stop_car();
                reset_free_detection();
                change_state(RobotState::MoveOut);
            }
            break;

        // 4. Leg 1: move until free, and record how far we went
        case RobotState::MoveOut: {
            Leg result = step_leg_until_free();

            if (result == Leg::Done) {
                out_leg_ms = millis_now() - state_start_ms;
                ESP_LOGI(TAG, "Out leg recorded: %u ms (~%d cm)",
                         (unsigned)out_leg_ms,
                         (int)(out_leg_ms * APPROX_SPEED_CM_PER_S / 1000.0f));
                stop_car();
                change_state(RobotState::TurnParallel);
            }
            break;   // Leg::Failed has already moved to Stopped
        }

        // 5. Turn back to the original heading
        case RobotState::TurnParallel:
            turn_dir(opposite(bypass_direction));

            if (elapsed >= turn_time_ms(opposite(bypass_direction))) {
                stop_car();
                reset_free_detection();
                change_state(RobotState::MoveAlong);
            }
            break;

        // 6. Leg 2: move until free again (we are past the obstacle)
        case RobotState::MoveAlong: {
            Leg result = step_leg_until_free();

            if (result == Leg::Done) {
                stop_car();
                change_state(RobotState::TurnToLine);
            }
            break;
        }

        // 7. Turn toward the line
        case RobotState::TurnToLine:
            turn_dir(opposite(bypass_direction));

            if (elapsed >= turn_time_ms(opposite(bypass_direction))) {
                stop_car();
                change_state(RobotState::ReturnToLine);
            }
            break;

        // 8. Leg 3: drive back until an IR sensor sees the line.
        //    Fast for most of the recorded distance, slow near where the line should be.
        case RobotState::ReturnToLine: {
            if (ir_line_detected()) {
                ESP_LOGI(TAG, "Line found on the return leg after %u ms (out leg was %u ms)",
                         (unsigned)elapsed, (unsigned)out_leg_ms);
                line_found();
                break;
            }

            uint32_t slow_after_ms = out_leg_ms * RETURN_SLOW_START_PCT / 100;
            uint32_t give_up_ms    = out_leg_ms + RETURN_EXTRA_MS;

            if (elapsed >= give_up_ms) {
                ESP_LOGW(TAG, "Line not at the expected distance, searching");
                start_search();
                break;
            }

            if (elapsed < slow_after_ms) {
                move_forward();
            } else {
                move_forward_speed(SLOW_SPEED);
            }

            front_blocked_check("path blocked while returning to the line");
            break;
        }

        // 8b. Line not found where expected: sweep one way, sweep back past centre,
        //     re-centre, creep forward, and repeat with wider sweeps.
        case RobotState::SearchForLine: {
            if (ir_line_detected()) {
                ESP_LOGI(TAG, "Line found by the search (round %d)", search_round);
                line_found();
                break;
            }

            uint32_t amplitude = (uint32_t)search_round * SEARCH_STEP_MS;
            uint32_t phase_len = amplitude;
            if (search_phase == 1) phase_len = 2 * amplitude;
            if (search_phase == 3) phase_len = SEARCH_CREEP_MS;

            if (millis_now() - search_phase_start_ms >= phase_len) {
                search_phase_start_ms = millis_now();
                search_phase++;

                if (search_phase > 3) {
                    search_phase = 0;
                    search_round++;

                    if (search_round > SEARCH_ROUNDS) {
                        enter_stopped("line not found after searching");
                        break;
                    }
                    ESP_LOGI(TAG, "Search round %d", search_round);
                }
            }

            if (search_phase == 3) {
                move_forward_speed(SLOW_SPEED);
                if (front_blocked_check("path blocked while searching for the line")) {
                    break;
                }
            } else if (search_phase == 1) {
                turn_dir(bypass_direction);
            } else {
                turn_dir(opposite(bypass_direction));
            }
            break;
        }

        // 9. Pivot back to the original heading; the IR sensors then take over
        case RobotState::AlignWithLine:
            turn_dir(bypass_direction);

            if (elapsed >= ALIGN_MIN_MS && ir_read_mid()) {
                stop_car();
                change_state(RobotState::FollowLine);
            } else if (elapsed >= ALIGN_TIMEOUT_MS) {
                stop_car();
                ESP_LOGW(TAG, "Align timed out, handing over to line following");
                change_state(RobotState::FollowLine);
            }
            break;

        // 10. Safe stop (terminal; the reason was logged by enter_stopped)
        case RobotState::Stopped:
            stop_car();
            break;
        }

#if DEBUG_PRINT_DISTANCES
        print_distances();
#endif

        vTaskDelay(1);  // yield so the idle task and watchdog can run
    }
}