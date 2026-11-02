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
#define NAV_SWAP_TURNS  0
#define NAV_SWAP_SIDES  0
#define NAV_SIDE_SENSOR_SCALE_PCT  100
namespace {
constexpr uint32_t NAV_PERIOD_MS                 = 10;  
constexpr uint32_t NAV_START_IGNORE_MS           = 500; 
constexpr uint32_t NAV_JUNCTION_CONFIRM_MS       = 40;  
constexpr uint32_t NAV_JUNCTION_ROLL_MS          = 300;
constexpr uint32_t NAV_JUNCTION_TURN_MIN_MS      = 200; 
constexpr uint32_t NAV_JUNCTION_TURN_TIMEOUT_MS  = 1500;
constexpr uint32_t NAV_JUNCTION_IGNORE_MS        = 1000;
constexpr uint32_t NAV_LINE_LOST_STOP_MS         = 600; 
constexpr int      NAV_OBSTACLE_CM               = 25; 
constexpr uint32_t NAV_OBSTACLE_CONFIRM_MS       = 120;
constexpr int      NAV_SIDE_CLEARANCE_CM         = 25;  
constexpr int      NAV_SAFE_CM                   = 20;
constexpr int      NAV_SAFE_TOLERANCE_CM         = 4;
constexpr int      NAV_LOST_CM                   = 45;  
constexpr uint32_t NAV_LOST_CONFIRM_MS           = 150;
constexpr int      NAV_CONTOUR_FRONT_CM          = 18; 
constexpr uint32_t NAV_INNER_CONFIRM_MS          = 80;
constexpr int      NAV_BLOCKED_CM                = 10;  
constexpr uint32_t NAV_BLOCKED_CONFIRM_MS        = 100;
constexpr int      NAV_MAX_INNER_CORNERS         = 4;
constexpr uint32_t NAV_CORNER_CLEAR_MS           = 300; 
constexpr uint32_t NAV_SEEN_TIMEOUT_MS           = 1500;
constexpr uint32_t NAV_MAX_CONTOUR_MS            = 20000;
constexpr int      NAV_SIDE_RECHECKS             = 5;   
constexpr uint32_t NAV_SIDE_RECHECK_DELAY_MS     = 500;

constexpr uint32_t NAV_TURN_LEFT_MS              = 350;
constexpr uint32_t NAV_TURN_RIGHT_MS             = 350;
constexpr int      NAV_RETURN_SLOW_START_PCT     = 80;  
constexpr uint32_t NAV_RETURN_EXTRA_MS           = 1500;
constexpr uint32_t NAV_AXLE_TO_LINE_MS           = 300; 
constexpr uint32_t NAV_ALIGN_MIN_MS              = 150;
constexpr uint32_t NAV_ALIGN_TIMEOUT_MS          = 1200;
constexpr uint32_t NAV_SEARCH_STEP_MS            = 150; 
constexpr uint32_t NAV_SEARCH_CREEP_MS           = 250;
constexpr int      NAV_SEARCH_ROUNDS             = 4;   
constexpr uint32_t NAV_SEARCH_TIMEOUT_MS         = 8000;
constexpr float    NAV_APPROX_SPEED_CM_PER_S     = 25.0f;
}  


static MoveDir s_current_dir = MoveDir::FORWARD;
static bool s_moving = false;
static int64_t s_segment_start_us = 0;


static void flush_segment() {
    if (s_moving) {
        int64_t now = esp_timer_get_time();
        uint32_t elapsed_ms = (uint32_t)((now - s_segment_start_us) / 1000);
        movement_log_record(s_current_dir, elapsed_ms);
        s_moving = false;
    }
}


static void begin_segment(MoveDir dir) {
    if (s_moving && dir == s_current_dir) return;
    flush_segment();
    s_current_dir = dir;
    s_segment_start_us = esp_timer_get_time();
    s_moving = true;
}

static void stop_and_flush() {
    flush_segment();
    motor_stop();
}

static void set_active_direction(MoveDir dir, int speed) {
    if (s_moving && dir == s_current_dir) return;
    begin_segment(dir);

    switch (dir) {
        case MoveDir::FORWARD:  motor_forward(speed); break;
        case MoveDir::BACKWARD: motor_backward(speed); break;
        case MoveDir::LEFT:     motor_turn_left(speed); break;
        case MoveDir::RIGHT:    motor_turn_right(speed); break;
        case MoveDir::PIVOT_LEFT:  motor_turn_left(speed); break;
        case MoveDir::PIVOT_RIGHT: motor_turn_right(speed); break;
    }
}


static int g_speed = 255;
static int g_turn  = 255;
static int g_slow  = 180;

static void nav_drive(int left_speed, int right_speed) {
#if NAV_SWAP_SIDES
    int t = left_speed; left_speed = right_speed; right_speed = t;
#endif
    begin_segment(MoveDir::FORWARD);
    motor_forward(left_speed > right_speed ? left_speed : right_speed);   
    motor_set_left_speed(left_speed);                                     
    motor_set_right_speed(right_speed);
}

static void nav_turn(bool left) {
#if NAV_SWAP_TURNS
    left = !left;
#endif
    set_active_direction(left ? MoveDir::LEFT : MoveDir::RIGHT, g_turn);
}

static void nav_stop() {
    stop_and_flush();
}

struct NavInputs {
    int  front_cm;      // 999 = no echo
    int  left_cm;
    int  right_cm;
    bool ir_left;       // true = black
    bool ir_mid;
    bool ir_right;
    char destination;   // 'A' straight, 'B' right, 'C' left
};

static int cm_or_none(float v) {
    return (v > 0.0f) ? (int)v : 999;
}

static void nav_read_inputs(NavInputs &in) {
    float d[4];
    robot_state_get_distances(d);

    in.front_cm = cm_or_none(d[US_FRONT_CENTER]);
    in.left_cm  = cm_or_none(d[US_FRONT_LEFT]);
    in.right_cm = cm_or_none(d[US_FRONT_RIGHT]);

    if (in.left_cm  < 999) in.left_cm  = in.left_cm  * NAV_SIDE_SENSOR_SCALE_PCT / 100;
    if (in.right_cm < 999) in.right_cm = in.right_cm * NAV_SIDE_SENSOR_SCALE_PCT / 100;

    LinePattern p = read_line_pattern();
    in.ir_left  = p.left;
    in.ir_mid   = p.center;
    in.ir_right = p.right;

    switch (robot_state_get_target_location()) {
        case DeliveryLocation::LOC_B: in.destination = 'B'; break;   // right
        case DeliveryLocation::LOC_C: in.destination = 'C'; break;   // left
        default:                      in.destination = 'A'; break;   // straight
    }
}


enum class NavState {
    Idle,                 // line mode just started
    FollowToJunction,     // line following until the junction
    TurnAtJunction,       // turn onto the right or left branch
    FollowToDestination,  // line following after the junction until the end marker / end of line
    CheckSides,           // bypass: pick the freer side
    TurnOut,              // bypass: turn toward it and start the path record
    Contour,              // bypass: go round the obstacle at a safe distance
    ReturnToLine,         // bypass: drive back the recorded sideways distance
    SearchForLine,        // bypass: line not where expected, sweep and creep
    AlignWithLine,        // bypass: turn back to the original heading
    Arrived,              // destination reached: stopped until line mode is switched off and on again
    Stopped               // error stop: same
};

enum class NavDir { Left, Right };

struct Hold { bool on; uint32_t since; };   // "true continuously for N ms" helper

static NavState    s_state             = NavState::Idle;
static NavState    s_resume_state      = NavState::FollowToJunction;   // where to go after a bypass
static uint32_t    s_state_start_ms    = 0;
static NavDir      s_bypass_dir        = NavDir::Right;
static NavDir      s_junction_turn     = NavDir::Right;
static uint32_t    s_junction_ignore_until_ms = 0;
static int         s_side_recheck      = 0;

// Path record while going round the obstacle (time driven at constant speed)
static int32_t     s_lateral_ms        = 0;   // + = away from the line, - = back toward it
static int32_t     s_forward_ms        = 0;   // + = along the original heading
static int         s_heading_quarters  = 0;   // 90-degree turns toward the bypass side since the bypass began
static uint32_t    s_out_leg_ms        = 0;   // recorded sideways distance used to avoid the obstacle
static uint32_t    s_last_contour_ms   = 0;
static uint32_t    s_face_start_ms     = 0;
static bool        s_side_seen         = false;
static int         s_inner_corners     = 0;

static int         s_search_round      = 0;
static int         s_search_phase      = 0;   // 0 sweep, 1 sweep back, 2 re-centre, 3 creep
static uint32_t    s_search_phase_ms   = 0;

static Hold        s_front_hold, s_bar_hold, s_lost_line_hold, s_wall_lost_hold, s_inner_hold, s_blocked_hold;

static uint32_t nav_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static void nav_delay(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// true once cond has been true without a break for ms
static bool nav_hold(Hold &h, bool cond, uint32_t ms) {
    if (!cond) {
        h.on = false;
        return false;
    }
    if (!h.on) {
        h.on = true;
        h.since = nav_millis();
    }
    return (nav_millis() - h.since) >= ms;
}

static void nav_reset_holds() {
    s_front_hold.on = s_bar_hold.on = s_lost_line_hold.on = false;
    s_wall_lost_hold.on = s_inner_hold.on = s_blocked_hold.on = false;
}

static const char *nav_state_name(NavState s) {
    switch (s) {
        case NavState::Idle:                return "IDLE";
        case NavState::FollowToJunction:    return "FOLLOW_TO_JUNCTION";
        case NavState::TurnAtJunction:      return "TURN_AT_JUNCTION";
        case NavState::FollowToDestination: return "FOLLOW_TO_DESTINATION";
        case NavState::CheckSides:          return "CHECK_SIDES";
        case NavState::TurnOut:             return "TURN_OUT";
        case NavState::Contour:             return "CONTOUR";
        case NavState::ReturnToLine:        return "RETURN_TO_LINE";
        case NavState::SearchForLine:       return "SEARCH_FOR_LINE";
        case NavState::AlignWithLine:       return "ALIGN_WITH_LINE";
        case NavState::Arrived:             return "ARRIVED";
        default:                            return "STOPPED";
    }
}

static void nav_change_state(NavState s) {
    s_state = s;
    s_state_start_ms = nav_millis();
    nav_reset_holds();
    ESP_LOGI(TAG, "State -> %s", nav_state_name(s));
}

static NavDir nav_opposite(NavDir d) {
    return (d == NavDir::Left) ? NavDir::Right : NavDir::Left;
}

static void nav_turn_dir(NavDir d) {
    nav_turn(d == NavDir::Left);
}


static void nav_enter_stopped(const char *reason, const NavInputs &in) {
    nav_stop();
    ESP_LOGE(TAG, "STOPPED: %s (front %d, left %d, right %d cm)", reason, in.front_cm, in.left_cm, in.right_cm);
    nav_change_state(NavState::Stopped);
}

static void nav_arrived(char destination) {
    nav_stop();
    ESP_LOGI(TAG, "Arrived at location %c", destination);
    nav_change_state(NavState::Arrived);
}


static void nav_follow_line(const NavInputs &in, bool all_black_goes_straight) {
    bool l = in.ir_left, m = in.ir_mid, r = in.ir_right;

    if (m && !l && !r) {
        nav_drive(g_speed, g_speed);
    } else if (l && m && r && all_black_goes_straight) {
        nav_drive(g_slow, g_slow);
    } else if (l && m) {
        nav_turn(true);
    } else if (r && m) {
        nav_turn(false);
    } else if (l) {
        nav_turn(true);
    } else if (r) {
        nav_turn(false);
    } else {
        nav_drive(g_slow, g_slow);
    }
}

// ---- Junction ----
static void nav_take_junction(char dest) {
    ESP_LOGI(TAG, "Junction: location %c -> %s", dest,
             dest == 'B' ? "turn right" : (dest == 'C' ? "turn left" : "straight on"));

    
    nav_drive(g_slow, g_slow);
    nav_delay(NAV_JUNCTION_ROLL_MS);
    nav_stop();

    if (dest == 'B' || dest == 'C') {
        s_junction_turn = (dest == 'B') ? NavDir::Right : NavDir::Left;
        nav_change_state(NavState::TurnAtJunction);
    } else {
        s_junction_ignore_until_ms = nav_millis() + NAV_JUNCTION_IGNORE_MS;
        nav_change_state(NavState::FollowToDestination);
    }
}
static bool nav_check_obstacle(const NavInputs &in) {
    if (nav_hold(s_front_hold, in.front_cm <= NAV_OBSTACLE_CM, NAV_OBSTACLE_CONFIRM_MS)) {
        nav_stop();
        s_resume_state = s_state;
        s_side_recheck = 0;
        ESP_LOGI(TAG, "Obstacle at %d cm", in.front_cm);
        nav_change_state(NavState::CheckSides);
        return true;
    }
    return false;
}

static int nav_heading_index() {
    return ((s_heading_quarters % 4) + 4) % 4;  
}


static void nav_odo_add(uint32_t ms) {
    int32_t t = (int32_t)ms;

    switch (nav_heading_index()) {
        case 0:  s_forward_ms += t; break;
        case 1:  s_lateral_ms += t; break;
        case 2:  s_forward_ms -= t; break;
        default: s_lateral_ms -= t; break;
    }
}

static void nav_pivot(NavDir d) {
    nav_turn_dir(d);
    nav_delay(d == NavDir::Left ? NAV_TURN_LEFT_MS : NAV_TURN_RIGHT_MS);
    nav_stop();

    s_heading_quarters += (d == s_bypass_dir) ? 1 : -1;
    ESP_LOGI(TAG, "Turn %s, heading index %d", d == NavDir::Left ? "left" : "right", nav_heading_index());
}

static void nav_reset_face() {
    s_side_seen = false;
    s_face_start_ms = nav_millis();
    s_last_contour_ms = nav_millis();
    s_wall_lost_hold.on = false;
    s_inner_hold.on = false;
}

static void nav_steer_to_safe_distance(int side_cm) {
    bool wall_on_left = (s_bypass_dir == NavDir::Right);
    int  error        = side_cm - NAV_SAFE_CM;   
    int  left_speed   = g_speed;
    int  right_speed  = g_speed;

    if (error > NAV_SAFE_TOLERANCE_CM) {            
        if (wall_on_left) left_speed  = g_slow;
        else              right_speed = g_slow;
    } else if (error < -NAV_SAFE_TOLERANCE_CM) {
        if (wall_on_left) right_speed = g_slow;
        else              left_speed  = g_slow;
    }

    nav_drive(left_speed, right_speed);
}

static void nav_choose_bypass(const NavInputs &in) {
    int l = in.left_cm;
    int r = in.right_cm;
    ESP_LOGI(TAG, "Sides: left %d cm, right %d cm", l, r);

    if (l >= NAV_SIDE_CLEARANCE_CM && r >= NAV_SIDE_CLEARANCE_CM) {
        s_bypass_dir = (l > r) ? NavDir::Left : NavDir::Right;
    } else if (l >= NAV_SIDE_CLEARANCE_CM) {
        s_bypass_dir = NavDir::Left;
    } else if (r >= NAV_SIDE_CLEARANCE_CM) {
        s_bypass_dir = NavDir::Right;
    } else {
        s_side_recheck++;

        if (s_side_recheck > NAV_SIDE_RECHECKS) {
            nav_enter_stopped("both sides blocked", in);
        } else {
            ESP_LOGW(TAG, "Both sides blocked, looking again (%d/%d)", s_side_recheck, NAV_SIDE_RECHECKS);
            nav_delay(NAV_SIDE_RECHECK_DELAY_MS);  
        }
        return;
    }

    ESP_LOGI(TAG, "Bypassing to the %s", s_bypass_dir == NavDir::Left ? "left" : "right");
    nav_change_state(NavState::TurnOut);
}


static void nav_line_found() {
    nav_drive(g_slow, g_slow);
    nav_delay(NAV_AXLE_TO_LINE_MS);
    nav_stop();
    nav_change_state(NavState::AlignWithLine);
}

static void nav_start_search() {
    s_search_round = 1;
    s_search_phase = 0;
    s_search_phase_ms = nav_millis();
    nav_change_state(NavState::SearchForLine);
}


static void nav_reset() {
    if (s_state != NavState::Idle) {
        ESP_LOGI(TAG, "Run dropped (manual mode, power off or unlock)");
        nav_change_state(NavState::Idle);
    }
}

static void nav_step(const NavInputs &in) {
    uint32_t now     = nav_millis();
    uint32_t elapsed = now - s_state_start_ms;

    switch (s_state) {


    case NavState::Idle:
        ESP_LOGI(TAG, "Run started: A = straight, B = right, C = left at the junction");
        movement_log_reset();
        s_junction_ignore_until_ms = now + NAV_START_IGNORE_MS;
        nav_change_state(NavState::FollowToJunction);
        break;

    case NavState::FollowToJunction:
    case NavState::FollowToDestination: {
        bool after_junction = (s_state == NavState::FollowToDestination);
        bool ignoring       = (now < s_junction_ignore_until_ms);
        bool all_black      = in.ir_left && in.ir_mid && in.ir_right;
        bool all_white      = !in.ir_left && !in.ir_mid && !in.ir_right;

        if (nav_check_obstacle(in)) {
            break;
        }

        if (!ignoring && nav_hold(s_bar_hold, all_black, NAV_JUNCTION_CONFIRM_MS)) {
            if (after_junction) {
                nav_arrived(in.destination);          
            } else {
                nav_take_junction(in.destination);    
            }
            break;
        }

        if (nav_hold(s_lost_line_hold, all_white, NAV_LINE_LOST_STOP_MS)) {
            if (after_junction) {
                nav_arrived(in.destination);          
            } else {
                nav_enter_stopped("line lost before the junction", in);
            }
            break;
        }

        nav_follow_line(in, ignoring);
        break;
    }

    case NavState::TurnAtJunction:
        nav_turn_dir(s_junction_turn);

        if ((elapsed >= NAV_JUNCTION_TURN_MIN_MS && in.ir_mid) || elapsed >= NAV_JUNCTION_TURN_TIMEOUT_MS) {
            if (!in.ir_mid) {
                ESP_LOGW(TAG, "Branch not found while turning, carrying on");
            }
            nav_stop();
            s_junction_ignore_until_ms = now + NAV_JUNCTION_IGNORE_MS;
            nav_change_state(NavState::FollowToDestination);
        }
        break;


    case NavState::CheckSides:
        nav_stop();
        nav_choose_bypass(in);
        break;

    case NavState::TurnOut:
        s_lateral_ms = 0;
        s_forward_ms = 0;
        s_heading_quarters = 0;
        s_inner_corners = 0;

        nav_pivot(s_bypass_dir);   
        nav_reset_face();
        nav_change_state(NavState::Contour);
        break;

    case NavState::Contour: {
        nav_odo_add(now - s_last_contour_ms);
        s_last_contour_ms = now;

        if (elapsed > NAV_MAX_CONTOUR_MS) {
            nav_enter_stopped("went round the obstacle for too long", in);
            break;
        }

        int side  = (s_bypass_dir == NavDir::Right) ? in.left_cm : in.right_cm;
        int front = in.front_cm;

        if (side <= NAV_LOST_CM) {
            s_side_seen = true;
        }
        bool wall_gone = nav_hold(s_wall_lost_hold, side > NAV_LOST_CM, NAV_LOST_CONFIRM_MS);


        if (nav_hold(s_inner_hold, front <= NAV_CONTOUR_FRONT_CM, NAV_INNER_CONFIRM_MS)) {
            nav_stop();
            s_inner_corners++;

            if (s_inner_corners > NAV_MAX_INNER_CORNERS) {
                nav_enter_stopped("boxed in while going round the obstacle", in);
                break;
            }
            ESP_LOGI(TAG, "Inner corner (front %d cm), turning away", front);
            nav_pivot(s_bypass_dir);
            nav_reset_face();
            break;
        }


        bool edge_passed = (s_side_seen && wall_gone) ||
                           (!s_side_seen && (now - s_face_start_ms) > NAV_SEEN_TIMEOUT_MS);

        if (edge_passed) {
            ESP_LOGI(TAG, "Edge passed, rolling on then turning around it");
            nav_drive(g_speed, g_speed);
            nav_delay(NAV_CORNER_CLEAR_MS);      
            nav_odo_add(NAV_CORNER_CLEAR_MS);
            nav_pivot(nav_opposite(s_bypass_dir));   
            nav_reset_face();

            if (nav_heading_index() == 3) {
                s_out_leg_ms = (s_lateral_ms > 0) ? (uint32_t)s_lateral_ms : 0;
                ESP_LOGI(TAG, "Path record: sideways %d ms (~%d cm), forward %d ms (~%d cm)",
                         (int)s_lateral_ms, (int)(s_lateral_ms * NAV_APPROX_SPEED_CM_PER_S / 1000.0f),
                         (int)s_forward_ms, (int)(s_forward_ms * NAV_APPROX_SPEED_CM_PER_S / 1000.0f));
                nav_change_state(NavState::ReturnToLine);
            }
            break;
        }

        nav_steer_to_safe_distance(side);
        break;
    }

    case NavState::ReturnToLine: {
        if (in.ir_left || in.ir_mid || in.ir_right) {
            ESP_LOGI(TAG, "Line found on the return leg after %u ms (recorded %u ms)",
                     (unsigned)elapsed, (unsigned)s_out_leg_ms);
            nav_line_found();
            break;
        }

        if (nav_hold(s_blocked_hold, in.front_cm <= NAV_BLOCKED_CM, NAV_BLOCKED_CONFIRM_MS)) {
            nav_enter_stopped("path blocked while returning to the line", in);
            break;
        }

        uint32_t slow_after_ms = s_out_leg_ms * NAV_RETURN_SLOW_START_PCT / 100;
        uint32_t give_up_ms    = s_out_leg_ms + NAV_RETURN_EXTRA_MS;

        if (elapsed >= give_up_ms) {
            ESP_LOGW(TAG, "Line not at the recorded distance, searching");
            nav_start_search();
            break;
        }

        int speed = (elapsed < slow_after_ms) ? g_speed : g_slow;
        nav_drive(speed, speed);
        break;
    }

    case NavState::SearchForLine: {
        if (in.ir_left || in.ir_mid || in.ir_right) {
            ESP_LOGI(TAG, "Line found by the search (round %d)", s_search_round);
            nav_line_found();
            break;
        }

        if (elapsed >= NAV_SEARCH_TIMEOUT_MS) {
            nav_enter_stopped("line not found within the search time", in);
            break;
        }

        int      widen     = (s_search_round < NAV_SEARCH_ROUNDS) ? s_search_round : NAV_SEARCH_ROUNDS;
        uint32_t amplitude = (uint32_t)widen * NAV_SEARCH_STEP_MS;
        uint32_t phase_len = amplitude;
        if (s_search_phase == 1) phase_len = 2 * amplitude;
        if (s_search_phase == 3) phase_len = NAV_SEARCH_CREEP_MS;

        if (now - s_search_phase_ms >= phase_len) {
            s_search_phase_ms = now;
            s_search_phase++;

            if (s_search_phase > 3) {
                s_search_phase = 0;
                s_search_round++;
                ESP_LOGI(TAG, "Search round %d", s_search_round);
            }
        }

        if (s_search_phase == 3) {
            if (nav_hold(s_blocked_hold, in.front_cm <= NAV_BLOCKED_CM, NAV_BLOCKED_CONFIRM_MS)) {
                nav_enter_stopped("path blocked while searching for the line", in);
                break;
            }
            nav_drive(g_slow, g_slow);
        } else if (s_search_phase == 1) {
            nav_turn_dir(s_bypass_dir);
        } else {
            nav_turn_dir(nav_opposite(s_bypass_dir));
        }
        break;
    }
    case NavState::AlignWithLine:
        nav_turn_dir(s_bypass_dir);

        if (elapsed >= NAV_ALIGN_MIN_MS && in.ir_mid) {
            nav_stop();
            nav_change_state(s_resume_state);
        } else if (elapsed >= NAV_ALIGN_TIMEOUT_MS) {
            nav_stop();
            ESP_LOGW(TAG, "Align timed out, carrying on with the route");
            nav_change_state(s_resume_state);
        }
        break;

    case NavState::Arrived:
    case NavState::Stopped:
        nav_stop();
        break;
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

    g_speed = (SPEED_FORWARD > 255) ? 255 : SPEED_FORWARD;
    g_turn  = (SPEED_TURN    > 255) ? 255 : SPEED_TURN;
    g_slow  = g_speed * 70 / 100;
}

void control_task(void *pvParameters) {
    (void)pvParameters;

    ESP_LOGI(TAG, "Settling sensors for %dms before control starts...", STARTUP_SETTLE_MS);
    vTaskDelay(pdMS_TO_TICKS(STARTUP_SETTLE_MS));
    ESP_LOGI(TAG, "Control loop starting");

    while (1) {
        if (robot_state_get_unlock_active() || robot_state_is_powered_off()) {
            stop_and_flush();
            nav_reset();
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (robot_state_get_mode() == RobotMode::MANUAL) {
            nav_reset();
            run_manual();
        } else {
            NavInputs in;
            nav_read_inputs(in);
            nav_step(in);
        }

        vTaskDelay(pdMS_TO_TICKS(NAV_PERIOD_MS));
    }
}