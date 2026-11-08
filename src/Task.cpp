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
#include <atomic>

static const char *TAG = "CONTROL";
#define NAV_SWAP_TURNS  1
#define NAV_SWAP_SIDES  0
#define NAV_SWAP_IR     0
#define NAV_SIDE_SENSOR_SCALE_PCT  100
#define NAV_SPEED_PCT  100
#define NAV_FOLLOW_SPEED_PCT  100
#define NAV_STEER_GENTLE_PCT  75
#define NAV_STEER_SHARP_PCT   40
#define NAV_DEBUG_LINE 1
#define NAV_OBSTACLE_AVOIDANCE 0

// ---- Tuning (all times in ms, distances in cm) ----
namespace {
constexpr uint32_t NAV_PERIOD_MS                 = 10;     // control loop period

// Junction and destination
constexpr uint32_t NAV_START_IGNORE_MS           = 500;    // ignore black bars for this long after a run starts
constexpr uint32_t NAV_JUNCTION_CONFIRM_MS       = 20;     // all three IR sensors black this long = junction / marker
constexpr uint32_t NAV_JUNCTION_ROLL_MS          = 300;    // roll onto the junction so the wheels are over it before turning
constexpr uint32_t NAV_JUNCTION_TURN_MIN_MS      = 200;    // turn at least this long (to leave the old line) ...
constexpr uint32_t NAV_JUNCTION_TURN_TIMEOUT_MS  = 1500;   // ... then until the middle sensor finds the branch, or give up
constexpr uint32_t NAV_JUNCTION_IGNORE_MS        = 1000;   // after the junction, ignore bars while the car clears it
constexpr uint32_t NAV_LINE_LOST_STOP_MS         = 200;    // all IR white this long (and no obstacle) = stop
constexpr int NAV_JUNCTION_TURN_PCT              = 70;
// Obstacle: stop and notify the web page
constexpr int      NAV_OBSTACLE_CM               = 25;     // front sensor stops the car at this distance
constexpr uint32_t NAV_OBSTACLE_CONFIRM_MS       = 120;    // ... if it stays this close for this long
constexpr uint32_t NAV_RESUME_GRACE_MS           = 700;    // after Play, ignore the front sensor this long (its reading may be old)

#if NAV_OBSTACLE_AVOIDANCE
// Obstacle detection and going round
constexpr int      NAV_SIDE_CLEARANCE_CM         = 25;     // a side counts as free to turn into above this
constexpr int      NAV_SAFE_CM                   = 20;     // distance kept from the obstacle while going round it
constexpr int      NAV_SAFE_TOLERANCE_CM         = 4;
constexpr int      NAV_LOST_CM                   = 45;     // wall-side reading above this = obstacle edge passed
constexpr uint32_t NAV_LOST_CONFIRM_MS           = 150;
constexpr int      NAV_CONTOUR_FRONT_CM          = 18;     // front at or below this while going round = inner corner
constexpr uint32_t NAV_INNER_CONFIRM_MS          = 80;
constexpr int      NAV_BLOCKED_CM                = 10;     // front below this while returning/searching = stop
constexpr uint32_t NAV_BLOCKED_CONFIRM_MS        = 100;
constexpr int      NAV_MAX_INNER_CORNERS         = 4;
constexpr uint32_t NAV_CORNER_CLEAR_MS           = 300;    // roll past an edge before turning around it
constexpr uint32_t NAV_SEEN_TIMEOUT_MS           = 1500;   // wall never seen on a face: assume it is passed after this long
constexpr uint32_t NAV_MAX_CONTOUR_MS            = 20000;
constexpr int      NAV_SIDE_RECHECKS             = 5;      // times to re-look when both sides look blocked
constexpr uint32_t NAV_SIDE_RECHECK_DELAY_MS     = 500;

// Turns (time for roughly 90 degrees)
constexpr uint32_t NAV_TURN_LEFT_MS              = 350;
constexpr uint32_t NAV_TURN_RIGHT_MS             = 350;

// Finding the line again after going round
constexpr int      NAV_RETURN_SLOW_START_PCT     = 80;     // slow down after this % of the recorded sideways time
constexpr uint32_t NAV_RETURN_EXTRA_MS           = 1500;   // extra search time beyond the recorded sideways time
constexpr uint32_t NAV_AXLE_TO_LINE_MS           = 300;    // roll on after the IR sees the line so the wheels are over it
constexpr uint32_t NAV_ALIGN_MIN_MS              = 150;
constexpr uint32_t NAV_ALIGN_TIMEOUT_MS          = 1200;
constexpr uint32_t NAV_SEARCH_STEP_MS            = 150;    // turn time of search round 1; round n turns n x this
constexpr uint32_t NAV_SEARCH_CREEP_MS           = 250;
constexpr int      NAV_SEARCH_ROUNDS             = 4;      // sweeps widen for this many rounds
constexpr uint32_t NAV_SEARCH_TIMEOUT_MS         = 8000;   // keep searching this long, then stop
constexpr float    NAV_APPROX_SPEED_CM_PER_S     = 25.0f;  // calibrate; used for log messages only
#endif  // NAV_OBSTACLE_AVOIDANCE

}  // namespace


static MoveDir s_current_dir = MoveDir::FORWARD;
static bool s_moving = false;
static int64_t s_segment_start_us = 0;

// Close the segment that was running and write it to the log
static void flush_segment() {
    if (s_moving) {
        int64_t now = esp_timer_get_time();
        uint32_t elapsed_ms = (uint32_t)((now - s_segment_start_us) / 1000);
        movement_log_record(s_current_dir, elapsed_ms);
        s_moving = false;
    }
}

// Start a new log segment when the type of movement changes
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
static int g_speed = 500;
static int g_turn  = 500;
static int g_slow  = 300;


static void nav_drive(int left_speed, int right_speed) {
#if NAV_SWAP_SIDES
    int t = left_speed; left_speed = right_speed; right_speed = t;
#endif
    begin_segment(MoveDir::FORWARD);
    motor_forward(left_speed > right_speed ? left_speed : right_speed);   // sets the direction of both wheels
    motor_set_left_speed(left_speed);                                     // then each wheel gets its own speed
    motor_set_right_speed(right_speed);
}

// Turn using the project's own turn movement, so the return-trip replay stays consistent
static void nav_turn(bool left, int speed) {
#if NAV_SWAP_TURNS
    left = !left;
#endif
    set_active_direction(left ? MoveDir::LEFT : MoveDir::RIGHT, speed);
}

static void nav_stop() {
    stop_and_flush();
}

struct NavInputs {
    int  front_cm;     
    int  left_cm;
    int  right_cm;
    bool ir_left;       
    bool ir_mid;
    bool ir_right;
    char destination; 
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
#if NAV_SWAP_IR
    in.ir_left  = p.right;
    in.ir_mid   = p.center;
    in.ir_right = p.left;
#else
    in.ir_left  = p.left;
    in.ir_mid   = p.center;
    in.ir_right = p.right;
#endif

    // The location picked on the web page ("Pick a Location"); 0 = nothing picked yet
    switch (robot_state_get_target_location()) {
        case DeliveryLocation::LOC_A: in.destination = 'A'; break;   // straight
        case DeliveryLocation::LOC_B: in.destination = 'B'; break;   // right
        case DeliveryLocation::LOC_C: in.destination = 'C'; break;   // left
        default:                      in.destination = 0;   break;
    }
}


enum class NavState {
    Idle,          
    FollowToJunction,     
    RollOntoJunction, 
    JunctionWait,       
    ObstaclePause,       
    UserPause,         
    TurnAtJunction,    
    FollowToDestination,  
#if NAV_OBSTACLE_AVOIDANCE
    CheckSides,           // bypass: pick the freer side
    TurnOut,              // bypass: turn toward it and start the path record
    Contour,              // bypass: go round the obstacle at a safe distance
    ReturnToLine,         // bypass: drive back the recorded sideways distance
    SearchForLine,        // bypass ONLY: line not where expected after going round an obstacle
    AlignWithLine,        // bypass: turn back to the original heading
#endif  // NAV_OBSTACLE_AVOIDANCE

    Arrived,              // destination reached: stopped until line mode is switched off and on again
    Stopped               // error stop: same
};

enum class NavDir { Left, Right };

struct Hold { bool on; uint32_t since; };   

static NavState    s_state             = NavState::Idle;
static uint32_t    s_state_start_ms    = 0;
static NavDir      s_junction_turn     = NavDir::Right;
static bool        s_left_old_line     = false;
static uint32_t    s_junction_ignore_until_ms = 0;
static char        s_route_dest        = 'A';     // location taken at the junction
static bool        s_wait_logged       = false;

static Hold        s_bar_hold, s_lost_line_hold, s_front_hold;
static NavState    s_resume_state      = NavState::FollowToJunction;   // where to go after a pause
static uint32_t    s_obstacle_ignore_until_ms = 0;

// Shared with the web server task
static std::atomic<int>  s_status_code{0};      // NavState, read by control_journey_status()
static std::atomic<bool> s_play_req{false};     // Play pressed on the web page
static std::atomic<bool> s_pause_req{false};    // Stop pressed on the web page

#if NAV_OBSTACLE_AVOIDANCE
static NavDir      s_bypass_dir        = NavDir::Right;
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

static Hold        s_wall_lost_hold, s_inner_hold, s_blocked_hold;
#endif  // NAV_OBSTACLE_AVOIDANCE

static uint32_t nav_millis() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static inline void nav_delay(uint32_t ms) {
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
    s_bar_hold.on = s_lost_line_hold.on = s_front_hold.on = false;
#if NAV_OBSTACLE_AVOIDANCE
    s_wall_lost_hold.on = s_inner_hold.on = s_blocked_hold.on = false;
#endif
}

static const char *nav_state_name(NavState s) {
    switch (s) {
        case NavState::Idle:                return "IDLE";
        case NavState::FollowToJunction:    return "FOLLOW_TO_JUNCTION";
        case NavState::RollOntoJunction:    return "ROLL_ONTO_JUNCTION";
        case NavState::JunctionWait:        return "JUNCTION_WAIT";
        case NavState::ObstaclePause:       return "OBSTACLE_PAUSE";
        case NavState::UserPause:           return "USER_PAUSE";
        case NavState::TurnAtJunction:      return "TURN_AT_JUNCTION";
        case NavState::FollowToDestination: return "FOLLOW_TO_DESTINATION";
#if NAV_OBSTACLE_AVOIDANCE
        case NavState::CheckSides:          return "CHECK_SIDES";
        case NavState::TurnOut:             return "TURN_OUT";
        case NavState::Contour:             return "CONTOUR";
        case NavState::ReturnToLine:        return "RETURN_TO_LINE";
        case NavState::SearchForLine:       return "SEARCH_FOR_LINE";
        case NavState::AlignWithLine:       return "ALIGN_WITH_LINE";
#endif  // NAV_OBSTACLE_AVOIDANCE

        case NavState::Arrived:             return "ARRIVED";
        default:                            return "STOPPED";
    }
}

static void nav_change_state(NavState s) {
    s_state = s;
    s_status_code.store((int)s);
    s_state_start_ms = nav_millis();
    nav_reset_holds();
    ESP_LOGI(TAG, "State -> %s", nav_state_name(s));
}

#if NAV_OBSTACLE_AVOIDANCE
static NavDir nav_opposite(NavDir d) {
    return (d == NavDir::Left) ? NavDir::Right : NavDir::Left;
}

#endif  // NAV_OBSTACLE_AVOIDANCE

#if NAV_OBSTACLE_AVOIDANCE
static void nav_turn_dir(NavDir d) {
    nav_turn(d == NavDir::Left, g_turn);
}
#endif

// Error stop: say why. Stays stopped until line mode is switched off and on again.
static void nav_enter_stopped(const char *reason, const NavInputs &in) {
    nav_stop();
    ESP_LOGE(TAG, "STOPPED: %s (front %d, left %d, right %d cm)", reason, in.front_cm, in.left_cm, in.right_cm);
    robot_state_set_target_location(DeliveryLocation::NONE);   // the next run needs a new pick
    nav_change_state(NavState::Stopped);
}

static void nav_arrived(char destination) {
    nav_stop();
    ESP_LOGI(TAG, "Arrived at location %c", destination);
    robot_state_set_target_location(DeliveryLocation::NONE);   // the next run needs a new pick
    nav_change_state(NavState::Arrived);
}


#if NAV_DEBUG_LINE
static uint32_t s_dbg_ms = 0;
#endif

static void nav_follow_line(const NavInputs &in) {
    bool l = in.ir_left, m = in.ir_mid, r = in.ir_right;

    int fast  = g_speed * NAV_FOLLOW_SPEED_PCT / 100;
    int sharp = fast * NAV_STEER_SHARP_PCT / 100;      // inner wheel, big correction
    const char *act;

    if (l && m && r) {
        act = "ALL BLACK, STRAIGHT";                    // wide line or a bar: keep going straight
        nav_drive(fast, fast);
    } else if (m && !l && !r) {
        act = "FORWARD";
        nav_drive(fast, fast);
    // } else if (l && m) {
    //     act = "STEER LEFT (small)";
    //     nav_drive(gentle, fast);
    // } else if (m && r) {
    //     act = "STEER RIGHT (small)";
    //     nav_drive(fast, gentle);
    } else if (l) {
        act = "STEER LEFT (big)";
        nav_drive(sharp, fast);
    } else if (r) {
        act = "STEER RIGHT (big)";
        nav_drive(fast, sharp);
    } else {
        act = "NO LINE (holding, will stop)";
    }

#if NAV_DEBUG_LINE
    uint32_t now = nav_millis();
    if (now - s_dbg_ms >= 300) {
        s_dbg_ms = now;
        ESP_LOGI(TAG, "IR L%d M%d R%d -> %s", (int)l, (int)m, (int)r, act);
    }
#else
    (void)act;
#endif
}

// ---- Junction ----
static void nav_take_junction(char dest) {
    s_route_dest = dest;
    s_left_old_line = false;
    ESP_LOGI(TAG, "Junction: location %c -> %s", dest,
             dest == 'B' ? "turn right" : (dest == 'C' ? "turn left" : "straight on"));
    nav_change_state(NavState::RollOntoJunction);   // roll on until the wheels are over the junction
}

static bool nav_obstacle_stop_interrupt(const NavInputs &in) {
    if (nav_millis() < s_obstacle_ignore_until_ms) {      // just resumed: the reading may be from before
        s_front_hold.on = false;
        return false;
    }
    if (nav_hold(s_front_hold, in.front_cm <= NAV_OBSTACLE_CM, NAV_OBSTACLE_CONFIRM_MS)) {
        nav_stop();
        s_resume_state = s_state;
        s_play_req.store(false);
        ESP_LOGW(TAG, "OBSTACLE at %d cm: stopped, press Play on the web page to continue", in.front_cm);
        nav_change_state(NavState::ObstaclePause);
        return true;
    }
    return false;
}

#if NAV_OBSTACLE_AVOIDANCE
static bool nav_obstacle_interrupt(const NavInputs &in) {
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
    return ((s_heading_quarters % 4) + 4) % 4;   // 0 = original heading, 1 = toward the bypass side, 3 = back toward the line
}

// Add time driven to the axis the car is currently heading along
static void nav_odo_add(uint32_t ms) {
    int32_t t = (int32_t)ms;

    switch (nav_heading_index()) {
        case 0:  s_forward_ms += t; break;
        case 1:  s_lateral_ms += t; break;
        case 2:  s_forward_ms -= t; break;
        default: s_lateral_ms -= t; break;
    }
}

// Turn ~90 degrees (blocking is fine here: the sensors are read by their own tasks)
static void nav_pivot(NavDir d) {
    nav_turn_dir(d);
    nav_delay(d == NavDir::Left ? NAV_TURN_LEFT_MS : NAV_TURN_RIGHT_MS);
    nav_stop();

    s_heading_quarters += (d == s_bypass_dir) ? 1 : -1;
    ESP_LOGI(TAG, "Turn %s, heading index %d", d == NavDir::Left ? "left" : "right", nav_heading_index());
}

// Start looking at a new face of the obstacle
static void nav_reset_face() {
    s_side_seen = false;
    s_face_start_ms = nav_millis();
    s_last_contour_ms = nav_millis();
    s_wall_lost_hold.on = false;
    s_inner_hold.on = false;
}

// Keep the obstacle about NAV_SAFE_CM away using gentle differential steering
static void nav_steer_to_safe_distance(int side_cm) {
    bool wall_on_left = (s_bypass_dir == NavDir::Right);
    int  error        = side_cm - NAV_SAFE_CM;   // + = too far from the obstacle
    int  left_speed   = g_speed;
    int  right_speed  = g_speed;

    if (error > NAV_SAFE_TOLERANCE_CM) {            // too far: curve toward the obstacle
        if (wall_on_left) left_speed  = g_slow;
        else              right_spLISTENING TO THIS WHILE SLEEPING...PURE WORSHIP// MIN THEOPHILUS SUNDAYeed = g_slow;
    } else if (error < -NAV_SAFE_TOLERANCE_CM) {    // too close: curve away from it
        if (wall_on_left) right_speed = g_slow;
        else              left_speed  = g_slow;
    }

    nav_drive(left_speed, right_speed);
}

// Pick the clearer side. If both look blocked, look again a few times before giving up.
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
            nav_delay(NAV_SIDE_RECHECK_DELAY_MS);   // stay in CheckSides; the next tick has fresh readings
        }
        return;
    }

    ESP_LOGI(TAG, "Bypassing to the %s", s_bypass_dir == NavDir::Left ? "left" : "right");
    nav_change_state(NavState::TurnOut);
}

// An IR sensor just saw the line again: roll on until the wheels are over it, then align
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

#endif  // NAV_OBSTACLE_AVOIDANCE

// Drop any run in progress without touching the motors (manual mode, power off, unlock)
static void nav_reset() {
    s_wait_logged = false;
    s_play_req.store(false);
    s_pause_req.store(false);
    if (s_state != NavState::Idle) {
        ESP_LOGI(TAG, "Run dropped (manual mode, power off or unlock)");
        nav_change_state(NavState::Idle);
    }
}

static void nav_step(const NavInputs &in) {
    uint32_t now     = nav_millis();
    uint32_t elapsed = now - s_state_start_ms;

    // ---- Stop pressed on the web page ----
    bool pause_req      = s_pause_req.exchange(false);
    bool along_the_line = (s_state == NavState::FollowToJunction ||
                           s_state == NavState::RollOntoJunction ||
                           s_state == NavState::FollowToDestination);

    if (pause_req && (along_the_line || s_state == NavState::TurnAtJunction)) {
        nav_stop();
        s_resume_state = s_state;
        s_play_req.store(false);
        ESP_LOGI(TAG, "Stop pressed: journey paused");
        nav_change_state(NavState::UserPause);
        return;
    }

    // ---- OBSTACLE INTERRUPT: has priority over line following, the junction and the end-of-line stop ----
    if (along_the_line) {
#if NAV_OBSTACLE_AVOIDANCE
        if (nav_obstacle_interrupt(in)) {      // go-around version (switched off)
            return;
        }
#else
        if (nav_obstacle_stop_interrupt(in)) { // stop and notify the web page
            return;
        }
#endif
    }

    switch (s_state) {

    // 1. Line mode is on: wait for a location to be picked on the web page, then start the run.
    //    The movement log restarts here so the return trip after the delivery leads back to where this run began.
    case NavState::Idle:
        if (in.destination == 0) {
            nav_stop();
            if (!s_wait_logged) {
                ESP_LOGI(TAG, "Waiting for a location: press \"Pick a Location\" on the web page");
                s_wait_logged = true;
            }
            break;
        }
        s_wait_logged = false;
        ESP_LOGI(TAG, "Location %c picked, run started: A = straight, B = right, C = left at the junction", in.destination);
        movement_log_reset();
        s_junction_ignore_until_ms = now + NAV_START_IGNORE_MS;
        nav_change_state(NavState::FollowToJunction);
        break;

    // 2/3. Follow the line. Before the junction a black bar is the junction; after it, it is the destination.
    case NavState::FollowToJunction:
    case NavState::FollowToDestination: {
        bool after_junction = (s_state == NavState::FollowToDestination);
        bool ignoring       = (now < s_junction_ignore_until_ms);
        bool all_black      = in.ir_left && in.ir_mid && in.ir_right;
        bool all_white      = !in.ir_left && !in.ir_mid && !in.ir_right;

        if (!ignoring && nav_hold(s_bar_hold, all_black, NAV_JUNCTION_CONFIRM_MS)) {
            if (after_junction) {
                nav_arrived(s_route_dest);            // destination marker
            } else if (in.destination != 0) {
                nav_take_junction(in.destination);    // the location picked right now
            } else {
                nav_stop();
                ESP_LOGI(TAG, "Junction reached, no location picked: waiting");
                nav_change_state(NavState::JunctionWait);
            }
            break;
        }

        if (nav_hold(s_lost_line_hold, all_white, NAV_LINE_LOST_STOP_MS)) {
            if (after_junction) {
                ESP_LOGI(TAG, "End of line");
                nav_arrived(s_route_dest);
            } else {
                nav_enter_stopped("no line before the junction", in);
            }
            break;
        }

        nav_follow_line(in);
        break;
    }

    // At the junction with no location picked: wait for a pick
    case NavState::JunctionWait:
        nav_stop();
        if (in.destination != 0) {
            nav_take_junction(in.destination);
        }
        break;

    // Roll on until the wheels are over the junction, then go straight on or turn onto the branch.
   case NavState::RollOntoJunction:
        nav_drive(g_slow, g_slow);
 
        if (elapsed >= NAV_JUNCTION_ROLL_MS) {
            nav_stop();
 
            if (s_route_dest == 'B' || s_route_dest == 'C') {
                s_junction_turn = (s_route_dest == 'B') ? NavDir::Right : NavDir::Left;
                s_left_old_line = false;
                ESP_LOGI(TAG, "Turning %s onto the branch", s_junction_turn == NavDir::Right ? "right" : "left");
                nav_change_state(NavState::TurnAtJunction);
            } else {
                s_junction_ignore_until_ms = now + NAV_JUNCTION_IGNORE_MS;
                nav_change_state(NavState::FollowToDestination);
            }
        }
        break;

    // Turn onto the branch: leave the old line first, then stop when the middle sensor finds the branch
     case NavState::TurnAtJunction:
        nav_turn(s_junction_turn == NavDir::Left, g_turn * NAV_JUNCTION_TURN_PCT / 100);
 
        if (!in.ir_mid && !s_left_old_line) {
            s_left_old_line = true;
            ESP_LOGI(TAG, "Turn: the middle sensor has left the old line");
        }
 
        if ((s_left_old_line && in.ir_mid && elapsed >= NAV_JUNCTION_TURN_MIN_MS) ||
            elapsed >= NAV_JUNCTION_TURN_TIMEOUT_MS) {
            if (!(s_left_old_line && in.ir_mid)) {
                ESP_LOGW(TAG, "Branch not found while turning (left old line: %d), carrying on", (int)s_left_old_line);
            } else {
                ESP_LOGI(TAG, "Branch found, following it");
            }
            nav_stop();
            s_junction_ignore_until_ms = now + NAV_JUNCTION_IGNORE_MS;
            nav_change_state(NavState::FollowToDestination);
        }
        break;
#if NAV_OBSTACLE_AVOIDANCE
    // 4. Bypass: look left and right, choose the freer side
    case NavState::CheckSides:
        nav_stop();
        nav_choose_bypass(in);
        break;

    // 5. Bypass: turn toward the free side and start a fresh path record
    case NavState::TurnOut:
        s_lateral_ms = 0;
        s_forward_ms = 0;
        s_heading_quarters = 0;
        s_inner_corners = 0;

        nav_pivot(s_bypass_dir);   // heading index becomes 1: now driving away from the line
        nav_reset_face();
        nav_change_state(NavState::Contour);
        break;

    // 6. Bypass: go round the obstacle, keeping NAV_SAFE_CM from it.
    //    Every tick the time driven is added to the path record.
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

        // Inner corner: something ahead on this face. Turn away from the obstacle and carry on.
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

        // Outer corner: the obstacle's edge has passed the wall-side sensor.
        // If this face was never seen (thin obstacle), assume it is passed after NAV_SEEN_TIMEOUT_MS.
        bool edge_passed = (s_side_seen && wall_gone) ||
                           (!s_side_seen && (now - s_face_start_ms) > NAV_SEEN_TIMEOUT_MS);

        if (edge_passed) {
            ESP_LOGI(TAG, "Edge passed, rolling on then turning around it");
            nav_drive(g_speed, g_speed);
            nav_delay(NAV_CORNER_CLEAR_MS);          // let the whole car clear the edge
            nav_odo_add(NAV_CORNER_CLEAR_MS);
            nav_pivot(nav_opposite(s_bypass_dir));   // around the corner, toward the obstacle's next face
            nav_reset_face();

            if (nav_heading_index() == 3) {
                // Facing back toward the line: the obstacle is passed. Use the record to retrace.
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

    // 7. Bypass: retrace the recorded sideways distance until an IR sensor sees the line.
    //    Fast for most of the distance, slow near where the line should be.
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

    // 8. Bypass: line not where expected. Keep searching for NAV_SEARCH_TIMEOUT_MS, then stop.
    //    Turn one way, turn back past centre, re-centre, creep forward; the sweeps widen each round.
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

    // 9. Bypass: turn back to the original heading, then carry on with the route
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

#endif  // NAV_OBSTACLE_AVOIDANCE

    // Paused by the obstacle interrupt or by the Stop button: wait for Play on the web page
    case NavState::ObstaclePause:
    case NavState::UserPause:
        nav_stop();
        if (s_play_req.exchange(false)) {
            ESP_LOGI(TAG, "Play pressed: continuing the journey");
            s_obstacle_ignore_until_ms = now + NAV_RESUME_GRACE_MS;
            nav_change_state(s_resume_state);
        }
        break;

    // 10. Finished or stopped: stay put until line mode is switched off and on again
    case NavState::Arrived:
    case NavState::Stopped:
        nav_stop();
        break;
    }
}

// ===========================================================================
// Used by web_server.cpp (declared there, so no header has to change)
// ===========================================================================
// One word describing the journey, polled by the web page
const char *control_journey_status() {
    switch ((NavState)s_status_code.load()) {
        case NavState::Idle:          return "idle";       // line mode on, waiting for a location
        case NavState::JunctionWait:  return "waiting";    // at the junction, waiting for a location
        case NavState::ObstaclePause: return "obstacle";
        case NavState::UserPause:     return "paused";
        case NavState::Arrived:       return "arrived";
        case NavState::Stopped:       return "stopped";
        default:                      return "running";
    }
}

void control_request_play()  { s_play_req.store(true); }
void control_request_pause() { s_pause_req.store(true); }

// ===========================================================================
// Manual mode (unchanged)
// ===========================================================================
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

    g_speed = ((SPEED_FORWARD > 255) ? 255 : SPEED_FORWARD) * NAV_SPEED_PCT / 100;
    g_turn  = ((SPEED_TURN    > 255) ? 255 : SPEED_TURN)    * NAV_SPEED_PCT / 100;
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