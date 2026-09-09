#pragma once
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

enum class RobotMode { MANUAL, LINE_FOLLOW };
enum class ManualCmd { NONE, FORWARD, BACKWARD, LEFT, RIGHT, STOP };
enum class DeliveryLocation { NONE, LOC_A, LOC_B, LOC_C };

void robot_state_init();

void robot_state_set_mode(RobotMode m);
RobotMode robot_state_get_mode();

void robot_state_set_manual_cmd(ManualCmd c);
ManualCmd robot_state_get_manual_cmd();
bool robot_state_manual_cmd_is_stale(uint32_t max_age_ms);

void robot_state_set_distance(int index, float cm);
void robot_state_get_distances(float out[4]);

void robot_state_set_line_raw(int index, int raw);
void robot_state_get_line_raw(int out[3]);

std::string robot_state_new_order();
bool robot_state_check_pin(const std::string &pin);

void robot_state_set_unlock_active(bool active);
bool robot_state_get_unlock_active();

void robot_state_set_powered_off(bool off);
bool robot_state_is_powered_off();

void robot_state_set_target_location(DeliveryLocation loc);
DeliveryLocation robot_state_get_target_location();

// REMOVED: robot_state_request_priority_override() / consume_priority_override()
// control_task now checks mode/power directly instead of a separate flag.