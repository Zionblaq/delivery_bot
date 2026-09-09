#pragma once
#include <cstdint>

enum class MoveDir { FORWARD, BACKWARD, LEFT, RIGHT };

void movement_log_init();
void movement_log_record(MoveDir dir, uint32_t duration_ms);
void movement_log_reset();
void movement_log_retrace();