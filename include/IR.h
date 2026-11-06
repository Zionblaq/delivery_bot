#pragma once

struct LinePattern {
    bool left;
    bool center;
    bool right;
};

void line_sensor_init();
void line_sensor_task(void *pvParameters);

LinePattern read_line_pattern();
