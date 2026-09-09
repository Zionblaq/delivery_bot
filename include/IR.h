#pragma once

struct LinePattern {
    bool left;
    bool center;
    bool right;
};

void line_sensor_init();
LinePattern read_line_pattern();