#pragma once

struct LinePattern {
    bool left;
    bool center;
    bool right;
};

void LineSensor();
LinePattern read_line_pattern();