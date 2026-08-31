#pragma once

struct LinePattern {
    bool left;
    bool center;
    bool right;
};

void line_sensor_init();
void line_sensor_task(void *pvParameters);

// Reads the raw values currently in shared state and returns which
// sensors currently see black.
LinePattern read_line_pattern();