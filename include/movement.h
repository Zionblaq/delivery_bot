#pragma once

// Configure motor direction pins and PWM channels
void movement(void);

void move_forward(void);
void move_backward(void);
void turn_left(void);
void turn_right(void);
void stop_car(void);