#pragma once

void motor_control_init();
void motor_set_left_speed(int speed);
void motor_set_right_speed(int speed);
void motor_forward(int speed);
void motor_backward(int speed);
void motor_turn_left(int speed);
void motor_turn_right(int speed);
void motor_stop();