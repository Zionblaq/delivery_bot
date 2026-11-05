#pragma once

<<<<<<< HEAD
// Configure motor direction pins and PWM channels
void movement_init(void);
=======
enum class MoveDir { FORWARD, BACKWARD, LEFT, RIGHT, PIVOT_LEFT, PIVOT_RIGHT };
>>>>>>> 99be0c6 (Hopefully Works)

void move_forward(void);
void move_forward_speed(int speed);
void move_backward(void);
void turn_left(void);
void turn_right(void);
void stop_car(void);