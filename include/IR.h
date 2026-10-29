#pragma once

// Configure the three IR sensor pins as inputs
void lineSensor(void);

// Raw sensor readings (1 = HIGH, 0 = LOW)
int ir_read_right(void);
int ir_read_left(void);
int ir_read_mid(void);