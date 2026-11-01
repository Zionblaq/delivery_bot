#pragma once

#include "driver/gpio.h"

typedef struct {
    int front;
    int left;
    int right;
} distances_t;


void ultrasonic_init(void);


int ultrasonic_measure_cm(gpio_num_t trig, gpio_num_t echo);

void ultrasonic_read_all(distances_t *out);