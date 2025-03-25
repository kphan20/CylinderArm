#pragma once

#include "pid.h"

// setup GPIO for distance sensors
void sensor_gpio_setup();

// setup all tasks related to distance sensors
void sensor_task_setup();

PID_VAL_TYPE get_sensor_val();

void homing_sequence();
