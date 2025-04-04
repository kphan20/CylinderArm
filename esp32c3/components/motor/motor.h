#pragma once
#include "freertos/queue.h"
#include "pid.h"

void motor_gpio_setup();

void motor_task_setup();

void motor_set_command(PID_VAL_TYPE command);