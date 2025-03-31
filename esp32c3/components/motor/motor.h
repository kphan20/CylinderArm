#pragma once
#include "freertos/queue.h"
#include "pid.h"

typedef int32_t motor_cmd_t;

void motor_gpio_setup();

void motor_task_setup();

void motor_set_command(PID_VAL_TYPE command);