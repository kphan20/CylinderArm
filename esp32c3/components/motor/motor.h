#pragma once
#include "freertos/queue.h"
#include "pid.h"

typedef enum {
    UPPER_PRESSED = 0,
    LOWER_PRESSED,
    RELEASED,
} limit_notif_t;

void motor_gpio_setup();

void motor_task_setup();

void motor_set_command(PID_VAL_TYPE command);

void attach_limit_switch_listener(TaskHandle_t t);

void start_limit_protocol();