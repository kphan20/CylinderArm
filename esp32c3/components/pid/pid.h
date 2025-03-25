#pragma once

typedef float PID_VAL_TYPE;

PID_VAL_TYPE calc_pid(PID_VAL_TYPE setpoint, PID_VAL_TYPE sensor_val);