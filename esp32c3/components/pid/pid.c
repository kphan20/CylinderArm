#include "pid.h"

const PID_VAL_TYPE P_GAIN;
const PID_VAL_TYPE I_GAIN;
const PID_VAL_TYPE D_GAIN;
PID_VAL_TYPE prev_error;
PID_VAL_TYPE error_sum;
PID_VAL_TYPE error_sum_min;
PID_VAL_TYPE error_sum_max;

// TODO test scaling factor or sliding window for error sum
// TODO dT variable?
PID_VAL_TYPE calc_pid(PID_VAL_TYPE setpoint, PID_VAL_TYPE sensor_val)
{
    PID_VAL_TYPE error = setpoint - sensor_val;
    PID_VAL_TYPE err_delta = error - prev_error; // TODO divide by dT
    error_sum += error; // TODO times dT

    // integral clamp
    if (error_sum > error_sum_max)
    {
        error_sum = error_sum_max;
    }
    else if (error_sum < error_sum_min)
    {
        error_sum = error_sum_min;
    }

    PID_VAL_TYPE output = P_GAIN * error + I_GAIN * error_sum + D_GAIN * err_delta;
    prev_error = error;

    // TODO command motor based on output
    return output;
}