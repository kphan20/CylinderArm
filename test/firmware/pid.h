#ifndef PID_HEADER_H
#define PID_HEADER_H

#include <avr/io.h>
#include <stdbool.h>

struct PID
{
    int16_t kp; // proportional gain
    int16_t ki; // integral gain
    int16_t kd; // derivative gain
    int16_t prevErr; // previous error term
    int32_t errSum; // error sum
    int32_t prevOutput; // previous control output
    int32_t errSumMin; // clamps for integral sum
    int32_t errSumMax;
    uint16_t prevTime; // previous time PID was calculated
};

void pidReset(struct PID * p) {
    p->errSum = 0;
    p->prevErr = 0; // TODO think about reset logic
}

// TODO see if using a scaling factor or sliding window for the integral term will work
// TODO add dt term to account for variable time differences (based on clock cycles?)
// TODO add typedef for types
int32_t controlOutput(struct PID * p, int16_t err, int16_t errThreshold) {
    // setpoint has been reached
    if ((err < 0 ? -err : err) < errThreshold) {
        p->errSum = 0;
        p->prevErr = err;
        return 0; // TODO see if this is the correct thing to do when the setpoint is reached
    }
    uint16_t currTime = TCNT4; // TODO do I make this atomic
    uint16_t dt = currTime - p->prevTime;
    p->prevTime = currTime;

    // calculate derivative and integral terms
    int16_t errDelta = (err - p->prevErr) / dt; // TODO derivative kick?
    p->errSum += err * dt;

    // integral term clamping to help deal with windup
    if (p->errSum > p->errSumMax) {
        p->errSum = p->errSumMax;
    }
    else if (p->errSum < p->errSumMin)
    {
        p->errSum = p->errSumMin;
    }
    
    int32_t output = p->kp * err + p->ki * p->errSum + p->kd * errDelta;

    // update for next loop iteration
    p->prevErr = err;
    p->prevOutput = output;

    return output;
}

#endif

