#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdio.h>
#include "stepper.h"
#include "i2c.h"

// TODO find nice way to initialize PID struct
static struct Stepper s = {.maxPos=0, .minPos=0, .currClock=0, .currCommand=0, .currPos=0,.isClosedLoop=false, .nextClock=0};

void stepperInit() {
    TCCR0A = 2; // should set it to CTC mode
    TCCR0B |= 0x03; // set prescaler to 64
    OCR0A = 50; // 50 with 64 prescaler makes step generation at 5kHz, divide by 5 to get 1kHz PID loop
}

bool inRange(struct Stepper * s, int16_t newPos) {
    return !(s == NULL || (s->isClosedLoop && (newPos > s->maxPos || newPos < s->minPos)));
}

bool setCommand(struct Stepper * s, int16_t newCommand) {
    if (!inRange(s, newCommand)) return false;
    // TODO add port manipulation somewhere
    s->currCommand = newCommand; // TODO think about changes in setpoint
    s->commandChanged = true;
    return true;
}

void calcNewFreq(struct Stepper * s) {
    uint16_t desiredVel;
    uint16_t vMax; // mm/s, one rotation is 8mm, one rotation is 200 steps
    // 200 RPM -> 40000 steps/min -> 666 steps/sec -> 26 mm/s
    // 500 RPM -> 66 mm/s
    uint16_t estStepCount = (s->currPos >> 3) * 200 + (s->currPos & 0x7) * 25;
    // TODO tune error tolerance
    bool stepsMissed = (estStepCount > s->currSteps ? estStepCount - s->currSteps : s->currSteps - estStepCount) > 200;
    // s->nextClock is in effect current velocity
    s->currPos; // will be updated by sensor
    s->isGoingUp; // current direction of velocity
    // algo
    /// Find out if things are going according to plan
    if (s->commandChanged) {
        int16_t diff = s->currCommand - s->currPos;
        // diff is below some threshold, focus on getting to zero velocity
        if ((diff < 0 ? -diff : diff) < s->errThreshold) {
            desiredVel = 0;
        }
        // diff is too large 
        else {

        }
    }
    // TODO detect missed steps (don't know how to do this)
    else if (stepsMissed) {
        
    }
    // continue as normal
    else {

    }
    //// Cases where things aren't working out - new target, missed steps
    //// otherwise, continue with original profile plan
    s->nextClock = 0; // TODO figure out translation from control output to frequency
}

// returns true if stepper motor should be stepped
bool stepperDt(struct Stepper * s, uint16_t dt) {
    s->currClock += dt;
    // TODO figure out the sadness that is my distance sensors
    if (s->currClock > s->nextClock) {
        s->currClock = 0;
        int16_t err = s->currCommand - s->currPos;
        return (err < 0 ? -err : err) > s->errThreshold;
    }
    return false;
}