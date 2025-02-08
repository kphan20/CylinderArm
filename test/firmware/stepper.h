
#ifndef STEPPER_HEADER_H
#define STEPPER_HEADER_H

#include <stdbool.h>
#include "pid.h"

struct Stepper
{
    volatile int16_t currPos;
    volatile int16_t currCommand;
    volatile int16_t currSteps;
    volatile bool isGoingUp;
    volatile bool commandChanged;
    bool isClosedLoop;
    int16_t maxPos;
    int16_t minPos;
    volatile uint16_t currClock;
    volatile uint16_t nextClock;
    volatile int16_t errThreshold;
};

void stepperInit();

bool inRange(struct Stepper * s, int16_t newPos);

bool setCommand(struct Stepper * s, int16_t newCommand);

void calcNewFreq(struct Stepper * s);

// returns true if stepper motor should be stepped
bool stepperDt(struct Stepper * s, uint16_t dt);

#endif

