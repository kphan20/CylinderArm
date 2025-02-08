#ifndef APP_HEADER_H
#define APP_HEADER_H

#include "stepper.h"
#include "servo.h"
#include "h_bridge.h"
#include "pid.h"
#include "task_scheduler.h"

static struct PID pid;
static Motor baseMotor;
static Stepper elevatorStepper;
static Stepper carriageStepper;
static Stepper wristStepper;

void baseMotorPID() {
    controlOutput(&pid, );
}
void elevatorPID() {

}
void carriagePID() {

}
void wristPID() {

}

Task tasks[4] = {baseMotorPID, elevatorPID, carriagePID, wristPID};
Stepper steppers[3] = {elevatorStepper, carriageStepper, wristStepper};

#endif