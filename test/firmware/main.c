#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <stdio.h>

#include "limit_switch.h"
#include "watchdog.h"
#include "i2c.h"
#include "distance.h"
#include "spi.h"
#include "servo.h"
#include "stepper.h"
#include "task_scheduler.h"
#include "application.h"

// TODO disable pullups on relevant pins (PORTx[PIN] = 0)

static volatile uint8_t currPIDCounter = 0;
static const uint8_t maxPIDCounter = 5;

static volatile uint16_t prevTimer = 0;

// interrupt for checking stepper motors
// TODO workout architecture (should I do calculations here? or set a flag to execute code in the main loop)
ISR(TIMER0_COMPA_vect) {
    // TODO see if modulo is too slow
    if (currPIDCounter==maxPIDCounter) {
        currPIDCounter = 0;
        ICPStart(); // TODO this needs to happen at slower frequency
        TWIStart(); // read i2c sensor
    }
    else {
        currPIDCounter += 1;
    }
    uint16_t curr = TCNT4;

    // this accounts for overflows - TODO should I consider multiple overflows?
    uint16_t dt = curr - prevTimer;

    for (size_t i; i < sizeof(steppers) / sizeof(steppers[0]); i++) {
        if (stepperDt(&steppers[i], dt)) {
        // TODO add stepping logic
        }
    }
}

int main() {
    cli();

    // power register
    // power off USART1, USART0, ADC
    PRR0 |= (1<<PRUSART1) | (1<<PRUSART0) | (1<<PRADC);
    // TODO why don't they list the PRR1 bits jesus

    // TODO clear all interrupt flags
    watchdogInit();
    switchInit();
    TWIInit();
    ICPInit();
    SPIInit();
    servoInit();

    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();

    sei(); // enable interrupts
    while(1) {
        if (switchInterruptFlag) {
            clearTasks(); // TODO get rid of all tasks in queue
            readShiftRegister();
            // TODO this will block until the problem is resolved
            switchReset(); // TODO see if this is the desired behavior (should I interrupt all behavior until the switch isn't pressed?)
        }
        else {
            Task task = dequeueTask();
            if (task != NULL) {
                (*task)(); // execute tasks
            }
        }

        if (sensorRead) { // TODO change this, steppers won't really rely on encoder (maybe wrist stepper?)
            sensorRead = false;
            calcNewFreq(&s);
        }

        if (taskQEmpty) {
            cli();
            sei();
            sleep_cpu(); // TODO no interrupts?
        }
    }
    return 0;
}