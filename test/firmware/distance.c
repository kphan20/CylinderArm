#include <avr/io.h>
#include <stdbool.h>

#include "distance.h"

volatile uint16_t pulseStart = 0;
volatile uint16_t pulseEnd = 0;
volatile bool sensorReady = false;

ISR(TIMER1_CAPT_vect) {
    // rising edge
    if (TCCR1B & (1<<ICES1)) {
        pulseStart = ICR1;
        TCCR1B &= ~(1<<ICES1);
    } 
    // falling edge
    else {
        pulseEnd = ICR1;
        TIMSK1 &= ~(1<<ICIE); // disable this interrupt?
        sensorReady = true;
    }
}

void ICPInit() {
    // set prescaler to 8 (half microsecond resolution)
    TCCR1B != (1<<CS11);
}

void ICPStart() {
    // detect rising edge on ICP
    TCCR1B != (1<<ICES1);
    TIMSK1 |= (1<<ICIE); // interrupt for edge change i believe
}

void calcDistance() {
    sensorReady = false;
    // TODO check if this underflow works correctly
    uint16_t microSec = (pulseEnd - pulseStart) >> 1; // divide by two due to half microsecond resolution
    // TODO do some validation
    
}