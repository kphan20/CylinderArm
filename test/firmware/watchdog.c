#include "watchdog.h"

ISR(WDT_vect) {
    // Check all flags and write SPI info
    // 8 bits
    //// PID
    //// ACTUATOR
    //// I2C
    //// PWM
    //// LIMIT SWITCH
    //// TASK QUEUE OVERRUN?
    
}

void watchdogInit() {
    MCUSR &= ~(1<<WDRF);
    // set WDCE to 1 in order to change WDE and prescaler
    // -U lfuse:w::m
    WDTCSR |= (1<<WDIE); // Do I need to reset WDIE every time?
    WDTCSR |= (1<<WDP2) | (1<<WDP0); // 0.5s timeout
}