#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

#include "limit_switch.h"

// TODO may require debouncing

volatile bool switchInterruptFlag = false;
uint8_t limitSwitchState = 0;
const uint8_t READ_DELAY = 5;
const uint8_t NUM_SWITCHES = 8;

// currently interrupt on PE3
ISR(INT0_vect) {
    // TODO are there issues disabling it right away
    EIMSK &= ~(1<<INT0); // disable limit switch interrupt
    switchInterruptFlag = true; // set flag indicating switch state changed
    if ((EICRA & 0x3) == 0x3) { // rising edge case
        // TODO end all other interrupts, watchdog
        // probably need to create a bunch of enables
        // TODO set heartbeat message to SWITCH_HIT message
        
    }
    else { // falling edge case

    }
}

void switchInit() {
    // set PE3 as input
    DDRD &= ~(1<<LIMIT_SWITCH_INTERRUPT);
    // set shift register input
    DDRC &= ~(1<<SR_INPUT);
    // set shift register clock as output
    DDRC |= 1<<SR_CLK;

    // enable external interrupt
    EICRA |= (1<<ISC01)|(1<<ISC00); // rising edge interrupt
    EIMSK |= (1<<INT0);
}

void readShiftRegister() {
    limitSwitchState = 0;
    _delay_us(READ_DELAY); // allow for latch to settle
    for (uint8_t i = 0; i < NUM_SWITCHES; i++) {
        PORTC |= (1<<SR_CLK); // rising clock edge
        _delay_us(READ_DELAY);
        limitSwitchState = (limitSwitchState << 1) | ((PORTC & (1<<SR_INPUT)) >> SR_INPUT);
        PORTC &= ~(1<<SR_CLK); // falling edge
        _delay_us(READ_DELAY);
    }

    switchInterruptFlag = false; // stop reading shift register
    // enable falling edge detection
    EIFR |= (1<<INTF0); // clear interrupt flag first
    EICRA = (EICRA & 0xFC) | 0x2; // falling edge interrupt
    EIMSK |= (1<<INT0); // enable interrupt
}

void switchReset() {
    _delay_ms(READ_DELAY);
    // wait for switches to stop contacting
    while (!switchInterruptFlag) {

    }

    switchInterruptFlag = false; // wait for next rising edge to trigger
    EIFR |= (1<<INTF0); // clear interrupt flag first
    EICRA = (EICRA & 0xFC) | 0x3; // rising edge interrupt
    _delay_us(READ_DELAY);
    EIMSK |= (1<<INT0); // enable interrupt
}