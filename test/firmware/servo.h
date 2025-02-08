
#ifndef SERVO_HEADER_H
#define SERVO_HEADER_H

#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>

struct Servo
{
    uint16_t currPos;
    uint16_t maxPos;
    uint16_t minPos;
};

void servoInit() {
    DDRB |= (1<<PB1); // set pin as output
    TCCR1A |= (1<<COM1A1) | (1<<WGM11) | (1<<WGM10); // non-inverting fast PWM
    TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler 8
    OCR1A = 39999; // 50 Hz
    // 39999 with prescaler 8 = 50hz
    // duty cycle register 1000 to 5000
    // 0.0675 degrees per register value
    OCR1B = 3000; // duty cycle (default neutral)
}

bool inRange(struct Servo * s, uint16_t newPos) {
    return !(s == NULL || newPos > s->maxPos || newPos < s->minPos);
}

bool setPos(struct Servo * s, uint16_t newPos) {
    if (!inRange(s, newPos)) return false;
    s->currPos = newPos;
    return true;
}

// stats
// 500-2500 microsec
/// prescaler of 64, TOP 4999
/// duty cycle register 125 to 625
// range: 270 degrees
/// .54 degrees per register duty cycle value
// dead band width 3 microsec
// operating frequency: 50-330Hz

#endif

