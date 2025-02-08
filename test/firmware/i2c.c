#include <avr/io.h>
#include <stdbool.h>
#include "i2c.h"

#define AS5600_I2C_ADDRESS 0x36
#define AS5600_RAW_REG 0x0C
#define AS5600_ANG_REG 0x0E

// steps for reading from AS5600
#define WRITE_REG 0
#define READ_REG_0 1
#define READ_REG_1 2

static volatile uint16_t angle = 0;
static volatile uint8_t AS5600I2CState = WRITE_REG;

// TODO don't need to rewrite angle, raw angle or magnitude address - these are special

static const uint8_t TWDR_ADDR[] = {AS5600_I2C_ADDRESS << 1, (AS5600_I2C_ADDRESS << 1) | 1};

volatile bool sensorRead = false;

// THERE ARE TWO TWI BUSES (for some reason this started as TWI1)

// first bus interrupt
ISR(TWI_vect) {

}

// second bus interrupt
ISR(TWI1_vect) {
    // mask the prescaler bits and find the status code
    switch(TWSR1 & 0xFC) {
        case 0x08: // start received by peripheral in controller receiver mode
            TWDR1 = TWDR_ADDR[AS5600I2CState];
            TWCR1 |= (1<<TWINT) | (1<<TWEN); // TODO do I need to explicitly set the other bits?
            break;
        case 0x10: // repeated start
            TWDR1 = TWDR_ADDR[AS5600I2CState];
            TWCR1 |= (1<<TWINT) | (1<<TWEN); // TODO do I need to explicitly set the other bits?
            break;
        case 0x18: // SLA+W received with ACK
            // write the register
            TWDR1 = AS5600_RAW_REG; // TODO figure out what to read
            TWCR1 |= (1<<TWINT) | (1<<TWEN);
            break;
        case 0x20: // SLA+W received with NACK
            // TODO repeat start?
            TWIStart();
            break;
        case 0x28: // data byte transmitted, ACK
            // move on to sensor read (TODO maybe restart instead?)
            AS5600I2CState = READ_REG_0;
            TWIStop();
            break;
        case 0x30: // data byte transmitted, NACK
            // TODO should I just restart?
            TWIStart();
            break;
        case 0x38: // arbitration lost in SLA+R or NACK bit
            TWIStart(); // resend the start signal
            break;
        case 0x40: // SLA+R message ack received
            // TWEA if we want to send ACK, TWSTA for repeat start, TWSTO for stop
            TWCR1 |= (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
            break;
        case 0x48: // SLA+R message nack received
            TWIStart(); // resend the start signal
            break;
        case 0x50: // data byte received with ack returned
            if (AS5600I2CState == READ_REG_0) {
                angle = 0;
                // TODO data validation?
                angle = (TWDR1 & 0xF) << 8; // move upper bits to proper location
                AS5600I2CState = READ_REG_1;
                TWCR1 |= (1<<TWINT) | (1<<TWEA) | (1<<TWEN);
            }
            else if (AS5600I2CState == READ_REG_1)
            {
                angle |= TWDR1;
                AS5600I2CState = READ_REG_0;
                sensorRead = true;
                TWCR1 |= (1<<TWINT) | (1<<TWEN); // NACK returned when both bytes read
            }
            else {
                // TODO wtf do I put here
            }
            break;
        case 0x58: // data byte received with nack returned
            TWIStop(); // TODO see if you want to differentiate between end of data and manual nack
        default:
            TWCR1 |= (1<<TWINT); // TODO this shouldn't happen
            break;
    }
}

void TWIInit() {
    // set clock rate (Max 1Mhz for AS5600)
    TWBR1 = 12; // with prescaler of 1 and 16MHZ cpu frequency, this should be 400khz SCL

    // enable TWI, interrupts, acks? (TODO)
    TWCR1 |= (1<<TWEN) | (1<<TWIE) | (1<<TWEA);
}

void TWIStart() {
    TWCR1 |= (1<<TWEN) | (1<<TWSTA) | (1<<TWINT);
}

void TWIStop() {
    // write stop condition
    TWCR1 |= (1<<TWEN) | (1<<TWSTO) | (1<<TWINT);
}
