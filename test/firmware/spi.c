#include <avr/io.h>

#include "spi.h"

// by default set to heartbeat response
volatile uint8_t nextSPIByte = 0x01;
volatile uint8_t lastSPIRead = 0;

ISR(SPI0_STC_vect) {
    lastSPIRead = SPDR0;
    SPDR0 = nextSPIByte; // prepare for next transmission
    nextSPIByte =  0x01; // reset back to heartbeat response
}

void SPIInit() {
    // set output
    DDRB |= (1<<PB4);
    // TODO DORD0 (data order)
    SPCR0 |= (1<<SPE0) | (1<<SPIE0);
}