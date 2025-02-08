#ifndef LIMIT_SWITCH_HEADER_H
#define LIMIT_SWITCH_HEADER_H

#define SR_INPUT PC0
#define SR_CLK PC1
#define LIMIT_SWITCH_INTERRUPT PD2

extern volatile bool switchInterruptFlag;

void switchInit();

void readShiftRegister();

void switchReset();

#endif