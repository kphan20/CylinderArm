#ifndef I2C_HEADER_H
#define I2C_HEADER_H

#include <stdbool.h>

extern volatile bool sensorRead;

void TWIInit();

void TWIStart();

void TWIStop();

#endif