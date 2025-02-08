#ifndef TASK_HEADER_H
#define TASK_HEADER_H

#include <avr/io.h>
#include <stdbool.h>

inline const uint8_t TASK_Q_SIZE = 32; // will be power of two for efficiency

typedef void (*Task)(void);

volatile extern bool taskQEmpty;

void enqueueTask(Task task);

Task dequeueTask();

void clearTasks();

#endif