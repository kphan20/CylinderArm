#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/atomic.h>

#include "task_scheduler.h"

static volatile Task buffer[TASK_Q_SIZE];
static volatile uint8_t writeHead = 0, readHead = 0;

volatile bool taskQEmpty = true;

void enqueueTask(Task task) {
    // TODO for now, overwrite oldest task (add watchdog log later)
    if (writeHead == readHead) {
        readHead = (readHead + 1) & (TASK_Q_SIZE - 1);
    }
    buffer[writeHead] = task;
    writeHead = (writeHead + 1) & (TASK_Q_SIZE - 1);
    taskQEmpty = false; 
}

Task dequeueTask() {
    if (writeHead == readHead) {
        taskQEmpty = true;
        return NULL;
    }
    Task task;
    // TODO I think this should be atomic
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        Task task = buffer[readHead];
        readHead = (readHead + 1) & (TASK_Q_SIZE - 1);
    }
    return task;
}

void clearTasks() {
    writeHead = readHead;
    taskQEmpty = true;
}