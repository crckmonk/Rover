#ifndef SCHEDULER_H
#define SCHEDULER_H
#include "main.h"

typedef void (*TaskFunc_t)(void);

typedef struct {
    TaskFunc_t func;
    uint32_t interval_ms;
    uint32_t last_run;
    uint8_t enabled;
} Task_t;

#define MAX_TASKS 8

void Scheduler_Init(void);
int8_t Scheduler_AddTask(TaskFunc_t func, uint32_t interval_ms);
void Scheduler_Run(void);

#endif