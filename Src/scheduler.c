#include "scheduler.h"


static Task_t tasks[MAX_TASKS];
static uint8_t task_count =0;


void Scheduler_Init(void) {
    task_count = 0;
    for (int i = 0; i < MAX_TASKS; i++) {
        tasks[i].func = NULL;
        tasks[i].enabled = 0;
    }
}


int8_t Scheduler_AddTask(TaskFunc_t func, uint32_t interval_ms) {
    if (task_count >= MAX_TASKS) return -1;
    
    tasks[task_count].func = func;
    tasks[task_count].interval_ms = interval_ms;
    tasks[task_count].last_run = 0;
    tasks[task_count].enabled = 1;
    
    return task_count++;
}


void Scheduler_Run(void) {
    uint32_t now = HAL_GetTick();
    
    for (uint8_t i = 0; i < task_count; i++) {
        if (tasks[i].enabled && tasks[i].func != NULL) {
            if ((now - tasks[i].last_run) >= tasks[i].interval_ms) {
                tasks[i].last_run = now;
                tasks[i].func();
            }
        }
    }
}