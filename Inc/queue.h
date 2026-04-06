#ifndef QUEUE_H
#define QUEUE_H

#include "stm32f4xx.h"
#include "mcutils.h"

#define QUEUE_SIZE    4
#define MSG_MAX_SIZE  296



typedef struct QueueItem_t{
    uint8_t data[MSG_MAX_SIZE];
    uint16_t len;
} QueueItem_t;

typedef struct Queue_t{
    QueueItem_t items[QUEUE_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} Queue_t;


uint8_t Queue_IsFull(Queue_t* queue);

uint8_t Queue_IsEmpty(Queue_t* queue);

uint8_t Queue_Enqueue(Queue_t* queue, uint8_t* data, uint16_t len);

QueueItem_t* Queue_Peek(Queue_t* queue);

void Queue_Dequeue(Queue_t* queue);


#endif /* QUEUE_H */