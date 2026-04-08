#include "queue.h"

static const struct Queue_t EmptyQueue;


HAL_StatusTypeDef Queue_Init(Queue_t* queue){
    if (queue == NULL) return HAL_ERROR;
    *queue = EmptyQueue;
    return HAL_OK;
}



uint8_t Queue_IsFull(Queue_t* queue) {
    return queue->count >= QUEUE_SIZE;
}

uint8_t Queue_IsEmpty(Queue_t* queue) {
    return queue->count == 0;
}

uint8_t Queue_Enqueue(Queue_t* queue, uint8_t* data, uint16_t len) {
    if (Queue_IsFull(queue) || len > MSG_MAX_SIZE) {
        DEBUG_PRINTF(DBG_ERROR, "[ESP] TX Queue full or msg too large\r\n");
        return 0;
    }
    
    QueueItem_t *item = &queue->items[queue->tail];
    memcpy(item->data, data, len);
    item->len = len;
    
    queue->tail = (queue->tail + 1) % QUEUE_SIZE;
    queue->count++;
    
    return 1;
}


QueueItem_t* Queue_Peek(Queue_t* queue) {
    if (Queue_IsEmpty(queue)) {
        return NULL;
    }
    return &queue->items[queue->head];
}

void Queue_Dequeue(Queue_t* queue) {
    if (!Queue_IsEmpty(queue)) {
        queue->head = (queue->head + 1) % QUEUE_SIZE;
        queue->count--;
    }
}