#ifndef REMOTE_TEST_QUEUE_H
#define REMOTE_TEST_QUEUE_H
#include "FreeRTOS.h"
typedef struct TestQueue *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t count, UBaseType_t size);
BaseType_t xQueuePeek(QueueHandle_t queue, void *value, TickType_t wait);
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *value);
#endif
