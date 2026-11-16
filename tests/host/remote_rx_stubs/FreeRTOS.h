#ifndef REMOTE_TEST_FREERTOS_H
#define REMOTE_TEST_FREERTOS_H
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;
#define pdFALSE 0
#define pdTRUE 1
#define pdFAIL 0
#define pdPASS 1
#define configASSERT(x) assert(x)
void Test_EnterCritical(void);
void Test_ExitCritical(void);
#define taskENTER_CRITICAL() Test_EnterCritical()
#define taskEXIT_CRITICAL() Test_ExitCritical()
#define portYIELD_FROM_ISR(x) ((void)(x))
#endif
