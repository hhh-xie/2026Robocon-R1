#ifndef REMOTE_TEST_HAL_H
#define REMOTE_TEST_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct { uint16_t remaining; } DMA_HandleTypeDef;
typedef struct
{
    void *Instance;
    uint32_t ErrorCode;
    DMA_HandleTypeDef *hdmarx;
} UART_HandleTypeDef;
#define USART2 ((void *)2)
#define DMA1_Channel1_IRQn 1
#define USART2_IRQn 2
#define __HAL_UART_CLEAR_PEFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_FEFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_NEFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)(x))
#define __HAL_UART_CLEAR_IDLEFLAG(x) ((void)(x))
#define __HAL_DMA_GET_COUNTER(x) ((x)->remaining)
#define __DMB() ((void)0)
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *uart);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *uart,
                                            uint8_t *data, uint16_t count);
void HAL_NVIC_ClearPendingIRQ(int irq);
#endif
