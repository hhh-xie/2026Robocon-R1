#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__ 

#include "stm32g4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

typedef struct
{
    int16_t ch[32];//这里根据具体的遥控器通道定义来设置有多少个通道
    uint8_t code12[12];
    uint8_t code12_update_flag;
    uint32_t code12_update_count;
    uint32_t frame_seq;
    TickType_t last_valid_tick;
}ELRS_t;


/* RC_Init/RC_Receive 仅由 RemoteTask 调用；快照接口仅用于任务上下文。 */
BaseType_t RC_Init(TaskHandle_t owner_task);
void RC_Receive(void);
BaseType_t RC_GetSnapshot(ELRS_t *snapshot);
void Data_Change(int16_t *value);
void Dead_Zone(int16_t *value);
void Channel_optimization_key(int16_t *value);

/* 调试镜像；业务读取必须经过 RC_GetSnapshot()，内部使用 xQueuePeek。 */
extern volatile ELRS_t Elrs;
extern volatile uint16_t rx_len;
extern volatile uint8_t rc_rx_started;
extern volatile uint32_t rc_rx_event_count;
extern volatile uint32_t rc_parse_ok_count;
extern volatile uint32_t rc_header_fail_count;
extern volatile uint32_t rc_crc_fail_count;
extern volatile uint32_t rc_uart_error_count;
extern volatile uint32_t rc_arm_ok_count;
extern volatile uint32_t rc_arm_fail_count;
extern volatile uint32_t rc_last_error_code;
extern volatile uint16_t rc_last_expected_size;
extern volatile uint8_t rc_last_arm_status;
extern volatile uint8_t rc_last_frame_len;
extern volatile uint8_t rc_last_frame_type;
extern volatile uint8_t rc_last_crc_calc;
extern volatile uint8_t rc_last_crc_recv;
extern volatile uint8_t rc_last_rx_data[8];
extern volatile uint8_t rc_code12_pack[4];
extern volatile uint8_t rc_code12_trigger;

extern volatile uint32_t rc_ring_overflow_count;
extern volatile uint32_t rc_dropped_byte_count;
extern volatile uint32_t rc_recovery_count;
extern volatile uint16_t rc_ring_high_water;

#endif
