#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__ 

#include "stm32g4xx_hal.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

typedef struct
{
    int16_t ch[32];//这里根据具体的遥控器通道定义来设置有多少个通道
    uint8_t code12[12];
    uint8_t code12_update_flag;
    uint32_t code12_update_count;
}ELRS_t;


void RC_Receive(void);
uint8_t RC_Receive_Data(const uint8_t *data_buff);
void Data_Change(int16_t *value);
void Dead_Zone(int16_t *value);
void Channel_optimization_key(int16_t *value);

extern volatile ELRS_t Elrs;
extern volatile uint8_t rx_end_flag;
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

#endif
