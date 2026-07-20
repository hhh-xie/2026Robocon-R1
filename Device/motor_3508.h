#ifndef _MOTOR_3508_H
#define _MOTOR_3508_H

#include "stm32g4xx_hal.h"
#include "motor_dm.h"
#include "stdint.h"
#include "fdcan.h"
#include "pid.h"

extern M3508 motor_3508;

void CAN1_Motor_3508(uint32_t ID, int16_t motor1_i, int16_t motor2_i, int16_t motor3_i, int16_t motor4_i);
void CAN2_Motor_3508(uint32_t ID, int16_t motor1_i, int16_t motor2_i, int16_t motor3_i, int16_t motor4_i);
void CAN3_Motor_3508(uint32_t ID, int16_t motor1_i, int16_t motor2_i, int16_t motor3_i, int16_t motor4_i);
void CAN1_Receive_Motor(M3508_msg *motor_msg,uint8_t *receive_data);
void CAN2_Receive_Motor(M3508_msg *motor_msg,uint8_t *receive_data);
void CAN3_Receive_Motor(M3508_msg *motor_msg,uint8_t *receive_data);

void M3508_Speed_Contorl(void);
void M3508_Angle_Contorl(void);
void M3508_Chassis_Angle_Contorl(void);

#endif
