#ifndef _MOTOR_DM_H_
#define _MOTOR_DM_H_

#include "stdint.h"
#include "string.h"
#include "fdcan.h"
#include "pid.h"

#define DMReturn_ID 0x00

#define DM4340p1_ID 0x01
#define DM10010L_ID 0x02
#define DM4340p2_ID 0x03
#define DM4310_ID 0x04

#define DM_Mode_MIT 1
#define DM_Mode_Pos_Speed 2
#define DM_Mode_Speed 3
#define DM_Mode_Pos_Speed_Current 4

#define DM_REG_OT_VALUE     2
#define DM_REG_OC_VALUE     3
#define DM_REG_ACC          4
#define DM_REG_DEC          5
#define DM_REG_MAX_SPD      6
#define DM_REG_TIMEOUT      9
#define DM_REG_CTRL_MODE    10

extern DM_Motor dm_motor;

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

void CAN2_Motor_DM_Pos_Speed(uint32_t ID,float p_des, float v_des);
void CAN3_Motor_DM_Pos_Speed_4310(uint32_t ID,float p_des, float v_des);
void CAN2_Motor_DM_Pos_Speed_Current(uint32_t ID,float p_des,float v_des,float i_des);
void CAN2_Motor_DM_MIT(uint32_t ID,float p_des,float v_des,float kp,float kd,float t_ff,float P_MIN,float P_MAX,float V_MIN,float V_MAX,float T_MIN,float T_MAX);

void CAN2_Motor_DM_Commom_Cmd(uint32_t ID,uint8_t* cmd);
void CAN3_Motor_DM_Commom_Cmd(uint32_t ID,uint8_t* cmd);
void DM_Start(uint8_t mode,uint32_t ID);
void DM_Stop(uint8_t mode,uint32_t ID);
void DM_Save_Zone(uint8_t mode,uint32_t ID);
void DM_Clear_Error(uint8_t mode,uint32_t ID);

void CAN2_Motor_DM_Write_Reg(uint32_t motor_id, uint8_t rid, uint32_t value);
void CAN2_Motor_DM_Write_Reg_Float(uint32_t motor_id, uint8_t rid, float value);
void CAN2_Motor_DM_Write_Reg_Uint32(uint32_t motor_id, uint8_t rid, uint32_t value);
void CAN2_Motor_DM_Save_Reg(uint32_t motor_id);
void CAN2_Motor_DM_Feedback_Update(DM_Motor* dm_motor, uint8_t *rx_data);

void DM_Set_Mode(uint32_t id,uint8_t mode);
void DM_Set_AccDec(uint32_t id, float acc, float dec);
void DM_Set_MaxSpeed(uint32_t id, float max_speed);
void DM_Set_Timeout(uint32_t id, uint32_t timeout);
void DM_Set_CurrentLimit(uint32_t id, float oc_value);
void DM_Set_TempLimit(uint32_t id, float temp_limit);
void DM_Save_Params(uint32_t id);


#endif

