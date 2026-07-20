#ifndef PID_H
#define PID_H

#include "stm32g4xx_hal.h"
#include "stdlib.h"

#define PID_SPEED_MAX_OUT 16384
#define PID_ANGLE_MAX_OUT 16384

enum OID_MODE
{
  PID_POSITION = 1,
  PID_SPEED,
  PID_DELAT
};

typedef struct{
  uint8_t mode;
  float kp;
  float ki;
  float kd;
  float set;
  float get;
  float out;
  float pout;
  float iout;
  float dout;
  float max_out;
  float max_iout;
  float err[3];
  float dbuf[3];
}speed_pid_t;

typedef struct{
  speed_pid_t M3508_ID1;
  speed_pid_t M3508_ID2;
  speed_pid_t M3508_ID3;
  speed_pid_t M3508_ID4;
  speed_pid_t M3508_ID5;
  speed_pid_t M3508_ID6;
  speed_pid_t M3508_ID7;
}speed_ID;

typedef struct{
  uint8_t mode;
  float kp;
  float ki;
  float kd;
  float set;
  float get;
  float out;
  float pout;
  float iout;
  float dout;
  float max_out;
  float max_iout;
  float err[3];
  float dbuf[3];
}angle_pid_t;

typedef struct{
  angle_pid_t M3508_ID1;
  angle_pid_t M3508_ID2;
  angle_pid_t M3508_ID3;
  angle_pid_t M3508_ID4;
  angle_pid_t M3508_ID5;
  angle_pid_t M3508_ID6;
  angle_pid_t M3508_ID7;
}angle_ID;



typedef struct{
  uint8_t temperature;
  int16_t current;
  int16_t current_out;
  int16_t speed;
  int16_t target_speed;//0-16384
  uint16_t ecd;
  uint16_t last_ecd;
  int32_t total_ecd;
  float target_ecd;//0-8192
  uint8_t cnt;
}M3508_msg;

typedef struct {
  M3508_msg ID1;
  M3508_msg ID2;
  M3508_msg ID3;
  M3508_msg ID4;
  M3508_msg ID5;
  M3508_msg ID6;
  M3508_msg ID7;
}M3508;

typedef struct{
  int16_t position;
  int16_t speed;
  uint16_t torque;
  uint8_t T_Mos;
  uint8_t T_Rotor;
  uint8_t ID;
  uint8_t ERR;
}DM_Motor_msg;

typedef struct{
  DM_Motor_msg DM4340p1_ID1;
  DM_Motor_msg DM10010L_ID2;
  DM_Motor_msg DM4340p2_ID3;
  DM_Motor_msg DM4310_ID4;
}DM_Motor;

void  PID_Speed_Init(speed_ID *pid);
void  PID_Angle_Init(angle_ID *pid);
float PID_Speed_Calc_3508(M3508_msg *motor_msg, speed_pid_t *pid);
float PID_Angle_Calc_3508(M3508_msg *motor_msg, angle_pid_t *pid);
void  get_total_3508_angle(M3508_msg *p);

extern speed_ID motor_speed;
extern angle_ID motor_angle;

#endif

