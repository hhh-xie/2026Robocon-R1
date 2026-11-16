#ifndef _ARM_MOTOR_H
#define _ARM_MOTOR_H

#include "motor_dm.h"
#include "fdcan.h"
#include "stdio.h"
#include "math.h"
#include "arm_math.h"
#include "remote_control.h"
#include "gpio.h"

typedef struct
{
    float base;
    float shoulder;
    float elbow;
    float shoulder_tff;
    float elbow_tff;
} Arm_Motor_t;

#define ARM_JOYSTICK_DEADBAND 0.08f
#define ARM_JOYSTICK_FULL_SCALE 10.0f
#define ARM_JOYSTICK_EXPO 0.60f
#define ARM_BASE_MAX_STEP 0.012f
#define ARM_SHOULDER_MAX_STEP 0.008f
#define ARM_ELBOW_MAX_STEP 0.015f

#define ARM_RC_DEAD_ZONE 100
#define ARM_RC_STEP 0.00005f

#define ARM_ANGLE_JOYSTICK_THRESHOLD 1.0f
#define ARM_ANGLE_STEP 0.01f

#define ARM_BASE_MIN (-6.14f)
#define ARM_BASE_MAX (6.14f)
#define ARM_SHOULDER_MIN (-6.57f)
#define ARM_SHOULDER_MAX (6.57f)
#define ARM_ELBOW_MIN (-6.20f)
#define ARM_ELBOW_MAX (6.20f)

#define ARM_BASE_KP 18.0f
#define ARM_BASE_KD 4.8f

#define ARM_SHOULDER_KP 100.0f
#define ARM_SHOULDER_KD 6.0f

#define ARM_ELBOW_KP 60.0f
#define ARM_ELBOW_KD 3.2f


#define DM4340_P_MIN (-12.5f)
#define DM4340_P_MAX (12.5f)
#define DM4340_V_MIN (-30.0f)
#define DM4340_V_MAX (30.0f)
#define DM4340_T_MIN (-28.0f)
#define DM4340_T_MAX (28.0f)

//+6到-6，+逆时针
#define DM10010_P_MIN (-12.5f)
#define DM10010_P_MAX (12.5f)
#define DM10010_V_MIN (-25.0f)
#define DM10010_V_MAX (25.0f)
#define DM10010_T_MIN (-200.0f)
#define DM10010_T_MAX (200.0f)

#define ARM_GRAVITY_SHOULDER 20.0f
#define ARM_GRAVITY_ELBOW 3.0f

#define ARM_LINK1 0.49f
#define ARM_LINK2 0.35f

#define ARM_BASE_HEIGHT 0.0f
#define ARM_IK_ELBOW_DIR 1.0f

void Arm_Remote_Update(const ELRS_t *remote);
void Arm_Motor_Init(void);
void Arm_Motor_Start(void);
void Arm_Motor_Stop(void);
void Arm_Mode_Enter(void);
void Arm_Auto_Task_Update(void);
void Arm_Suction_Set(uint8_t enable);

void Arm_RC_Control(int16_t x, int16_t y, int16_t z);
void Arm_RC_Control_From_Remote(void);
void Arm_Angle_Control(float dm4340p1_angle, float dm10010l_angle, float dm4340p2_angle);
void Arm_Move_To_Position(float x, float y, float z);
void Arm_Motor_Control(void);

#endif
