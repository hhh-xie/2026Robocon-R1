#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define _CLAW_MOTOR_H
#define DM4310_ID 0x04U

typedef struct
{
    int16_t ch[16];
} RemoteControlStub;
typedef RemoteControlStub ELRS_t;

typedef struct
{
    float target_ecd;
    int32_t total_ecd;
    uint8_t cnt;
} Motor3508MessageStub;

typedef struct
{
    Motor3508MessageStub ID7;
} Motor3508Stub;

RemoteControlStub Elrs;
Motor3508Stub motor_3508;

static float transmitted_position;
static float transmitted_speed;

void CAN3_Motor_DM_Pos_Speed_4310(uint32_t id, float position, float speed)
{
    assert(id == DM4310_ID);
    transmitted_position = position;
    transmitted_speed = speed;
}

void CAN3_Motor_DM_Commom_Cmd(uint32_t id, uint8_t *cmd)
{
    (void)id;
    (void)cmd;
}

#include "../Control/claw_motor.c"

static void Assert_Float_Near(float actual, float expected)
{
    if (fabsf(actual - expected) >= 0.00001f)
    {
        fprintf(stderr, "float mismatch: actual=%f expected=%f\n", actual, expected);
    }
    assert(fabsf(actual - expected) < 0.00001f);
}

static void Reset_DM_Control(void)
{
    uint8_t i;

    for (i = 0U; i < 16U; i++)
    {
        Elrs.ch[i] = 0;
    }

    claw_dm_button_last = 0xFFU;
    claw_arm_mode_last = 0U;
    claw_dm_target_valid = 0U;
    claw_dm_target_pos = DM4310_Pos1;
    claw_dm_command_pos = DM4310_Pos1;
    claw_position_initialized = 0U;
    claw_home_finished = 0U;
    claw_ch8_target_reached = 0U;
    claw_ch8_last = 0xFFU;
    claw_ch8_auto_active = 0U;
    dm4310_init_count = 2U;
    motor_3508.ID7.target_ecd = 0.0f;
    motor_3508.ID7.total_ecd = 0;
    motor_3508.ID7.cnt = 0U;
    transmitted_position = 0.0f;
    transmitted_speed = 0.0f;
}

static void Test_Forward_Command_Uses_One_Step(void)
{
    Reset_DM_Control();
    Elrs.ch[6] = 2;

    Claw_DM_Button_Control(&Elrs);

    Assert_Float_Near(transmitted_position, DM4310_Pos1 + CLAW_DM_POSITION_STEP);
    Assert_Float_Near(transmitted_speed, 5.0f);
}

static void Test_Reverse_Command_Uses_One_Step(void)
{
    Assert_Float_Near(Claw_Approach_Target(DM4310_Pos2, DM4310_Pos1, CLAW_DM_POSITION_STEP),
                      DM4310_Pos2 - CLAW_DM_POSITION_STEP);
}

static void Test_Final_Command_Clamps_To_Target(void)
{
    Assert_Float_Near(Claw_Approach_Target(DM4310_Pos3 - 0.01f, DM4310_Pos3, 0.02f),
                      DM4310_Pos3);
}

static void Test_Ch8_Auto_Also_Selects_DM_Pos2(void)
{
    Reset_DM_Control();
    claw_position_initialized = 1U;
    claw_home_finished = 1U;
    claw_ch8_last = 1U;
    claw_target_ecd = CLAW_HOME_ECD;
    Elrs.ch[8] = CLAW_CH8_AUTO_VALUE;

    Claw_Motor_Control(&Elrs);

    assert(claw_ch8_auto_active != 0U);
    assert(claw_dm_target_valid != 0U);
    Assert_Float_Near(claw_dm_target_pos, DM4310_Pos2);
}

int main(void)
{
    Test_Forward_Command_Uses_One_Step();
    Test_Reverse_Command_Uses_One_Step();
    Test_Final_Command_Clamps_To_Target();
    Test_Ch8_Auto_Also_Selects_DM_Pos2();
    puts("claw DM step tests passed");
    return 0;
}
