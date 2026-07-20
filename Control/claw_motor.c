#include "claw_motor.h"

/*
    3508电机控制夹爪的纵向旋转，达妙4310控制夹爪的平面旋转
*/

#define CLAW_MANUAL_STEP 25.0f            //手动控制时的步长
#define CLAW_FINE_STEP 10.0f              //到目标角度之后的微调步长
#define CLAW_HOME_ECD 1558.0f             //初始ecd可直接夹取
#define CLAW_AUTO_STEP 280.0f             //自动控制时的步长
#define CLAW_CH8_TARGET_ECD 50350.0f      //对接时的目标ecd
#define CLAW_MIN_OFFSET (0.0f)            //最小偏移量
#define CLAW_MAX_OFFSET (74243.0f)        //最大偏移量
#define CLAW_DM_POSITION_STEP 0.03f       //达妙位置指令每10ms的最大变化量(rad)

#define DM4310_Pos1 (-1.94f)     
#define DM4310_Pos2 (1.18f)
#define DM4310_Pos3 (3.5f)                //1、2区3.5f       

#define CLAW_DM_BUTTON Elrs.ch[6]
#define CLAW_ARM_MODE_VALUE Elrs.ch[7]
#define CLAW_CH8_VALUE Elrs.ch[8]
#define CLAW_RC_VALUE Elrs.ch[11]
#define CLAW_ARM_MODE_ACTIVE_VALUE 3
#define CLAW_CH8_AUTO_VALUE 2
#define CLAW_RC_CONTROL_VALUE 400

static float claw_current_offset = 0.0f;
static float claw_home_ecd = CLAW_HOME_ECD;
static float claw_target_ecd = 0.0f;
static uint8_t claw_position_initialized = 0U;
static uint8_t claw_home_finished = 0U;
static uint8_t claw_ch8_target_reached = 0U;
static uint8_t claw_ch8_last = 0xFFU;
static uint8_t claw_ch8_auto_active = 0U;
static uint8_t claw_dm_button_last = 0xFFU;
static uint8_t claw_arm_mode_last = 0U;
static uint8_t claw_dm_target_valid = 0U;
static float claw_dm_target_pos = DM4310_Pos1;
static float claw_dm_command_pos = DM4310_Pos1;
static uint8_t dm_start_cmd_4310[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

static uint8_t dm4310_init_count = 0U;

static float Claw_Limit_Float(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    if (value < min_value)
    {
        return min_value;
    }
    return value;
}

static float Claw_Approach_Target(float current, float target, float step)
{
    float delta = target - current;

    if ((delta <= step) && (delta >= -step))
    {
        return target;
    }

    return current + ((delta > 0.0f) ? step : -step);
}

static void Claw_Set_Target_Ecd(float target_ecd)
{
    claw_target_ecd = target_ecd;
    motor_3508.ID7.target_ecd = claw_target_ecd;
    claw_current_offset = Claw_Limit_Float(claw_target_ecd - claw_home_ecd,CLAW_MIN_OFFSET,CLAW_MAX_OFFSET);
}

static void Claw_Update_DM_Target_From_Button(uint8_t button)
{
    if(Elrs.ch[7] == CLAW_ARM_MODE_ACTIVE_VALUE)
    {
        return;
    }
    if (button == 1U)
    {
        claw_dm_target_pos = DM4310_Pos1;
        claw_dm_target_valid = 1U;
    }
    else if (button == 2U)
    {
        claw_dm_target_pos = DM4310_Pos2;
        claw_dm_target_valid = 1U;
    }
    else if (button == 3U)
    {
        claw_dm_target_pos = DM4310_Pos3;
        claw_dm_target_valid = 1U;
    }
}

static void Claw_DM_Button_Control(void)
{
    uint8_t dm_button = (uint8_t)CLAW_DM_BUTTON;
    uint8_t arm_mode = (uint8_t)CLAW_ARM_MODE_VALUE;

    if (arm_mode == CLAW_ARM_MODE_ACTIVE_VALUE)
    {
        claw_dm_button_last = dm_button;
        claw_arm_mode_last = arm_mode;
        return;
    }

    if (claw_arm_mode_last == CLAW_ARM_MODE_ACTIVE_VALUE)
    {
        claw_dm_button_last = dm_button;
    }
    else if ((claw_dm_button_last == 0xFFU) || (dm_button != claw_dm_button_last))
    {
        Claw_Update_DM_Target_From_Button(dm_button);
        claw_dm_button_last = dm_button;
    }

    if (claw_dm_target_valid != 0U)
    {
        claw_dm_command_pos = Claw_Approach_Target(claw_dm_command_pos,
                                                   claw_dm_target_pos,
                                                   CLAW_DM_POSITION_STEP);
        CAN3_Motor_DM_Pos_Speed_4310(DM4310_ID, claw_dm_command_pos, 5.0f);
    }

    claw_arm_mode_last = arm_mode;
}


void Claw_Motor_Control(void)
{
    if(dm4310_init_count < 2U)
    {
        CAN3_Motor_DM_Commom_Cmd(DM4310_ID+0x100, dm_start_cmd_4310);
        dm4310_init_count++;
    }

    Claw_DM_Button_Control();

    if ((claw_position_initialized == 0U) && (motor_3508.ID7.cnt != 0))
    {
        claw_current_offset = 0.0f;
        claw_target_ecd = (float)motor_3508.ID7.total_ecd;
        motor_3508.ID7.target_ecd = claw_target_ecd;
        claw_position_initialized = 1U;
    }

    if (claw_position_initialized == 0U)
    {
        motor_3508.ID7.target_ecd = (float)motor_3508.ID7.total_ecd;
        return;
    }

    if (claw_home_finished == 0U)
    {
        Claw_Set_Target_Ecd(Claw_Approach_Target(claw_target_ecd, CLAW_HOME_ECD, CLAW_AUTO_STEP));
        if (claw_target_ecd == CLAW_HOME_ECD)
        {
            claw_home_finished = 1U;
        }
        return;
    }

    if (CLAW_ARM_MODE_VALUE == CLAW_ARM_MODE_ACTIVE_VALUE)
    {
        claw_ch8_target_reached = 0U;
        claw_ch8_auto_active = 0U;
        claw_ch8_last = (uint8_t)CLAW_CH8_VALUE;
        motor_3508.ID7.target_ecd = claw_target_ecd;
        return;
    }

    if (claw_arm_mode_last == CLAW_ARM_MODE_ACTIVE_VALUE)
    {
        claw_ch8_last = (uint8_t)CLAW_CH8_VALUE;
    }
    else if ((CLAW_CH8_VALUE == CLAW_CH8_AUTO_VALUE) && (claw_ch8_last != CLAW_CH8_AUTO_VALUE))
    {
        claw_ch8_auto_active = 1U;
        claw_ch8_target_reached = 0U;
        claw_dm_target_pos = DM4310_Pos2;
        claw_dm_target_valid = 1U;
    }
    else if (CLAW_CH8_VALUE != CLAW_CH8_AUTO_VALUE)
    {
        claw_ch8_auto_active = 0U;
        claw_ch8_target_reached = 0U;
    }

    claw_ch8_last = (uint8_t)CLAW_CH8_VALUE;

    if (claw_ch8_auto_active != 0U)
    {
        if (claw_ch8_target_reached == 0U)
        {
            Claw_Set_Target_Ecd(Claw_Approach_Target(claw_target_ecd, CLAW_CH8_TARGET_ECD, CLAW_AUTO_STEP));
            if (claw_target_ecd == CLAW_CH8_TARGET_ECD)
            {
                claw_ch8_target_reached = 1U;
                claw_ch8_auto_active = 0U;
            }
            return;
        }

        if((CLAW_RC_VALUE > CLAW_RC_CONTROL_VALUE) || (CLAW_RC_VALUE < -CLAW_RC_CONTROL_VALUE))
        {
            claw_current_offset += (CLAW_RC_VALUE > 0) ? CLAW_FINE_STEP : -CLAW_FINE_STEP;
        }
    }
    else
    {
        claw_ch8_target_reached = 0U;

        if((CLAW_RC_VALUE > CLAW_RC_CONTROL_VALUE) || (CLAW_RC_VALUE < -CLAW_RC_CONTROL_VALUE))
        {
            claw_current_offset += (CLAW_RC_VALUE > 0) ? CLAW_MANUAL_STEP : -CLAW_MANUAL_STEP;
        }
    }

    claw_current_offset = Claw_Limit_Float(claw_current_offset, CLAW_MIN_OFFSET, CLAW_MAX_OFFSET);
    Claw_Set_Target_Ecd(claw_home_ecd + claw_current_offset);

}
