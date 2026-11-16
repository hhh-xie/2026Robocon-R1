#include "lift_motor.h"

#define STEP 10000.0f
#define DEBUG_STEP 1000.0f

#define min_pos (-3530332.0f * 19.0f)
#define max_pos (3530332.0f * 19.0f)

#define RC_Value remote->ch[9]
#define RC_Control_Value 400
#define RC_Debug_Value1 remote->ch[2]
#define RC_Debug_Value2 remote->ch[1]

#define LIFT_SENSOR_MAX_VALUE 305U
#define LIFT_SENSOR_MIN_VALUE 70U

static float lift_current_target = 0.0f;

static float id5_init_ecd = 0.0f;
static uint8_t id5_position_initialized = 0U;
static float id6_init_ecd = 0.0f;
static uint8_t id6_position_initialized = 0U;
static float debug_ecd1 = 0.0f;
static float debug_ecd2 = 0.0f;

static float Lift_Limit_Delta(float delta)
{
#if 0
    if (sensor.update == 0U)
    {
        return delta;
    }

    if ((sensor.value >= LIFT_SENSOR_MAX_VALUE) && (delta > 0.0f))
    {
        return 0.0f;
    }

    if ((sensor.value <= LIFT_SENSOR_MIN_VALUE) && (delta < 0.0f))
    {
        return 0.0f;
    }
#endif

    return delta;
}

void Lift_Motor_Control(const ELRS_t *remote)
{
    if ((id5_position_initialized == 0U) && (motor_3508.ID5.cnt != 0))
    {
        id5_init_ecd = (float)motor_3508.ID5.total_ecd;
        lift_current_target = 0.0f;
        id5_position_initialized = 1U;
    }
    if (id5_position_initialized == 0U)
    {
        motor_3508.ID5.target_ecd = (float)motor_3508.ID5.total_ecd;
        return;
    }

    if ((id6_position_initialized == 0U) && (motor_3508.ID6.cnt != 0))
    {
        id6_init_ecd = (float)motor_3508.ID6.total_ecd;
        lift_current_target = 0.0f;
        id6_position_initialized = 1U;
    }
    if (id6_position_initialized == 0U)
    {
        motor_3508.ID6.target_ecd = (float)motor_3508.ID6.total_ecd;
        return;
    }

    if ((remote->ch[7] != 2) &&
        ((RC_Value > RC_Control_Value) || (RC_Value < -RC_Control_Value)))
    {
        float lift_delta = (RC_Value > 0) ? STEP : -STEP;
        lift_current_target += Lift_Limit_Delta(lift_delta);
    }

    if (remote->ch[7] == 2)
    {
        if ((RC_Debug_Value1 > RC_Control_Value) || (RC_Debug_Value1 < -RC_Control_Value))
        {
            debug_ecd1 += (RC_Debug_Value1 > 0) ? DEBUG_STEP : -DEBUG_STEP;
        }
        if ((RC_Debug_Value2 > RC_Control_Value) || (RC_Debug_Value2 < -RC_Control_Value))
        {
            debug_ecd2 += (RC_Debug_Value2 > 0) ? DEBUG_STEP : -DEBUG_STEP;
        }
    }

    if (lift_current_target > max_pos) lift_current_target = max_pos;
    if (lift_current_target < min_pos) lift_current_target = min_pos;

    motor_3508.ID5.target_ecd = id5_init_ecd + lift_current_target + debug_ecd1;
    motor_3508.ID6.target_ecd = id6_init_ecd + lift_current_target + debug_ecd2;
}
