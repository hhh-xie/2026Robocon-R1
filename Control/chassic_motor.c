#include "chassic_motor.h"

#define LF_WHEEL_SPEED motor_3508.ID1.target_speed
#define RF_WHEEL_SPEED motor_3508.ID2.target_speed
#define LB_WHEEL_SPEED motor_3508.ID3.target_speed
#define RB_WHEEL_SPEED motor_3508.ID4.target_speed

static int lf_dir = 1;
static int rf_dir = -1;
static int lb_dir = -1;
static int rb_dir = 1;


void wheel_control(int16_t vx, int16_t vy, int16_t vw)
{
    float scale_factor;
    float max_speed;
    
    LF_WHEEL_SPEED = lf_dir*(vy + vx + vw);
    RF_WHEEL_SPEED = rf_dir*(vy - vx - vw);
    RB_WHEEL_SPEED = rb_dir*(vy + vx - vw);
    LB_WHEEL_SPEED = lb_dir*(vy - vx + vw);

    if(LF_WHEEL_SPEED < 200 && LF_WHEEL_SPEED > -200)
    {
        LF_WHEEL_SPEED = 0;
    }
    if(RF_WHEEL_SPEED < 200 && RF_WHEEL_SPEED > -200)
    {
        RF_WHEEL_SPEED = 0;
    }
    if(RB_WHEEL_SPEED < 200 && RB_WHEEL_SPEED > -200)
    {
        RB_WHEEL_SPEED = 0;
    }
    if(LB_WHEEL_SPEED < 200 && LB_WHEEL_SPEED > -200)
    {
        LB_WHEEL_SPEED = 0;
    }

    max_speed = LF_WHEEL_SPEED;
    if (max_speed < 0.0f) max_speed = -max_speed;
    if (RF_WHEEL_SPEED > max_speed) max_speed = RF_WHEEL_SPEED;
    if (-RF_WHEEL_SPEED > max_speed) max_speed = -RF_WHEEL_SPEED;
    if (RB_WHEEL_SPEED > max_speed) max_speed = RB_WHEEL_SPEED;
    if (-RB_WHEEL_SPEED > max_speed) max_speed = -RB_WHEEL_SPEED;
    if (LB_WHEEL_SPEED > max_speed) max_speed = LB_WHEEL_SPEED;
    if (-LB_WHEEL_SPEED > max_speed) max_speed = -LB_WHEEL_SPEED;

    if (max_speed > PID_SPEED_MAX_OUT)
    {
        scale_factor = (float)PID_SPEED_MAX_OUT / max_speed;
        LF_WHEEL_SPEED *= scale_factor;
        RF_WHEEL_SPEED *= scale_factor;
        RB_WHEEL_SPEED *= scale_factor;
        LB_WHEEL_SPEED *= scale_factor;
    }
}

