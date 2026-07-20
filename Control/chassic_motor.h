#ifndef _CHASSIC_MOTOR_H
#define _CHASSIC_MOTOR_H

#include "stm32g4xx_hal.h"
#include "motor_3508.h"
#include "pid.h"

void wheel_control(int16_t vx, int16_t vy, int16_t vw);

#endif
