#ifndef _LIFT_MOTOR_H
#define _LIFT_MOTOR_H

#include "remote_control.h"
#include "motor_3508.h"
#include "math.h"
#include "pid.h"
#include "usart.h"

void Lift_Motor_Control(const ELRS_t *remote);

#endif
