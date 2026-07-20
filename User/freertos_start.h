#ifndef __FREERTOS_START_H__
#define __FREERTOS_START_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "string.h"
#include "main.h"
#include "pid.h"
#include "usart.h"
#include "remote_control.h"
#include "chassic_motor.h"
#include "arm_motor.h"
#include "lift_motor.h"
#include "claw_motor.h"
#include "motor_3508.h"
#include "motor_dm.h"


void freertos_start(void);

extern TaskHandle_t remote_task_handle;
extern TaskHandle_t arm_task_handle;
extern TaskHandle_t m3508_pid_task_handle;
extern TaskHandle_t debug_task_handle;
extern TaskHandle_t lift_claw_task_handle;
extern TaskHandle_t chassis_task_handle;
extern TaskHandle_t gpio_task_handle;

#endif
