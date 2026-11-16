#include "freertos_start.h"


/* ============================== FreeRTOS 任务配置 ==============================
 * 数字越大的任务优先级越高；任务栈大小的单位是 StackType_t，而不是字节。
 * 遥控任务由通知唤醒；其他周期任务均使用 vTaskDelayUntil()，以固定时间基准运行，减少周期漂移。
 */

/* 启动任务：只负责创建其他业务任务，完成后立即删除自身。 */
#define START_TASK_STACK        256
#define START_TASK_PRIORITY     1
TaskHandle_t start_task_handle;
static void StartTask(void *pvParameters);

/* 遥控任务：通知唤醒后读取软件环形区、组帧并发布最新状态队列。 */
#define REMOTE_TASK_STACK       512
#define REMOTE_TASK_PRIORITY    4
#define REMOTE_TASK_RETRY_MS    10U
TaskHandle_t remote_task_handle;
static void RemoteTask(void *pvParameters);

/* 机械臂任务：每 5 ms 更新摇杆控制、自动路径和达妙电机目标。 */
#define ARM_TASK_STACK          768
#define ARM_TASK_PRIORITY       6
#define ARM_TASK_PERIOD_MS      5U
TaskHandle_t arm_task_handle;
static void ArmTask(void *pvParameters);

/* 3508 PID 任务：最高业务优先级，每 1 ms 更新底盘、升降和夹爪电机闭环。 */
#define M3508_PID_TASK_STACK        512
#define M3508_PID_TASK_PRIORITY     7
#define M3508_PID_TASK_PERIOD_MS    1U
TaskHandle_t m3508_pid_task_handle;
static void M3508PidTask(void *pvParameters);

/* 调试任务：预留低频串口打印位置，避免高频打印影响实时控制。 */
#define DEBUG_TASK_STACK        512
#define DEBUG_TASK_PRIORITY     1
#define DEBUG_TASK_PERIOD_MS    500U
TaskHandle_t debug_task_handle;
static void DebugTask(void *pvParameters);

/* 升降与夹爪任务：每 10 ms 根据遥控输入更新两套机构的控制目标。 */
#define LIFT_CLAW_TASK_STACK        512
#define LIFT_CLAW_TASK_PRIORITY     4
#define LIFT_CLAW_TASK_PERIOD_MS    10U
TaskHandle_t lift_claw_task_handle;
static void LiftClawTask(void *pvParameters);

/* 底盘指令任务：每 5 ms 读取遥控器并在速度控制与停车位置保持间切换。 */
#define CHASSIS_TASK_STACK      512
#define CHASSIS_TASK_PRIORITY   5
#define CHASSIS_TASK_PERIOD_MS  5U
TaskHandle_t chassis_task_handle;
static void ChassisTask(void *pvParameters);
/* 由底盘任务写、3508 PID 任务读；volatile 防止编译器缓存跨任务共享状态。 */
static volatile uint8_t chassis_hold_mode = 0U;

/* GPIO/串口辅助任务：每 20 ms 处理吸盘、外部 GPIO 和 Code12 数据发送。 */
#define GPIO_TASK_STACK         256
#define GPIO_TASK_PRIORITY      2
#define GPIO_TASK_PERIOD_MS     20U
TaskHandle_t gpio_task_handle;
static void GpioTask(void *pvParameters);

/* FreeRTOS 启动入口，由 main() 在外设、CAN 过滤器和 PID 初始化后调用。 */
void freertos_start(void)
{
    /* 调度器启动前先创建唯一的 StartTask，由它在调度开始后创建业务任务。 */
    xTaskCreate((TaskFunction_t)StartTask,
                (char *)"StartTask",
                (configSTACK_DEPTH_TYPE)START_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)START_TASK_PRIORITY,
                (TaskHandle_t *)&start_task_handle);
    /* 启动抢占式调度；正常情况下此函数不会返回。 */
    vTaskStartScheduler();
}

/* 一次性任务创建器：依次创建全部业务任务，然后释放自己的栈和控制块。 */
static void StartTask(void *pvParameters)
{
    /* 当前创建任务时未传入用户参数，显式转换为 void 消除未使用警告。 */
    (void)pvParameters;

    /* 创建遥控接收任务，任务句柄保存到全局变量供调试或外部管理使用。 */
    xTaskCreate((TaskFunction_t)RemoteTask,
                (char *)"RemoteTask",
                (configSTACK_DEPTH_TYPE)REMOTE_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)REMOTE_TASK_PRIORITY,
                (TaskHandle_t *)&remote_task_handle);

    /* 创建机械臂控制任务。 */
    xTaskCreate((TaskFunction_t)ArmTask,
                (char *)"ArmTask",
                (configSTACK_DEPTH_TYPE)ARM_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)ARM_TASK_PRIORITY,
                (TaskHandle_t *)&arm_task_handle);

    /* 创建 3508 电机高频 PID 任务。 */
    xTaskCreate((TaskFunction_t)M3508PidTask,
                (char *)"M3508PidTask",
                (configSTACK_DEPTH_TYPE)M3508_PID_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)M3508_PID_TASK_PRIORITY,
                (TaskHandle_t *)&m3508_pid_task_handle);

    /* 创建低优先级、低频率的调试任务。 */
    xTaskCreate((TaskFunction_t)DebugTask,
                (char *)"DebugTask",
                (configSTACK_DEPTH_TYPE)DEBUG_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)DEBUG_TASK_PRIORITY,
                (TaskHandle_t *)&debug_task_handle);

    /* 创建升降和夹爪目标更新任务。 */
    xTaskCreate((TaskFunction_t)LiftClawTask,
                (char *)"LiftClawTask",
                (configSTACK_DEPTH_TYPE)LIFT_CLAW_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)LIFT_CLAW_TASK_PRIORITY,
                (TaskHandle_t *)&lift_claw_task_handle);

    /* 创建底盘遥控与停车保持管理任务。 */
    xTaskCreate((TaskFunction_t)ChassisTask,
                (char *)"ChassisTask",
                (configSTACK_DEPTH_TYPE)CHASSIS_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)CHASSIS_TASK_PRIORITY,
                (TaskHandle_t *)&chassis_task_handle);

    /* 创建吸盘、外部输出和 Code12 通信任务。 */
    xTaskCreate((TaskFunction_t)GpioTask,
                (char *)"GpioTask",
                (configSTACK_DEPTH_TYPE)GPIO_TASK_STACK,
                (void *)NULL,
                (UBaseType_t)GPIO_TASK_PRIORITY,
                (TaskHandle_t *)&gpio_task_handle);

    /* NULL 表示删除当前任务；之后 StartTask 不再占用系统资源。 */
    vTaskDelete(NULL);
}

/* DMA 接收一次启动，ISR 只搬字节；组帧和 CRC 全部在本任务执行。 */
static void RemoteTask(void *pvParameters)
{
    (void)pvParameters;
    if (RC_Init(xTaskGetCurrentTaskHandle()) != pdPASS)
    {
        Error_Handler();
    }
    for (;;)
    {
        RC_Receive();
        /* 通知会被保留，不会丢失“处理结束到进入等待之间”到达的事件。
         * 10 ms 超时仅用于接收启动失败后的重试，不是遥控解析周期。
         */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(REMOTE_TASK_RETRY_MS));
    }
}

/* 机械臂调度任务：根据 B 三段开关在手动机械臂模式和后台自动路径间切换。 */
static void ArmTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    /* 记录上一周期是否处于 B=3，用于只在进入模式的瞬间执行一次初始化。 */
    uint8_t arm_mode_last = 0U;
    ELRS_t remote;

    (void)pvParameters;

    for (;;)
    {
        /* Peek 不移除状态；同一次循环的机械臂输入来自同一帧。 */
        RC_GetSnapshot(&remote);
        Arm_Remote_Update(&remote);
        if(remote.ch[7] == 3)
        {
            /* 从其他模式首次切入时，重新使能电机并同步路径和按键历史状态。 */
            if(arm_mode_last == 0U)
            {
                Arm_Motor_Start();
                Arm_Mode_Enter();
                arm_mode_last = 1U;
            }
            /* 右 X 控制底座、左 Y 控制肩部、右 Y 控制肘部；
             * /80 将遥控通道缩放到机械臂浮点摇杆范围，负号修正安装方向。
             */
            Arm_Angle_Control(remote.ch[0] / 80.0f, -remote.ch[2] / 80.0f, -remote.ch[1] / 80.0f);
        }
        else
        {
            /* 离开手动模式后不接收新的摇杆控制，但已启动的非阻塞路径可继续完成。 */
            Arm_Auto_Task_Update();
            /* 清除进入标志，使下次切回 B=3 时重新执行模式进入处理。 */
            arm_mode_last = 0U;
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(ARM_TASK_PERIOD_MS));
    }
}

/* 3508 电机闭环任务：以 1 kHz 频率生成并发送底盘、升降和夹爪电流指令。 */
static void M3508PidTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        /* 底盘停车保持时使用编码器位置环；有移动命令时使用速度环。 */
        if(chassis_hold_mode != 0U)
        {
            M3508_Chassis_Angle_Contorl();
        }
        else
        {
            M3508_Speed_Contorl();
        }
        /* 始终更新其他采用角度环控制的 3508 电机。 */
        M3508_Angle_Contorl();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(M3508_PID_TASK_PERIOD_MS));
    }
}

/* 低频调试任务：需要观察变量时可恢复下面已有的 printf，不影响其他任务周期。 */
static void DebugTask(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        /* 保留的原始调试输出；当前均被注释，因此任务只执行周期延时。 */
        //printf("%d\n",motor_3508.ID7.total_ecd);
        //printf("Sensor Value: %d\n", sensor.value);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(DEBUG_TASK_PERIOD_MS));
    }
}

/* 升降与夹爪共享一份队列快照和 10 ms 的人机控制周期。 */
static void LiftClawTask(void *pvParameters)
{
    ELRS_t remote;
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        RC_GetSnapshot(&remote);
        /* 根据遥控通道、限位传感器等信息更新升降机构目标。 */
        Lift_Motor_Control(&remote);
        /* 更新夹爪 3508/达妙电机目标及自动定位逻辑。 */
        Claw_Motor_Control(&remote);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LIFT_CLAW_TASK_PERIOD_MS));
    }
}

/* 底盘遥控任务：读取一份一致的遥控快照，并管理静止时的编码器位置保持。 */
static void ChassisTask(void *pvParameters)
{
    /* ch0=右 X（旋转），ch2=左 Y（前后），ch3=左 X（横移），ch7=B 模式开关。 */
    ELRS_t remote;
    int16_t ch0;
    int16_t ch2;
    int16_t ch3;
    int16_t ch7;
    /* 一次停车过程只锁存一次四轮当前位置，避免目标位置被反馈值不断覆盖。 */
    uint8_t chassis_hold_latched = 0U;
    TickType_t last_wake_time = xTaskGetTickCount();

    (void)pvParameters;

    for (;;)
    {
        RC_GetSnapshot(&remote);
        ch0 = remote.ch[0];
        ch2 = remote.ch[2];
        ch3 = remote.ch[3];
        ch7 = remote.ch[7];
        /* B=1 选择底盘模式；其他模式下底盘目标始终清零。 */
        if(ch7 == 1)
        {
            /* 三个运动通道均为 0 时进入停车流程。 */
            if((ch0 == 0) && (ch2 == 0) && (ch3 == 0))
            {
                /* 先把速度目标清零，避免继续沿上一条移动指令行驶。 */
                wheel_control(0, 0, 0);
                /* 四个电机都收到过有效反馈后，锁存当前累计编码器作为保持目标。 */
                if((chassis_hold_latched == 0U) &&
                   (motor_3508.ID1.cnt != 0U) &&
                   (motor_3508.ID2.cnt != 0U) &&
                   (motor_3508.ID3.cnt != 0U) &&
                   (motor_3508.ID4.cnt != 0U))
                {
                    /* 位置环以停车瞬间的四轮位置为目标，从而抵抗外力推动。 */
                    motor_3508.ID1.target_ecd = (float)motor_3508.ID1.total_ecd;
                    motor_3508.ID2.target_ecd = (float)motor_3508.ID2.total_ecd;
                    motor_3508.ID3.target_ecd = (float)motor_3508.ID3.total_ecd;
                    motor_3508.ID4.target_ecd = (float)motor_3508.ID4.total_ecd;
                    chassis_hold_latched = 1U;
                }
                /* 只有成功锁存目标后才允许高频 PID 任务切入底盘位置环。 */
                chassis_hold_mode = chassis_hold_latched;
            }
            else
            {
                /* 任一摇杆产生运动命令时退出停车保持，并为下一次停车解除锁存。 */
                chassis_hold_mode = 0U;
                chassis_hold_latched = 0U;
                /* 将横移、前后、旋转三个通道放大后交给麦轮速度分解函数。 */
                wheel_control(ch3 * 10, ch2 * 10, ch0 * 10);
            }
        }
        else
        {
            /* 非底盘模式清除速度和位置保持状态，防止模式切换后底盘残留动作。 */
            chassis_hold_mode = 0U;
            chassis_hold_latched = 0U;
            wheel_control(0, 0, 0);
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CHASSIS_TASK_PERIOD_MS));
    }
}

/* 把连续 4 个 Code12 数字打包成一个字节，供 UART4 的紧凑二进制帧使用。 */
static uint8_t Code12_Pack_4Digits(const uint8_t *digits)
{
    /* 将 4 个取值范围为 0～3 的数字压缩到一个字节，每个数字占 2 bit。 */
    uint8_t packed = 0U;

    for(uint8_t i = 0U; i < 4U; i++)
    {
        uint8_t value = digits[i];
        /* 防御性限幅，避免异常值占用相邻数字的位。 */
        if(value > 3U)
        {
            value = 3U;
        }

        /* 先左移已有数据，再把当前数字放入最低两位。 */
        packed = (uint8_t)((packed << 2U) | value);
    }

    return packed;
}

/* GPIO 与外部串口通信任务。
 * 根据 B/E/F/C 模式开关和 A/D 按键，在 Code12 红外发送、二维码串口、
 * 吸盘 PE3、辅助输出 PE1 之间选择功能；按键型串口命令使用上升沿防重复发送。
 */
static void GpioTask(void *pvParameters)
{
    ELRS_t remote;
    TickType_t last_wake_time = xTaskGetTickCount();
    /* 本周期使用的遥控通道快照：A、E、F、B、C、D。 */
    int16_t ch4;
    int16_t ch5;
    int16_t ch6;
    int16_t ch7;
    int16_t ch8;
    int16_t ch10;
    /* 标记 B=1、F=2、C=3 的放置吸盘直控组合。 */
    uint8_t place_suction_control_selected;
    /* Code12 原始 12 位数字及两种发送格式的临时缓冲区。 */
    uint8_t code12[12];
    uint8_t code12_msg[13];
    uint8_t code12_ir_msg[16];
    /* UART5 清空外部二维码模块/上位机状态的文本命令。 */
    uint8_t clear_msg[] = "clear\n";
    /* 三个“上一周期已发送”标志用于实现模式或按键的单次触发。 */
    uint8_t code12_send_last = 0U;
    uint8_t code12_ir_send_last = 0U;
    uint8_t clear_send_last = 0U;

    (void)pvParameters;

    for (;;)
    {
        /* 通道和 Code12 均来自这一份队列快照。 */
        RC_GetSnapshot(&remote);
        ch4 = remote.ch[4];
        ch5 = remote.ch[5];
        ch6 = remote.ch[6];
        ch7 = remote.ch[7];
        ch8 = remote.ch[8];
        ch10 = remote.ch[10];

        /* 该特殊组合与下面的 B=1、C=3 Code12 发送条件重叠，因此单独排除。 */
        place_suction_control_selected = ((ch7 == 1) && (ch6 == 2) && (ch8 == 3)) ? 1U : 0U;

        /* B=1、C=3 且不是放置吸盘组合：向 UART4 发送一次 Code12 压缩数据。 */
        if((ch7 == 1) && (ch8 == 3) && (place_suction_control_selected == 0U))
        {
            /* 保持在同一组合时只发送一次，离开后再次进入才允许重发。 */
            if(code12_ir_send_last == 0U)
            {
                /* 使用与模式开关相同快照中的 Code12。 */
                memcpy(code12, remote.code12, sizeof(code12));

                /* 16 字节帧先整体清零，前 3 字节分别打包 4 个 Code12 数字。 */
                memset(code12_ir_msg, 0, sizeof(code12_ir_msg));
                for(uint8_t i = 0U; i < 3U; i++)
                {
                    code12_ir_msg[i] = Code12_Pack_4Digits(&code12[i * 4U]);
                }

                /* 阻塞式发送完整 16 字节帧，超时时间为 100 ms。 */
                HAL_UART_Transmit(&huart4, code12_ir_msg, sizeof(code12_ir_msg), 100U);
            }
            /* 锁存本模式已发送状态，并清除另外两个 UART5 命令的历史状态。 */
            code12_ir_send_last = 1U;
            code12_send_last = 0U;
            clear_send_last = 0U;
        }
        /* B=2：A 键发送 clear 命令，D 键发送 12 位 ASCII Code12 数据。 */
        else if(ch7 == 2)
        {
            /* 离开 UART4 模式后允许下次重新进入时再次发送红外帧。 */
            code12_ir_send_last = 0U;

            /* A 键按下沿向 UART5 发送 "clear\n"，持续按住不会重复发送。 */
            if((ch4 == 1) && (clear_send_last == 0U))
            {
                HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&huart5, clear_msg, sizeof(clear_msg) - 1U, 100U);
                printf("QR UART5 CLEAR %s\n", (tx_status == HAL_OK) ? "OK" : "ERR");
            }
            /* 保存 A 键状态，为下一周期上升沿判断做准备。 */
            clear_send_last = (ch4 == 1) ? 1U : 0U;

            /* D 键按下沿复制并发送当前 Code12。 */
            if((ch10 == 1) && (code12_send_last == 0U))
            {
                memcpy(code12, remote.code12, sizeof(code12));

                /* 0～3 数字转换成 ASCII 字符，末尾添加换行形成 13 字节文本帧。 */
                for(uint8_t i = 0U; i < 12U; i++)
                {
                    code12_msg[i] = (uint8_t)('0' + code12[i]);
                }
                code12_msg[12] = '\n';

                /* 发送状态同时通过 printf 输出，便于调试 UART5 通信。 */
                HAL_StatusTypeDef tx_status = HAL_UART_Transmit(&huart5, code12_msg, sizeof(code12_msg), 100U);
                printf("QR UART5 %s: %.*s", (tx_status == HAL_OK) ? "OK" : "ERR", 13, (char *)code12_msg);
            }
            /* 保存 D 键状态，避免一个长按动作产生多帧重复数据。 */
            code12_send_last = (ch10 == 1) ? 1U : 0U;
        }
        else
        {
            /* 不在两种 Code12 通信模式时复位所有发送锁存标志。 */
            code12_send_last = 0U;
            code12_ir_send_last = 0U;
            clear_send_last = 0U;
            /* 放置吸盘组合：A 打开 PE3 吸盘，D 关闭 PE3 吸盘。 */
            if(place_suction_control_selected != 0U)
            {
                if(ch4==1)
                {
                    Arm_Suction_Set(1U);
                }
                if(ch10==1)
                {
                    Arm_Suction_Set(0U);
                }
            }
            /* 非机械臂模式且 E=1：提供通用的吸盘手动控制。 */
            else if((ch7 != 3) && (ch5 == 1))
            {
                if(ch4==1)
                {
                    Arm_Suction_Set(1U);
                }
                if(ch10==1)
                {
                    Arm_Suction_Set(0U);
                }
            }
            /* 非机械臂模式且 E=3：A/D 分别置位和复位辅助输出 PE1。 */
            else if((ch7 != 3) && (ch5 == 3))
            {
                if(ch4==1)
                {
                    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
                }
                if(ch10==1)
                {
                    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
                }
            }
        }
        
        /* 以 20 ms 周期扫描模式和按键，满足人机操作同时降低串口/GPIO负担。 */
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(GPIO_TASK_PERIOD_MS));
    }
}


/* FreeRTOS 栈溢出钩子。
 * configCHECK_FOR_STACK_OVERFLOW=2 时检测到任务栈越界会进入此函数；
 * 关闭中断并永久停机，便于调试器保留故障现场，禁止系统带故障继续运行。
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    /* 参数可用于调试时查看故障任务，本实现不在运行时打印。 */
    (void)xTask;
    (void)pcTaskName;

    /* 进入不可恢复故障状态，避免其他任务或中断继续访问可能已损坏的内存。 */
    taskDISABLE_INTERRUPTS();
    for (;;)
    {
    }
}
