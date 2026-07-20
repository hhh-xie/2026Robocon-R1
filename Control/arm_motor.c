#include "arm_motor.h"

/* ============================== 遥控器通道映射 ==============================
 * Elrs.ch[] 中的原始 CRSF 通道已经在 remote_control.c 中完成了摇杆居中、
 * 三段开关离散化和按键 0/1 化。这里用有业务含义的名字代替通道下标，
 * 便于后续阅读机械臂模式、预设动作和示教功能的判定条件。
 */
#define ARM_MODE_VALUE Elrs.ch[7]
#define ARM_TEACH_C_VALUE Elrs.ch[8]
#define ARM_TEACH_F_VALUE Elrs.ch[6]
#define ARM_TEACH_PRINT_KEY Elrs.ch[4]

/* 三段开关和按键在解析后的有效值。机械臂手动模式由 B 通道值 3 选中。 */
#define ARM_MODE_ACTIVE_VALUE 3
#define ARM_TEACH_C_ENABLE_VALUE 1
#define ARM_TEACH_F_ENABLE_VALUE 1
#define ARM_TEACH_KEY_PRESSED 1

/* A 键用于“吸取/执行”，D 键用于“回收/释放”；F、C 三段开关选择动作。 */
#define ARM_PICK_KEY_VALUE Elrs.ch[4]
#define ARM_RECYCLE_KEY_VALUE Elrs.ch[10]
#define ARM_PRESET_F_MODE_VALUE Elrs.ch[6]
#define ARM_PRESET_C_MODE_VALUE Elrs.ch[8]

/* 三段开关经过 Data_Change() 后只会得到 1、2、3。 */
#define ARM_SWITCH_POS_1 1
#define ARM_SWITCH_POS_2 2
#define ARM_SWITCH_POS_3 3

/* 每到达一个路径点后默认停留 50 ms，防止尚未稳定就直接切换下一点。 */
#define ARM_TASK_DEFAULT_HOLD_MS 50U
/* 仅用于静态数组；由编译器根据数组总大小计算路径点数量。 */
#define ARM_ARRAY_SIZE(array) ((uint8_t)(sizeof(array) / sizeof((array)[0])))

/* 自动路径每个 5 ms 控制周期允许三个关节目标值变化的最大步长。 */
#define ARM_PRESET_BASE_STEP 0.008f
#define ARM_PRESET_SHOULDER_STEP 0.008f
#define ARM_PRESET_ELBOW_STEP 0.008f
/* 三个关节与目标的误差都不超过该值时，才认为路径点已经到达。 */
#define ARM_PRESET_REACHED_TOLERANCE 0.001f

/* 一个路径点保存三个关节目标值以及到达后的保持时间。
 * elbow_relative 是肘关节相对肩关节的目标量；真正下发给肘电机的目标值
 * 会在 Arm_Move_To_Path_Point() 中与 shoulder 相加后再进行限幅。
 */
typedef struct
{
    float base;
    float shoulder;
    float elbow_relative;
    uint16_t hold_time_ms;
} Arm_Path_Point_t;

/* 一条自动任务由只读路径点数组及其点数构成。 */
typedef struct
{
    const Arm_Path_Point_t *points;
    uint8_t point_count;
} Arm_Path_Task_t;

/* 路径任务开始或结束时需要联动执行的吸盘动作。 */
typedef enum
{
    /* 只运行路径，不自动改变吸盘。 */
    ARM_TASK_ACTION_NONE = 0,
    /* 任务启动时打开吸盘，用于吸取方块。 */
    ARM_TASK_ACTION_OPEN_SUCTION,
    /* 任务结束时关闭吸盘，用于完成回收/放置。 */
    ARM_TASK_ACTION_CLOSE_SUCTION,
} Arm_Task_Action_t;

/* 当前机械臂的三个绝对关节目标及预留的前馈力矩成员。 */
static Arm_Motor_t arm_motor;
/* 肘关节相对肩关节的独立控制量，用于让摇杆和路径表更直观。 */
static float arm_elbow_relative;

/* 示教打印键的上一周期状态，用于只在按键按下沿打印一次。 */
static uint8_t arm_teach_print_key_last = 0U;
/* 上一次由 F/C 开关选择的任务，用于识别开关是否切换到了新任务。 */
static const Arm_Path_Task_t *arm_last_selected_task = NULL;

/* 以下变量共同构成非阻塞路径任务状态机；任务每 5 ms 推进一步。 */
static const Arm_Path_Task_t *arm_active_task = NULL;
static uint8_t arm_active_task_point_index = 0U;
static uint8_t arm_active_task_hold_active = 0U;
static uint32_t arm_active_task_hold_start = 0U;
static Arm_Task_Action_t arm_active_task_action = ARM_TASK_ACTION_NONE;

/* A、D 两个按键的上一周期状态，用于检测从未按下到按下的上升沿。 */
static uint8_t arm_pick_key_last = 0U;
static uint8_t arm_recycle_key_last = 0U;

/* 记录自动吸盘和高度回位流程是否已启用，供开关任务选择逻辑使用。 */
static uint8_t arm_suction_auto_enabled = 0U;
static uint8_t arm_height_return_enabled = 0U;
/*
   吸取方块：
   {-0.169f, -3.056f, 0.223f, 100};
   {-0.169f, -2.552f, 0.312f, 100};
   {-0.039f, -2.099f, 0.201f, 100};

   {-0.038f, -2.964f, 0.170f, 100};3
   {-0.062f, -2.494f, 0.221f, 100};2
   {-0.095f, -2.020f, 0.145f, 100};1

   回收存储：
   {-2.395f, -1.402f, -0.885f, 100};
   {-3.054f, -1.681f, 0.243f, 100};
   放取方块：
   {0.125f, -1.701f, -1.110f, 100};
   {-0.090f, -1.716f, 0.017f, 100};
   递出方块：
   吸：{-0.052f, -2.845f, 0.236f,100};
   递：{0.012f, -2.415f, -1.477f, 100};
*/
/* 吸取高度 1：先到统一的接近点，再移动到高度 1 的吸取点。 */
static const Arm_Path_Point_t arm_pick_height1_points[] = {
    {-0.095f, -2.020f, 0.145f, ARM_TASK_DEFAULT_HOLD_MS},
    {-0.038f, -2.964f, 0.170f, ARM_TASK_DEFAULT_HOLD_MS},
};
/* 吸取高度 2：先到统一的接近点，再移动到高度 2 的吸取点。 */
static const Arm_Path_Point_t arm_pick_height2_points[] = {
    {-0.095f, -2.020f, 0.145f, ARM_TASK_DEFAULT_HOLD_MS},
    {-0.062f, -2.494f, 0.221f, ARM_TASK_DEFAULT_HOLD_MS},
};
/* 吸取高度 3 的路径；当前两个点数值相同，仍保留两点任务结构。 */
static const Arm_Path_Point_t arm_pick_height3_points[] = {
    {-0.095f, -2.020f, 0.145f, ARM_TASK_DEFAULT_HOLD_MS},
    {-0.095f, -2.020f, 0.145f, ARM_TASK_DEFAULT_HOLD_MS},
};
/* 将已吸取方块送往回收存储位置的两段路径。 */
static const Arm_Path_Point_t arm_recycle_block_points[] = {
    {-2.395f, -1.402f, -0.885f, ARM_TASK_DEFAULT_HOLD_MS},
    {-3.054f, -1.681f, 0.243f, ARM_TASK_DEFAULT_HOLD_MS},
};
/* 将方块移动到放置区域的两段路径。 */
static const Arm_Path_Point_t arm_place_block_points[] = {
    {0.125f, -1.701f, -1.110f, ARM_TASK_DEFAULT_HOLD_MS},
    {-0.090f, -1.716f, 0.017f, ARM_TASK_DEFAULT_HOLD_MS},
};
/* 对接/递出模式下用于吸取方块的单点路径。 */
static const Arm_Path_Point_t arm_transfer_pick_points[] = {
    {-0.052f, -2.845f, 0.236f, ARM_TASK_DEFAULT_HOLD_MS},
};
/* 对接/递出模式下用于把方块递出的单点路径。 */
static const Arm_Path_Point_t arm_transfer_place_points[] = {
    {0.012f, -2.415f, -1.477f, ARM_TASK_DEFAULT_HOLD_MS},
};

/* 将各路径点数组封装为统一任务对象，供后面的选择和状态机逻辑使用。 */
static const Arm_Path_Task_t ARM_TASK_PICK_HEIGHT1 = {
    arm_pick_height1_points,
    ARM_ARRAY_SIZE(arm_pick_height1_points),
};
static const Arm_Path_Task_t ARM_TASK_PICK_HEIGHT2 = {
    arm_pick_height2_points,
    ARM_ARRAY_SIZE(arm_pick_height2_points),
};
static const Arm_Path_Task_t ARM_TASK_PICK_HEIGHT3 = {
    arm_pick_height3_points,
    ARM_ARRAY_SIZE(arm_pick_height3_points),
};
static const Arm_Path_Task_t ARM_TASK_RECYCLE_BLOCK = {
    arm_recycle_block_points,
    ARM_ARRAY_SIZE(arm_recycle_block_points),
};
static const Arm_Path_Task_t ARM_TASK_PLACE_BLOCK = {
    arm_place_block_points,
    ARM_ARRAY_SIZE(arm_place_block_points),
};
static const Arm_Path_Task_t ARM_TASK_TRANSFER_PICK = {
    arm_transfer_pick_points,
    ARM_ARRAY_SIZE(arm_transfer_pick_points),
};
static const Arm_Path_Task_t ARM_TASK_TRANSFER_PLACE = {
    arm_transfer_place_points,
    ARM_ARRAY_SIZE(arm_transfer_place_points),
};


/* 将 value 约束到 [min, max]，所有关节目标更新最终都通过该函数限幅。 */
static float Arm_Limit(float value, float min, float max)
{
    /* 先处理上界再处理下界，保证返回值不会超出机械关节安全范围。 */
    if (value > max) value = max;
    if (value < min) value = min;
    return value;
}

/* 根据摇杆输入平滑更新单个关节目标值。
 * 处理顺序：归一化 -> 死区 -> 死区后的幅值重映射 -> 指数曲线 -> 步长累加 -> 限幅。
 * current 为当前目标位置，joystick 为遥控输入，max_step 限制单周期最大变化量。
 */
static float Arm_Joystick_Update(float current, float joystick, float max_step, float min, float max)
{
    float input;
    float abs_input;
    float curved;

    /* 将实际摇杆输入换算为 -1.0～1.0，异常超范围输入也会被截断。 */
    input = joystick / ARM_JOYSTICK_FULL_SCALE;
    input = Arm_Limit(input, -1.0f, 1.0f);
    abs_input = fabsf(input);

    /* 输入处于死区时保持原目标，避免摇杆中心抖动造成关节缓慢漂移。 */
    if (abs_input <= ARM_JOYSTICK_DEADBAND)
    {
        return current;
    }

    /* 去掉死区占用的幅值，并把剩余行程重新映射为 0～1。 */
    input = (abs_input - ARM_JOYSTICK_DEADBAND) / (1.0f - ARM_JOYSTICK_DEADBAND);
    if (joystick < 0.0f)
    {
        input = -input;
    }

    /* 线性项和三次项混合：小输入更细腻，大输入仍能达到最大步长。 */
    curved = (1.0f - ARM_JOYSTICK_EXPO) * input + ARM_JOYSTICK_EXPO * input * input * input;
    current += curved * max_step;

    return Arm_Limit(current, min, max);
}

/* 让 current 以固定 step 向 target 靠近，用于自动路径的限速位置过渡。
 * 若剩余误差已经小于一步，则直接落到目标，避免在目标两侧来回振荡。
 */
static float Arm_Approach_Target(float current, float target, float step, float min, float max)
{
    float delta = target - current;

    if ((delta <= step) && (delta >= -step))
    {
        return Arm_Limit(target, min, max);
    }

    current += (delta > 0.0f) ? step : -step;
    return Arm_Limit(current, min, max);
}

/* 判断三个关节中是否存在明显的人工摇杆输入。
 * 一旦人工输入有效，自动路径会立即停止，将控制权交还给操作者。
 */
static uint8_t Arm_Joystick_Is_Active(float base, float shoulder, float elbow)
{
    /* 使用与 Arm_Joystick_Update() 相同的满量程和死区定义。 */
    const float active_threshold = ARM_JOYSTICK_FULL_SCALE * ARM_JOYSTICK_DEADBAND;

    if ((fabsf(base) > active_threshold) ||
        (fabsf(shoulder) > active_threshold) ||
        (fabsf(elbow) > active_threshold))
    {
        return 1U;
    }

    return 0U;
}

/* 判断当前三个关节目标是否都已进入指定路径点的到达容差。 */
static uint8_t Arm_Path_Point_Reached(const Arm_Path_Point_t *point)
{
    if ((fabsf(arm_motor.base - point->base) <= ARM_PRESET_REACHED_TOLERANCE) &&
        (fabsf(arm_motor.shoulder - point->shoulder) <= ARM_PRESET_REACHED_TOLERANCE) &&
        (fabsf(arm_elbow_relative - point->elbow_relative) <= ARM_PRESET_REACHED_TOLERANCE))
    {
        return 1U;
    }

    return 0U;
}

/* 对旧的整数遥控接口应用中心死区；该接口与当前浮点角度接口并存。 */
static int16_t Arm_Dead_Zone(int16_t value)
{
    if(value < ARM_RC_DEAD_ZONE && value > -ARM_RC_DEAD_ZONE)
    {
        value = 0;
    }
    return value;
}

/* CMSIS-DSP 平方根函数的安全封装；运算失败时返回 0，避免未定义结果继续传播。 */
static float Arm_Sqrt(float value)
{
    float result = 0.0f;

    if (arm_sqrt_f32(value, &result) != ARM_MATH_SUCCESS)
    {
        result = 0.0f;
    }

    return result;
}

/* CMSIS-DSP atan2 的安全封装；用于计算底座角和逆运动学中的肩关节角。 */
static float Arm_Atan2(float y, float x)
{
    float result = 0.0f;

    if (arm_atan2_f32(y, x, &result) != ARM_MATH_SUCCESS)
    {
        result = 0.0f;
    }

    return result;
}

/* 以固定三位小数打印路径参数，并在数字末尾输出 f，格式可直接复制到路径表。
 * 这里不使用浮点 printf，从而避免嵌入式工程额外启用较大的浮点格式化支持。
 */
static void Arm_Print_Path_Value(float value)//把一个浮点数转换成“千分之一单位”的整数，也就是放大 1000 倍后四舍五入
{
    /* 正数加 0.5、负数减 0.5 后转整数，实现对称的四舍五入。 */
    int32_t milli_value = (int32_t)(value * 1000.0f + ((value >= 0.0f) ? 0.5f : -0.5f));

    if (milli_value < 0)
    {
        printf("-");
        milli_value = -milli_value;
    }

    printf("%ld.%03ldf",
           (long)(milli_value / 1000),
           (long)(milli_value % 1000));
}

/* 示教模式路径点打印。
 * 仅当 B=3、C=1、F=1 且 A 键出现按下沿时，将当前三个目标值按路径表格式输出。
 */
static void Arm_Teach_Point_Print(void)
{
    uint8_t print_key_pressed = 0U;

    /* 先确认当前组合确实是示教模式，防止 A 键在其他功能中触发打印。 */
    if ((ARM_MODE_VALUE == ARM_MODE_ACTIVE_VALUE) &&
        (ARM_TEACH_C_VALUE == ARM_TEACH_C_ENABLE_VALUE) &&
        (ARM_TEACH_F_VALUE == ARM_TEACH_F_ENABLE_VALUE))
    {
        if (ARM_TEACH_PRINT_KEY == ARM_TEACH_KEY_PRESSED)
        {
            print_key_pressed = 1U;
        }

        /* 上升沿检测：持续按住 A 键时只打印一次，不会每 5 ms 连续刷屏。 */
        if ((print_key_pressed != 0U) && (arm_teach_print_key_last == 0U))
        {
            printf("// F:%d C:%d\r\n",
                   ARM_PRESET_F_MODE_VALUE,
                   ARM_PRESET_C_MODE_VALUE);
            printf("{");
            Arm_Print_Path_Value(arm_motor.base);
            printf(", ");
            Arm_Print_Path_Value(arm_motor.shoulder);
            printf(", ");
            Arm_Print_Path_Value(arm_elbow_relative);
            printf(", %u},\r\n", (unsigned int)ARM_TASK_DEFAULT_HOLD_MS);
        }

        /* 保存本周期键值，供下一周期判断是否出现新的按下动作。 */
        arm_teach_print_key_last = print_key_pressed;
    }
    else
    {
        arm_teach_print_key_last = 0U;
    }
}

/* 将三个关节目标以各自固定步长向当前路径点推进。
 * 路径表保存的是肘相对量，因此最后需叠加肩关节目标得到肘电机绝对目标。
 */
static void Arm_Move_To_Path_Point(const Arm_Path_Point_t *point)
{
    arm_motor.base = Arm_Approach_Target(arm_motor.base,
                                         point->base,
                                         ARM_PRESET_BASE_STEP,
                                         ARM_BASE_MIN,
                                         ARM_BASE_MAX);
    arm_motor.shoulder = Arm_Approach_Target(arm_motor.shoulder,
                                             point->shoulder,
                                             ARM_PRESET_SHOULDER_STEP,
                                             ARM_SHOULDER_MIN,
                                             ARM_SHOULDER_MAX);
    arm_elbow_relative = Arm_Approach_Target(arm_elbow_relative,
                                             point->elbow_relative,
                                             ARM_PRESET_ELBOW_STEP,
                                             ARM_ELBOW_MIN,
                                             ARM_ELBOW_MAX);
    /* 肘电机指令采用绝对目标：肘相对量 + 当前肩关节目标。 */
    arm_motor.elbow = Arm_Limit(arm_elbow_relative + arm_motor.shoulder,
                                ARM_ELBOW_MIN,
                                ARM_ELBOW_MAX);
}

/* 根据 F、C 两个三段开关返回普通预设任务。
 * F=3 时 C=1/2/3 分别选择高度 3/2/1；F=2、C=3 选择放置路径。
 */
static const Arm_Path_Task_t *Arm_Get_Selected_Path_Task(void)
{
    if (ARM_PRESET_F_MODE_VALUE == ARM_SWITCH_POS_3)
    {
        if (ARM_PRESET_C_MODE_VALUE == ARM_SWITCH_POS_1)
        {
            return &ARM_TASK_PICK_HEIGHT3;
        }
        if (ARM_PRESET_C_MODE_VALUE == ARM_SWITCH_POS_2)
        {
            return &ARM_TASK_PICK_HEIGHT2;
        }
        if (ARM_PRESET_C_MODE_VALUE == ARM_SWITCH_POS_3)
        {
            return &ARM_TASK_PICK_HEIGHT1;
        }
    }
    if (ARM_PRESET_F_MODE_VALUE == ARM_SWITCH_POS_2)
    {
        if (ARM_PRESET_C_MODE_VALUE == ARM_SWITCH_POS_3)
        {
            return &ARM_TASK_PLACE_BLOCK;
        }
    }

    /* 当前开关组合没有定义自动路径。 */
    return NULL;
}

/* 只允许 F=3 的三个高度组合成为“吸取任务”，过滤掉放置任务。 */
static const Arm_Path_Task_t *Arm_Get_Selected_Pick_Task(void)
{
    if (ARM_PRESET_F_MODE_VALUE != ARM_SWITCH_POS_3)
    {
        return NULL;
    }

    return Arm_Get_Selected_Path_Task();
}

/* 判断当前 D 键命令是否从某个有效吸取高度发起，以决定是否允许自动回收。 */
static uint8_t Arm_Recycle_Command_From_Pick_Position(void)
{
    if ((ARM_PRESET_F_MODE_VALUE == ARM_SWITCH_POS_3) &&
        (Arm_Get_Selected_Pick_Task() != NULL))
    {
        return 1U;
    }

    return 0U;
}

/* 有效吸取高度下按 D 键时，返回统一的方块回收路径。 */
static const Arm_Path_Task_t *Arm_Get_Selected_Recycle_Task(void)
{
    if (Arm_Recycle_Command_From_Pick_Position() != 0U)
    {
        return &ARM_TASK_RECYCLE_BLOCK;
    }

    return NULL;
}

/* 回收任务启动后，记录回收前所选的吸取高度；否则以回收任务本身作为记录值。 */
static const Arm_Path_Task_t *Arm_Get_Recycle_Last_Selected_Task(void)
{
    if (Arm_Recycle_Command_From_Pick_Position() != 0U)
    {
        return Arm_Get_Selected_Pick_Task();
    }

    return &ARM_TASK_RECYCLE_BLOCK;
}

/* F=2、C=3 为放置模式；该模式下 A/D 键直接控制吸盘开关。 */
static uint8_t Arm_Place_Mode_Selected(void)
{
    if ((ARM_PRESET_F_MODE_VALUE == ARM_SWITCH_POS_2) &&
        (ARM_PRESET_C_MODE_VALUE == ARM_SWITCH_POS_3))
    {
        return 1U;
    }

    return 0U;
}

/* F=2、C=2 为递出/对接模式；A、D 键分别启动吸取和递出单点路径。 */
static uint8_t Arm_Transfer_Mode_Selected(void)
{
    if ((ARM_PRESET_F_MODE_VALUE == ARM_SWITCH_POS_2) &&
        (ARM_PRESET_C_MODE_VALUE == ARM_SWITCH_POS_2))
    {
        return 1U;
    }

    return 0U;
}

/* 返回由三段开关自动切换触发的任务。
 * 吸盘自动流程或高度回位流程有效时，F=3 可随 C 开关切换吸取高度；
 * F=2、C=3 则切换到方块放置路径。
 */
static const Arm_Path_Task_t *Arm_Get_Selected_Switch_Task(void)
{
    /* 已吸取方块或正在执行高度回位时，允许 C 开关直接切换目标高度。 */
    if (((arm_suction_auto_enabled != 0U) ||
         (arm_height_return_enabled != 0U)) &&
        (ARM_PRESET_F_MODE_VALUE == ARM_SWITCH_POS_3))
    {
        return Arm_Get_Selected_Pick_Task();
    }

    /* 放置组合不要求自动标志，可直接返回放置路径。 */
    if ((ARM_PRESET_F_MODE_VALUE == ARM_SWITCH_POS_2) &&
        (ARM_PRESET_C_MODE_VALUE == ARM_SWITCH_POS_3))
    {
        return &ARM_TASK_PLACE_BLOCK;
    }

    return NULL;
}

/* remote_control.c 已将 A、D 按键优化成 0/1，这里统一转换为 uint8_t 布尔值。 */
static uint8_t Arm_Key_Is_Pressed(int16_t key_value)
{
    return (key_value == 1) ? 1U : 0U;
}

/* 进入机械臂模式时同步按键历史值，避免已经按住的键被误判为新按下。 */
static void Arm_Update_Key_Last(void)
{
    arm_pick_key_last = Arm_Key_Is_Pressed(ARM_PICK_KEY_VALUE);
    arm_recycle_key_last = Arm_Key_Is_Pressed(ARM_RECYCLE_KEY_VALUE);
}

/* 清空当前路径任务的全部运行状态。
 * 该函数只停止自动路径，不会停止达妙电机，也不会主动改变当前关节目标。
 */
static void Arm_Stop_Path_Task(void)
{
    arm_active_task = NULL;
    arm_active_task_point_index = 0U;
    arm_active_task_hold_active = 0U;
    arm_active_task_hold_start = 0U;
    arm_active_task_action = ARM_TASK_ACTION_NONE;
}

/* 控制 PE3 吸盘输出，并同步记录自动吸盘状态。
 * enable 非零时统一归一化为 1；PE3 置位表示打开吸盘，复位表示关闭吸盘。
 */
void Arm_Suction_Set(uint8_t enable)
{
    enable = (enable != 0U) ? 1U : 0U;
    HAL_GPIO_WritePin(GPIOE,
                      GPIO_PIN_3,
                      (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    arm_suction_auto_enabled = enable;
}

/* 当前路径的所有点运行完毕后执行收尾动作，再彻底退出任务状态机。 */
static void Arm_Finish_Active_Path_Task(void)
{
    /* CLOSE_SUCTION 动作被设计为“任务结束后释放/关闭”，因此在此处执行。 */
    if (arm_active_task_action == ARM_TASK_ACTION_CLOSE_SUCTION)
    {
        Arm_Suction_Set(0U);
    }

    Arm_Stop_Path_Task();
}

/* 从第 0 个路径点开始一个新的非阻塞任务，并保存任务联动动作。 */
static void Arm_Start_Path_Task(const Arm_Path_Task_t *task, Arm_Task_Action_t action)
{
    /* 空任务或零路径点任务无法执行，统一按停止任务处理。 */
    if ((task == NULL) || (task->point_count == 0U))
    {
        Arm_Stop_Path_Task();
        return;
    }

    /* 每次启动都重置点索引和到达保持计时，避免继承上一个任务的进度。 */
    arm_active_task = task;
    arm_active_task_point_index = 0U;
    arm_active_task_hold_active = 0U;
    arm_active_task_hold_start = 0U;
    arm_active_task_action = action;

    /* OPEN_SUCTION 在移动开始前执行，使机械臂到达吸取位置时吸盘已工作。 */
    if (action == ARM_TASK_ACTION_OPEN_SUCTION)
    {
        Arm_Suction_Set(1U);
    }
}

/* 推进一次当前路径任务；本函数不阻塞，依靠 ArmTask 的 5 ms 周期反复调用。
 * 每个路径点经历“逐步接近 -> 判定到达 -> 保持计时 -> 切换下一点”四个阶段。
 */
static void Arm_Run_Active_Path_Task(void)
{
    const Arm_Path_Point_t *point;
    uint32_t now;

    /* 没有活动任务时不做任何目标更新。 */
    if (arm_active_task == NULL)
    {
        return;
    }

    /* 防御性检查，防止异常索引访问路径点数组之外的内存。 */
    if (arm_active_task_point_index >= arm_active_task->point_count)
    {
        Arm_Stop_Path_Task();
        return;
    }

    /* 取出当前点，并让三个关节目标各自向该点靠近一步。 */
    point = &arm_active_task->points[arm_active_task_point_index];
    Arm_Move_To_Path_Point(point);

    /* 尚未到达时取消保持状态；下次周期继续向当前点移动。 */
    if (Arm_Path_Point_Reached(point) == 0U)
    {
        arm_active_task_hold_active = 0U;
        return;
    }

    /* 首次进入到达容差时记录保持起点；后续周期只检查经过时间。 */
    now = HAL_GetTick();
    if (arm_active_task_hold_active == 0U)
    {
        arm_active_task_hold_active = 1U;
        arm_active_task_hold_start = now;
    }

    /* uint32_t 减法可自然兼容 HAL tick 计数器回绕。 */
    if ((uint32_t)(now - arm_active_task_hold_start) < point->hold_time_ms)
    {
        return;
    }

    /* 保持时间结束后进入下一点；最后一点结束则执行任务收尾。 */
    arm_active_task_hold_active = 0U;
    arm_active_task_point_index++;
    if (arm_active_task_point_index >= arm_active_task->point_count)
    {
        Arm_Finish_Active_Path_Task();
    }
}

/* 依次给底座、肩部、肘部三个达妙电机发送 MIT 模式启动命令。 */
void Arm_Motor_Start(void)
{
    DM_Start(DM_Mode_MIT, DM4340p1_ID);
    DM_Start(DM_Mode_MIT, DM10010L_ID);
    DM_Start(DM_Mode_MIT, DM4340p2_ID);
}

/* 依次给三个机械臂达妙电机发送 MIT 模式停止命令。 */
void Arm_Motor_Stop(void)
{
    DM_Stop(DM_Mode_MIT, DM4340p1_ID);
    DM_Stop(DM_Mode_MIT, DM10010L_ID);
    DM_Stop(DM_Mode_MIT, DM4340p2_ID);
}

/* 从非机械臂模式切入 B=3 的机械臂模式时调用一次。
 * 主要作用是同步开关/按键历史状态，避免模式切换瞬间误启动预设任务。
 */
void Arm_Mode_Enter(void)
{
    /* 记住切入时的开关任务，只有后续选择发生变化才按新任务处理。 */
    arm_last_selected_task = Arm_Get_Selected_Switch_Task();
    /* 正在运行的任务需要保留；没有任务时则确保所有子状态均已归零。 */
    if (arm_active_task == NULL)
    {
        Arm_Stop_Path_Task();
    }
    arm_teach_print_key_last = 0U;
    Arm_Update_Key_Last();
}

/* B 通道离开机械臂手动模式后仍可继续推进已经启动的自动路径。
 * 只有存在活动任务时才计算新目标并下发电机指令。
 */
void Arm_Auto_Task_Update(void)
{
    if (arm_active_task != NULL)
    {
        Arm_Run_Active_Path_Task();
        Arm_Motor_Control();
    }
}

/* 将全部软件目标和路径状态复位到零点，然后启动三个达妙电机。
 * 当前工程 main/StartTask 中未直接调用本函数，保留为完整初始化接口。
 */
void Arm_Motor_Init(void)
{
    /* 三个绝对关节目标从 0 rad 开始。 */
    arm_motor.base = 0.0f;
    arm_motor.shoulder = 0.0f;
    arm_motor.elbow = 0.0f;
    arm_elbow_relative = 0.0f;
    /* 前馈力矩成员目前未用于 Arm_Motor_Control()，初始化时仍明确清零。 */
    arm_motor.shoulder_tff = 0.0f;
    arm_motor.elbow_tff = 0.0f;
    arm_last_selected_task = NULL;
    arm_pick_key_last = 0U;
    arm_recycle_key_last = 0U;
    arm_suction_auto_enabled = 0U;
    arm_height_return_enabled = 0U;
    /* 清除可能残留的路径指针、点索引、保持计时和联动动作。 */
    Arm_Stop_Path_Task();

    /* 软件状态准备完毕后，使能三个达妙电机。 */
    Arm_Motor_Start();
}

/* 旧版整数遥控控制接口。
 * x/y/z 分别直接增量控制底座、肩部、肘部绝对目标，随后限幅并立即下发。
 */
void Arm_RC_Control(int16_t x, int16_t y, int16_t z)
{
    /* 先滤除遥控器中心附近的小幅噪声。 */
    x = Arm_Dead_Zone(x);
    y = Arm_Dead_Zone(y);
    z = Arm_Dead_Zone(z);

    /* 将整数摇杆量乘固定比例，转换成本周期的关节目标增量。 */
    arm_motor.base += x * ARM_RC_STEP;
    arm_motor.shoulder += y * ARM_RC_STEP;
    arm_motor.elbow += z * ARM_RC_STEP;

    /* 所有绝对关节目标限制在 arm_motor.h 定义的机械安全范围内。 */
    arm_motor.base = Arm_Limit(arm_motor.base, ARM_BASE_MIN, ARM_BASE_MAX);
    arm_motor.shoulder = Arm_Limit(arm_motor.shoulder, ARM_SHOULDER_MIN, ARM_SHOULDER_MAX);
    arm_motor.elbow = Arm_Limit(arm_motor.elbow, ARM_ELBOW_MIN, ARM_ELBOW_MAX);
    /* 同步肘相对量，使后续切换到浮点摇杆或路径控制时目标连续。 */
    arm_elbow_relative = Arm_Limit(arm_motor.elbow - arm_motor.shoulder, ARM_ELBOW_MIN, ARM_ELBOW_MAX);

    /* 更新完软件目标后立即发送三个 MIT 控制帧。 */
    Arm_Motor_Control();
}

/* 从全局遥控结构读取右 X、右 Y、左 Y 三个通道，调用旧版整数控制接口。 */
void Arm_RC_Control_From_Remote(void)
{
    Arm_RC_Control(Elrs.ch[0], Elrs.ch[1], Elrs.ch[2]);
}

/* 机械臂主控制入口，由 FreeRTOS 的 ArmTask 每 5 ms 调用一次。
 * 三个参数已在 freertos_start.c 中按 /80.0f 缩放并调整方向。
 * 控制优先级为：人工摇杆取消自动任务 > 按键启动任务 > 开关切换任务 >
 * 运行活动路径；没有活动路径时才执行普通摇杆增量控制。
 */
void Arm_Angle_Control(float dm4340p1_angle, float dm10010l_angle, float dm4340p2_angle)
{
    /* 本周期可能用到的普通路径、吸取路径和回收路径选择结果。 */
    const Arm_Path_Task_t *selected_task;
    const Arm_Path_Task_t *selected_pick_task;
    const Arm_Path_Task_t *selected_recycle_task;
    /* 提前采样摇杆和按键，确保本周期所有分支使用同一组输入状态。 */
    uint8_t joystick_active = Arm_Joystick_Is_Active(dm4340p1_angle, dm10010l_angle, dm4340p2_angle);
    uint8_t pick_key_pressed = Arm_Key_Is_Pressed(ARM_PICK_KEY_VALUE);
    uint8_t recycle_key_pressed = Arm_Key_Is_Pressed(ARM_RECYCLE_KEY_VALUE);
    uint8_t command_started = 0U;

    /* 人工摇杆拥有最高优先级：有明显输入时立即退出任何自动路径。 */
    if (joystick_active != 0U)
    {
        Arm_Stop_Path_Task();
    }
    else
    {
        /* 摇杆空闲时才根据当前 F/C 开关组合计算可启动的自动任务。 */
        selected_pick_task = Arm_Get_Selected_Pick_Task();
        selected_recycle_task = Arm_Get_Selected_Recycle_Task();

        /* 递出/对接模式：A 键启动吸取单点并打开吸盘，D 键启动递出单点。 */
        if (Arm_Transfer_Mode_Selected() != 0U)
        {
            /* A 键按下沿触发，持续按住不会重复重启任务。 */
            if ((pick_key_pressed != 0U) &&
                (arm_pick_key_last == 0U))
            {
                arm_last_selected_task = &ARM_TASK_TRANSFER_PICK;
                Arm_Start_Path_Task(&ARM_TASK_TRANSFER_PICK, ARM_TASK_ACTION_OPEN_SUCTION);
                command_started = 1U;
            }

            /* 同一周期只允许启动一个命令；A、D 同时按下时 A 键优先。 */
            if ((command_started == 0U) &&
                (recycle_key_pressed != 0U) &&
                (arm_recycle_key_last == 0U))
            {
                arm_last_selected_task = &ARM_TASK_TRANSFER_PLACE;
                Arm_Start_Path_Task(&ARM_TASK_TRANSFER_PLACE, ARM_TASK_ACTION_NONE);
                command_started = 1U;
            }
        }

        /* 放置模式：本分支由 A、D 键直接控制吸盘；路径仍由后面的开关逻辑选择。 */
        if (Arm_Place_Mode_Selected() != 0U)
        {
            if (pick_key_pressed != 0U)
            {
                Arm_Suction_Set(1U);
            }
            if (recycle_key_pressed != 0U)
            {
                Arm_Suction_Set(0U);
            }
        }

        /* 普通吸取模式：A 键按下沿启动当前高度路径，并在移动前打开吸盘。 */
        if ((pick_key_pressed != 0U) &&
            (arm_pick_key_last == 0U) &&
            (selected_pick_task != NULL))
        {
            arm_last_selected_task = selected_pick_task;
            Arm_Start_Path_Task(selected_pick_task, ARM_TASK_ACTION_OPEN_SUCTION);
            command_started = 1U;
        }

        /* 普通回收模式：D 键按下沿启动回收路径，结束后关闭吸盘。 */
        if ((command_started == 0U) &&
            (recycle_key_pressed != 0U) &&
            (arm_recycle_key_last == 0U) &&
            (selected_recycle_task != NULL))
        {
            /* 保存回收前的高度任务，避免后续开关判断把同一选择误当成新任务。 */
            arm_last_selected_task = Arm_Get_Recycle_Last_Selected_Task();
            if (Arm_Recycle_Command_From_Pick_Position() != 0U)
            {
                /* 标记回收来自吸取高度，使后续 F=3 时允许按 C 开关返回相应高度。 */
                arm_height_return_enabled = 1U;
            }
            Arm_Start_Path_Task(selected_recycle_task, ARM_TASK_ACTION_CLOSE_SUCTION);
            command_started = 1U;
        }

        /* 本周期没有按键命令时，再检查 F/C 开关是否切换到了另一条路径。 */
        if (command_started == 0U)
        {
            selected_task = Arm_Get_Selected_Switch_Task();
            /* 比较任务对象指针即可判断选择是否改变，避免每周期重复启动同一任务。 */
            if (selected_task != arm_last_selected_task)
            {
                arm_last_selected_task = selected_task;
                if (selected_task != NULL)
                {
                    Arm_Start_Path_Task(selected_task, ARM_TASK_ACTION_NONE);
                }
            }
        }
    }

    /* 有活动任务时由路径状态机生成目标；否则由三个摇杆分别更新目标。 */
    if (arm_active_task != NULL)
    {
        Arm_Run_Active_Path_Task();
    }
    else
    {
        /* 底座和肩部直接更新绝对目标，肘部先更新相对量再叠加肩部目标。 */
        arm_motor.base = Arm_Joystick_Update(arm_motor.base, dm4340p1_angle, ARM_BASE_MAX_STEP, ARM_BASE_MIN, ARM_BASE_MAX);
        arm_motor.shoulder = Arm_Joystick_Update(arm_motor.shoulder, dm10010l_angle, ARM_SHOULDER_MAX_STEP, ARM_SHOULDER_MIN, ARM_SHOULDER_MAX);
        arm_elbow_relative = Arm_Joystick_Update(arm_elbow_relative, dm4340p2_angle, ARM_ELBOW_MAX_STEP, ARM_ELBOW_MIN, ARM_ELBOW_MAX);
        arm_motor.elbow = Arm_Limit(arm_elbow_relative + arm_motor.shoulder, ARM_ELBOW_MIN, ARM_ELBOW_MAX);
    }
    /* 无论目标来自路径还是摇杆，本周期最后都统一发送三个电机控制帧。 */
    Arm_Motor_Control();
    /* 示教模式下检查是否需要输出当前路径点。 */
    Arm_Teach_Point_Print();
    /* 保存按键状态，供下一周期进行上升沿判断。 */
    arm_pick_key_last = pick_key_pressed;
    arm_recycle_key_last = recycle_key_pressed;
}

/* 根据末端目标笛卡尔坐标 (x, y, z) 计算三个关节角并立即下发。
 * 模型为底座绕 Z 轴旋转、肩肘组成平面二连杆；长度由 ARM_LINK1/2 定义。
 */
void Arm_Move_To_Position(float x, float y, float z)
{
    float r;
    float h;
    float d;
    float elbow;
    float shoulder;

    /* 底座角由目标点在 XY 平面中的方位角直接得到。 */
    arm_motor.base = Arm_Atan2(y, x);

    /* r 为目标到 Z 轴的水平距离，h 为相对机械臂基座高度。 */
    r = Arm_Sqrt(x * x + y * y);
    h = z - ARM_BASE_HEIGHT;

    /* 余弦定理解算肘角的 cos 值；限幅到 [-1,1] 防止浮点误差导致 acosf 非法。 */
    d = (r * r + h * h - ARM_LINK1 * ARM_LINK1 - ARM_LINK2 * ARM_LINK2) / (2.0f * ARM_LINK1 * ARM_LINK2);
    d = Arm_Limit(d, -1.0f, 1.0f);

    /* ARM_IK_ELBOW_DIR 决定采用抬肘或垂肘构型；再根据二连杆几何求肩角。 */
    elbow = ARM_IK_ELBOW_DIR * acosf(d);
    shoulder = Arm_Atan2(h, r) - Arm_Atan2(ARM_LINK2 * arm_sin_f32(elbow), ARM_LINK1 + ARM_LINK2 * arm_cos_f32(elbow));

    /* 将逆解结果限制到关节范围，并同步肘相对量以保持不同控制接口连续。 */
    arm_motor.base = Arm_Limit(arm_motor.base, ARM_BASE_MIN, ARM_BASE_MAX);
    arm_motor.shoulder = Arm_Limit(shoulder, ARM_SHOULDER_MIN, ARM_SHOULDER_MAX);
    arm_motor.elbow = Arm_Limit(elbow, ARM_ELBOW_MIN, ARM_ELBOW_MAX);
    arm_elbow_relative = Arm_Limit(arm_motor.elbow - arm_motor.shoulder, ARM_ELBOW_MIN, ARM_ELBOW_MAX);

    /* 逆解完成后立即发送目标。 */
    Arm_Motor_Control();
}


/* 将当前三个关节目标封装为达妙 MIT 控制帧并通过 FDCAN2 发送。
 * 参数顺序为：位置、速度、Kp、Kd、前馈力矩以及各字段的编码范围。
 */
void Arm_Motor_Control(void)
{
    /* 底座：DM4340p1，目标速度固定 1 rad/s，前馈力矩当前为 0。 */
    CAN2_Motor_DM_MIT(DM4340p1_ID,
                      arm_motor.base,
                      1.0f,
                      ARM_BASE_KP,
                      ARM_BASE_KD,
                      0.0f,
                      DM4340_P_MIN, DM4340_P_MAX,
                      DM4340_V_MIN, DM4340_V_MAX,
                      DM4340_T_MIN, DM4340_T_MAX);

    /* 肩部：DM10010L，使用更高的 Kp/Kd 以支撑主要负载。 */
    CAN2_Motor_DM_MIT(DM10010L_ID,
                      arm_motor.shoulder,
                      1.0f,
                      ARM_SHOULDER_KP,
                      ARM_SHOULDER_KD,
					  0.0f,
                      DM10010_P_MIN, DM10010_P_MAX,
                      DM10010_V_MIN, DM10010_V_MAX,
                      DM10010_T_MIN, DM10010_T_MAX);

    /* 肘部：DM4340p2，目标值已经包含肩部角与肘相对量的叠加。 */
    CAN2_Motor_DM_MIT(DM4340p2_ID,
                      arm_motor.elbow,
                      1.0f,
                      ARM_ELBOW_KP,
                      ARM_ELBOW_KD,
					  0.0f,
                      DM4340_P_MIN, DM4340_P_MAX,
                      DM4340_V_MIN, DM4340_V_MAX,
                      DM4340_T_MIN, DM4340_T_MAX);
}
