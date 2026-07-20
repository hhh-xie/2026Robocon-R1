# 2026 Robocon R1 控制系统

> 项目状态：稳定版本

本项目是面向 2026 Robocon R1 机器人的嵌入式控制工程，使用 STM32G474VETx 作为主控，基于 STM32 HAL 和 FreeRTOS 实现底盘、机械臂、升降机构、夹爪、电机与遥控器控制。

## 主要功能

- 底盘运动控制
- 三关节机械臂控制与预设路径执行
- 升降机构和夹爪控制
- M3508 电机闭环控制
- DM 电机位置与速度控制
- 遥控器通道解析
- PID 速度环与角度环控制
- FreeRTOS 周期任务调度
- FDCAN 与串口通信

## 硬件平台

| 项目 | 配置 |
| --- | --- |
| 主控 | STM32G474VETx，LQFP100 |
| 电机 | M3508、DM 系列电机 |
| CAN | FDCAN1、FDCAN2、FDCAN3 |
| 串口 | USART1、USART2、USART3、UART4、UART5 |
| 遥控器 | RadioMaster 遥控器与接收模块 |

## 软件环境

- STM32CubeMX
- Keil MDK-ARM
- Keil Assistant for VS Code（可选）
- STM32 HAL
- FreeRTOS
- CMSIS / CMSIS-DSP

## 工程目录

```text
R1/
├── Control/              # 底盘、机械臂、升降和夹爪控制
├── Core/                 # STM32CubeMX 生成的初始化与中断代码
├── Device/               # M3508、DM 电机驱动
├── Drivers/              # STM32 HAL 与 CMSIS
├── FreeRTOS/             # FreeRTOS 内核与移植层
├── MDK-ARM/              # Keil 工程、启动文件和链接脚本
├── User/                 # PID、遥控器解析和 FreeRTOS 任务
├── Tools/RadioMaster/    # RadioMaster 遥控器 Lua 工具
├── tests/                # 主机侧回归测试
├── docs/                 # 硬件资料
├── R1.ioc                # STM32CubeMX 工程
└── .mxproject            # STM32CubeMX 项目信息
```

## FreeRTOS 任务

| 任务 | 周期 | 用途 |
| --- | ---: | --- |
| RemoteTask | 2 ms | 解析遥控器数据 |
| ArmTask | 5 ms | 机械臂手动控制与自动路径更新 |
| M3508PidTask | 1 ms | M3508 电机闭环控制 |
| DebugTask | 500 ms | 周期调试输出 |
| LiftClawTask | 10 ms | 升降机构与夹爪控制 |
| ChassisTask | 5 ms | 底盘运动控制 |
| GpioTask | 20 ms | GPIO 状态控制 |

周期任务使用 `vTaskDelayUntil()` 按固定时间基准运行，以减少任务周期漂移。

## 通信配置

- FDCAN1、FDCAN2、FDCAN3 标称波特率均为 1 Mbit/s。
- USART2 RX 使用 DMA 循环接收。
- USART1、USART2、USART3、UART4 和 UART5 用于不同外设与调试通信。

具体引脚、过滤器和外设参数以 [`R1.ioc`](R1.ioc) 为准。

## 编译与下载

### 使用 Keil MDK

1. 安装 Keil MDK、STM32G4 Device Pack 及工程所需 CMSIS 组件。
2. 打开 `MDK-ARM/R1.uvprojx`。
3. 选择 `R1` Target。
4. 执行 Build/Rebuild。
5. 根据实际调试器和硬件连接配置下载选项，然后烧录程序。

### 使用 STM32CubeMX

使用 STM32CubeMX 打开根目录下的 `R1.ioc`，可以查看或调整时钟、GPIO、FDCAN、UART、DMA 和 FreeRTOS 配置。

重新生成代码前，请确认自定义代码位于 CubeMX 用户代码区域或独立源码目录中。

## RadioMaster 遥控器脚本

遥控器工具位于：

```text
Tools/RadioMaster/code12.lua
```

将其复制到 RadioMaster 遥控器 SD 卡：

```text
/SCRIPTS/TOOLS/code12.lua
```

该工具通过 CH13～CH16 传递 12 位编码数据。按键、开关和通道配置说明请查看 Lua 文件开头的注释。

## 测试

PowerShell 静态回归检查：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\arm_task_checks.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File .\tests\arm_height_paths_test.ps1
```

使用 MinGW GCC 运行夹爪 DM 电机主机侧测试：

```powershell
gcc -std=c11 -Wall -Wextra -Werror .\tests\claw_dm_step_test.c -o .\tests\claw_dm_step_test.exe
.\tests\claw_dm_step_test.exe
```

## 硬件资料

- [STM32G474 控制板原理图](docs/schematic_G474.pdf)

## 安全注意事项

- 上电前确认电源电压、极性和公共地连接正确。
- 使能电机前确认 CAN 总线、电机 ID 和控制方向。
- 首次调试机械臂、升降和夹爪时应限制速度，并预留紧急断电方式。
- 修改 PID、关节目标或自动路径后，应先在无负载或安全支撑条件下验证。
- 运行自动动作前确认机械结构周围没有人员和障碍物。

## 许可证

本项目使用 MIT License，详情参见 [LICENSE](LICENSE)。