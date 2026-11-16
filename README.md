# 2026 Robocon R1 控制系统

> 项目状态：结束版本

本项目是面向 2026 Robocon R1 机器人的嵌入式控制工程，使用 STM32G474VETx 作为主控，基于 STM32 HAL 和 FreeRTOS 实现底盘、机械臂、升降机构、夹爪、电机与遥控器控制，本项目没有提供ESP32屏幕端的代码

## 主要功能

- 摇杆控制麦轮底盘vx、vy、vw
- 对3508电机的pid速度环和位置环控制
- 调试功能能单边升降平台、打印机械臂当前点位
- 接收遥控器的12位数据再发送到esp32端显示对应的二维码
- 半自动实现夹取到武器杆到对接的状态转换
- 半自动实现机械臂对三个高度的KFS的吸取和回收
- 半自动实现机械臂放置一层高度以及拾取地上KFS

## 工程总览

| 项目   | 配置                                         |
| ---- | ------------------------------------------ |
| 主控   | STM32G474VETx，LQFP100，Cortex-M4F，硬件单精度 FPU |
| 电机   | M3508、DM4310、DM4340P、DM10010L              |
| CAN  | FDCAN1、FDCAN2、FDCAN3（Chassic CAN-1M）       |
| 串口   | USART1、USART2、USART3、UART4、UART5           |
| 遥控器  | RadioMaster 遥控器与接收模块                       |
| 开发环境 | STM32CubeMX+Keil Assistant for VS Code     |

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

| 任务           | 优先级 |     周期 | 用途                      |
| ------------ | --- | -----: | ----------------------- |
| RemoteTask   | 4   | 通知唤醒 | 环形区取数、CRSF 组帧并发布最新状态队列 |
| ArmTask      | 6   |   5 ms | 机械臂手动控制与自动路径更新          |
| M3508PidTask | 7   |   1 ms | M3508 电机闭环控制            |
| DebugTask    | 1   | 500 ms | 周期调试输出，用于临时调试查看某个值      |
| LiftClawTask | 4   |  10 ms | 升降机构与夹爪控制               |
| ChassisTask  | 5   |   5 ms | 底盘运动控制，遥控器无值时底盘死锁       |
| GpioTask     | 2   |  20 ms | GPIO 状态控制吸盘、电磁阀，12位数字发送 |

工程采用了“目标生成”和“高频执行”分层（有利于降低控制延时导致的控制速度跟不上解算速度的问题，在第一版机器的四舵轮底盘解算中尤为明显）：

```text
遥控 DMA 回调
    ↓ 软件环形缓冲区 + 任务通知
RemoteTask
    ↓ 组帧、CRC、解包并发布最新状态队列
底盘/机械臂/升降夹爪任务
    ↓ 生成速度或位置目标
M3508PidTask（1 kHz）
    ↓ PID计算和CAN电流指令
电机
```

除事件驱动的 RemoteTask 外，所有周期任务均使用 `vTaskDelayUntil()`，相比普通 `vTaskDelay()`，它按照绝对时间基准唤醒，可减少任务执行时间造成的周期漂移

注意：`GpioTask` 与 Timer Service Task 同为优先级2，并且 `GpioTask` 内使用最长100 ms超时的阻塞式串口发送，当前没有软件定时器，所以没有实际冲突；以后如果增加高实时性软件定时器，建议将串口改为 DMA/中断发送或重新调整优先级
## 通信配置

|   总线   |                       设备                        | 引脚Tx/H\|Rx/L |
| :----: | :---------------------------------------------: | :----------: |
| FDCAN1 |              麦轮底盘 4 个 M3508，ID1~4               |   PD1、PD0    |
| FDCAN2 |      机械臂达妙电机：DM4340底座、DM10010L肩部、DM4340肘部       |  PB13、PB12   |
| FDCAN3 | 升降 M3508 ID5/ID6、夹爪纵向旋转 M3508 ID7、夹爪平面旋转 DM4310 |   PA15、PA8   |
| USART1 |                printf调试口（115200）                |   PA9、PA10   |
| USART2 |         ELRS/CRSF 遥控接收机，DMA 接收（420000）          |   PA2、PA3    |
| USART3 |                距离传感器（未用）（115200）                |  PB10、PE15   |
| UART4  |              红外发送，只发送（未用）（115200）               |  PC10、PC11   |
| UART5  |        发送 `clear\n` 和 Code12 文本（115200）         |   PC12、PD2   |

- 这记录传感器是为了放置升降平台上升或者下降超出范围导致机器损坏，但是后来发现传感器数据容易飘动实际并未使用

- 红外传感器是由于规则限制后续并未使用，但是实测效果非常快速且精准，且不需要过多的解析和验证，逻辑简单，不过正确性并未验证

- 具体引脚、过滤器和外设参数请参考 [`R1.ioc`](R1.ioc) 为准


## 遥控器接收架构

遥控接收采用“循环 DMA → 软件环形缓冲区 → 任务组帧与解析 → 最新状态队列”的结构，不使用 Stream Buffer。

```text
USART2（420000，8N1）
    ↓ DMA 持续接收，128 字节循环区
IDLE / HT / TC 中断
    ↓ 根据当前 DMA 写位置，仅转存新增字节
软件环形缓冲区（512 字节，保留一个空位）
    ↓ vTaskNotifyGiveFromISR()
RemoteTask
    ↓ 寻找帧头、按长度组帧、CRC8 校验、16 通道解包和 Code12 处理
长度为 1 的遥控状态队列（xQueueOverwrite）
    ↓ RC_GetSnapshot() 内部使用 xQueuePeek，不取走数据
底盘 / 机械臂 / 升降 / 夹爪 / GPIO 任务
```

- `RemoteTask` 优先级保持 4，栈增加到 512 个 StackType_t。通过 `ulTaskNotifyTake()` 阻塞等待；10 ms 超时用于接收启动失败后的重试，不代表解析周期。
- 中断只转存数据并通知任务，不组帧、不计算 CRC，也不重新启动正常运行的循环 DMA。保留 HT/TC，使持续输入没有 IDLE 时仍有数据处理机会。
- USART2 和 DMA1_Channel1 抢占优先级均为 5，与 FreeRTOS 的中断 API 限制一致，同时避免两个接收回调互相抢占。
- 组帧状态跨多次通知保留，可以处理半帧和连续多帧；长度错误或 CRC 错误后逐字节重新同步帧头。
- 队列发布的是通过 CRC 的**解析后状态**，包含通道、Code12、`frame_seq` 和 `last_valid_tick`。各消费者每轮复制一份完整快照；`Elrs` 仅保留为调试镜像。
- 队列只保存最新状态，多个任务必须使用 `RC_GetSnapshot()` / `xQueuePeek()`，不能竞争调用 `xQueueReceive()`。它不是按键事件队列，短按发生在两次业务采样之间时仍可能被漏过。
- 环形缓冲区满时停止投递这一段不连续数据，由任务清空积压和半帧后重新同步。UART/DMA 错误由回调记录并通知，停止、清理和重新启动接收由任务执行。

调试时可观察 `rc_parse_ok_count`、`rc_crc_fail_count`、`rc_ring_high_water`、`rc_ring_overflow_count`、`rc_dropped_byte_count` 和 `rc_recovery_count`。`rc_dropped_byte_count` 统计中断未能投递的字节，不包含任务恢复时清空的积压数据。

满速连续输入下，128 字节 DMA 区约 3.05 ms 写满一圈，HT 间隔约 1.52 ms；中断必须及时服务，长时间屏蔽中断仍可能导致 DMA 覆盖。上板需测量中断响应、任务栈余量、队列状态更新延迟及对 1 ms 电机任务的影响。

本次只调整接收与任务间数据传递，原遥控通道映射和动作逻辑保持不变。当前没有遥控失联自动停车逻辑；通信停止后仍保留最后一次状态，`last_valid_tick` 仅提供后续检测依据。

## RadioMaster 遥控器脚本

由于本工程需要遥控器发送12位数字到主控，再由主控通过串口传输到ESP32S3的屏幕后显示对应的二维码，所以自己编写了一个遥控器脚本

遥控器工具位于：

```text
Tools/RadioMaster/code12.lua
```

用typec线与遥控器顶端连接后可访问遥控器中的sd存储卡，将其复制到 RadioMaster 遥控器 SD 卡即可：

```text
/SCRIPTS/TOOLS/code12.lua
```

该工具通过 CH13～CH16 传递 12 位编码数据，按键、开关和通道配置说明请查看 Lua 文件开头的注释

## 测试

遥控接收主机测试使用 HAL/FreeRTOS 测试替身驱动实际 `remote_control.c`：

```powershell
gcc -std=c11 -Wall -Wextra -Werror -Itests/host/remote_rx_stubs tests/remote_rx_test.c -o tests/remote_rx_test.exe
.\tests\remote_rx_test.exe
```

覆盖分包、连续多帧、DMA 与软件环形区回绕、重复/滞后回调、CRC 错误重同步、Code12 解码、多消费者 Peek、最新状态覆盖、环形区溢出以及 UART 错误恢复。测试同时断言 ISR 不执行解析和队列发布、正常接收不重复启动 DMA。

主机测试验证逻辑；中断时序、真实 UART 错误和机械动作仍需在开发板上验证。完整固件使用 `MDK-ARM/R1.uvprojx` 编译。

## 硬件资料

- [STM32G474 控制板原理图](docs/schematic_G474.pdf)
- [ESP32S3屏幕原理图](docs/schematic_ESP32S3Screen.pdf)
- [R1_遥控器手册](<docs/R1 遥控器控制手册.md>)

## 安全注意事项

- 上电前确认电源电压、极性和公共地连接正确，以及有无电源线松动的情况，放置短接导致主控板烧毁
- 首次调试机械臂、升降和夹爪时应限制速度，并预留紧急断电方式
- 拨杆拨到B3进入到机械臂模式的时候机械臂会回到事先设置的机械零位，过程非常危险，严禁再机械臂在半空中初始化
- 控制升降时要多注意上下限，现未启用距离传感器保护升降平台需要人为注意
## 许可证

本项目使用 MIT License，详情参见 [LICENSE](LICENSE)