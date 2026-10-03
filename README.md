# Maze Car：基于 STM32F103 的迷宫小车

这是一个基于 STM32F103C8T6 的双轮差速迷宫小车项目，使用 STM32 HAL 库开发，工程由 STM32CubeMX 生成并使用 Keil MDK-ARM 构建。

项目包含电机驱动、双编码器测速、速度 PID、航向角修正、三路 VL6180X 测距、WIT 姿态传感器、OLED 显示、按键交互以及预设迷宫路线状态机。。

## 功能概览

- 双直流电机 PWM 与方向控制
- TIM2、TIM3 双编码器计数
- 双轮速度 PID 控制
- 差速运动、航向角转向和直行辅助模式
- 三路 VL6180X 前、左、右距离采集
- ToF 数据滑动平均与墙体阈值判断
- WIT 姿态传感器航向角读取
- OLED 实时显示速度、航向角和距离
- 四按键模式切换与运动测试
- 按编码器里程和航向角执行预设迷宫路线

## 硬件设计

### 主控

- MCU：STM32F103C8T6，Cortex-M3，LQFP48
- 系统时钟：外部 8 MHz 晶振，经 PLL 9 倍频至 72 MHz
- 调试接口：SWD，PA13/SWDIO、PA14/SWCLK

### 双轮差速驱动

电机驱动使用两路方向信号、两路 PWM 和一个待机控制信号。驱动芯片的具体型号请以实物原理图为准。

| 功能 | STM32 引脚 | 配置 |
| --- | --- | --- |
| 电机 A PWM | PA8 | TIM1_CH1 |
| 电机 B PWM | PA9 | TIM1_CH2 |
| 电机 A 方向 | PA15 / PB3 | AIN1 / AIN2 |
| 电机 B 方向 | PB4 / PB5 | BIN1 / BIN2 |
| 驱动待机 | PA10 | STBY |

TIM1 的预分频值为 71、自动重装值为 99，在 72 MHz 定时器时钟下产生约 10 kHz 的 PWM，比较值范围对应约 0%～100% 占空比。

### 编码器

| 车轮编码器 | STM32 引脚 | 定时器 |
| --- | --- | --- |
| 编码器 A | PA6 / PA7 | TIM3_CH1 / TIM3_CH2 |
| 编码器 B | PA0 / PA1 | TIM2_CH1 / TIM2_CH2 |

TIM4 产生 1 ms 周期中断。程序每 20 次中断计算一次轮速并执行控制环，即控制周期约为 20 ms（50 Hz）。

### 距离传感器

项目使用三路 VL6180X 测量前、左、右方向距离，共用 I²C2：

| 功能 | STM32 引脚/地址 |
| --- | --- |
| I²C2 SCL / SDA | PB10 / PB11，100 kHz |
| 前方 VL6180X XSHUT | PB0 |
| 左侧 VL6180X XSHUT | PA4 |
| 右侧 VL6180X XSHUT | PA11 |
| 三个设备的 7-bit 地址 | 0x29 / 0x2A / 0x2B |

上电时通过三个 XSHUT 引脚依次唤醒传感器并修改地址，避免多个同地址设备发生总线冲突。迷宫模块对三路距离做 3 点滑动平均，并按阈值判断墙体。

### 姿态、显示与调试

| 模块 | 接口 | 配置 |
| --- | --- | --- |
| WIT 姿态传感器 | USART1，PB6 / PB7 | 115200，DMA 接收 |
| 串口调试输出 | USART2，PA2 / PA3 | 115200，8N1 |
| OLED | I²C1，PB8 / PB9 | 100 kHz，代码使用 8-bit 地址 0x78 |
| KEY1～KEY4 | PB12～PB15 | 上拉输入，低电平按下 |

当前源码中的 WIT 驱动实际使用 `USART1`；部分驱动文件头部的旧注释可能仍写有其他串口编号，应以代码和 CubeMX 配置为准。

## 软件设计

### 分层结构

```text
Maze_Car/
├── BSP/
│   ├── KEY/        # 按键扫描、工作模式和测试入口
│   ├── Maze/       # 里程统计、ToF 判墙和迷宫状态机
│   ├── Motor/      # 电机、编码器和运动控制
│   ├── OLED/       # OLED 显示
│   ├── PID/        # 通用 PID 控制器与示例
│   ├── VL6180X/    # 三路 ToF 测距驱动
│   └── WIT/        # 姿态传感器串口驱动
├── Core/           # 主程序、外设初始化和中断处理
├── Drivers/        # STM32 HAL 与 CMSIS
├── MDK-ARM/        # Keil 工程文件
├── Maze_Car.ioc    # STM32CubeMX 配置
├── .gitignore
└── README.md
```

### 基础运动单元

底层运动接口主要位于 `BSP/Motor/`：

- `Motor_Init()`：启动两路 PWM。
- `Motor_On()` / `Motor_Stop()`：使能或停止电机驱动。
- `MotorA_Duty()` / `MotorB_Duty()`：设置电机占空比和方向。
- `Encoder_Init()` / `Encoder_GetValue()`：启动并读取双编码器。
- `Motor_SetTargetSpeed()`：分别设置两轮目标速度。
- `Motion_Set_Speed(forward, turn)`：设置前进与转向分量。
- `Motion_Set_Target_Angle()`：设置目标航向角。
- `Motion_SetControlMode()`：切换运动控制模式。

已实现的控制模式包括：

| 模式 | 用途 |
| --- | --- |
| `MOTION_MODE_SPEED_ONLY` | 两轮独立速度控制 |
| `MOTION_MODE_DIFFERENTIAL` | 前进量与转向量组合的差速控制 |
| `MOTION_MODE_YAW_CORRECTED` | 使用航向角 PID 完成定角转向 |
| `MOTION_MODE_YAW_STRAIGHT` | 使用航向角辅助保持直行 |

### 迷宫流程

`BSP/Maze/maze.c` 中包含两套流程：

- `Maze_Hardcoded_Task()`：用于毕设展示的预设路线状态机，按编码器里程、左侧路口检测和目标航向角依次执行直行与转向。
- `Maze_Task()`：自主判路框架，已经包含一格里程、三向判墙和决策状态，但转向决策等部分仍有 `TODO`，尚未形成完整解迷宫算法。

默认主循环在 `MODE_MAZE_TEST` 下调用 `Maze_Hardcoded_Task()`。当前预设序列大致为：直行并寻找路口、左转、直行、左转、直行、右转、继续直行，具体格数和阈值见 `BSP/Maze/maze.h`。

## 编译与下载

### 使用 Keil MDK

1. 安装 Keil MDK-ARM 及 STM32F1 Device Family Pack。
2. 打开 `MDK-ARM/Maze_Car.uvprojx`。
3. 编译工程。
4. 使用 ST-Link 通过 SWD 下载到开发板。

工程记录的目标工具链为 MDK-ARM V5.32，STM32Cube 固件包为 STM32Cube FW_F1 V1.8.6。其他版本可能需要迁移工程或重新选择设备包。

### 使用 STM32CubeMX

1. 使用 STM32CubeMX 打开 `Maze_Car.ioc`。
2. 检查时钟、引脚、DMA 和中断配置。
3. 重新生成代码时，注意保留 `USER CODE BEGIN/END` 区域中的用户代码。
4. 重新生成后检查 Keil 工程中的 `BSP` 分组和头文件搜索路径是否仍然存在。

## 参数标定

以下参数与底盘、电机、编码器、电池电压和迷宫尺寸相关，移植后应重新标定：

- `MAZE_CELL_TICKS`：小车行驶一格对应的平均编码器计数。
- `TEST_OFFSET_TICKS`：检测到路口后继续前进的补偿距离。
- `WALL_*_THRESH_MM`：前、左、右墙体判断阈值。
- 速度环、航向角环和直行辅助环的 PID 参数。
- 左右电机方向、目标速度符号和机械安装对应关系。

建议先架空车轮验证方向与编码器符号，再从低速开始进行实车标定。

## 当前完成度与限制

- 已提供电机、编码器、PID、姿态、测距、OLED 和按键等基础运动与调试单元。
- 底层运动可使用编码器速度闭环和航向角修正，但不同车辆需要重新调参。
- 迷宫展示逻辑采用预设路线和固定状态序列，属于路线级开环控制。
- `Maze_Task()` 仅是自主判路框架，没有完成在线建图、完整转向决策、路径搜索和最短路径回放。
- 传感器异常处理、边界条件和长期运行稳定性仍有继续完善空间。

由于时间有限，本项目最终只完成了基于硬编码序列的开环走迷宫验证。为时间紧张的同学提供基础运动代码，可以有更多的时间继续研究A*、洪水等迷宫算法。



