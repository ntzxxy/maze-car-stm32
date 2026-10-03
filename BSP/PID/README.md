# PID控制器模块

## 概述

这是一个通用的PID控制器模板，适用于STM32F407项目。该模块提供了标准的PID控制算法实现，支持位置式和增量式两种PID算法。

## 文件结构

```
BSP/PID/
├── pid_controller.h      # PID控制器核心头文件
├── pid_controller.c      # PID控制器核心实现
├── pid_example.h         # 使用示例头文件
├── pid_example.c         # 使用示例实现
└── README.md            # 说明文档
```

## 功能特性

- 支持位置式PID和增量式PID算法
- 可配置的PID参数（Kp, Ki, Kd）
- 输出限幅功能
- 积分限幅功能（防止积分饱和）
- 控制器使能/禁用
- 参数重置功能
- 完整的错误检查

## API接口

### 初始化函数
```c
void PID_Init(PidController_t *pid, PidType_t type);
```

### 参数设置函数
```c
void PID_SetParam(PidController_t *pid, float kp, float ki, float kd);
void PID_SetTarget(PidController_t *pid, float target);
void PID_SetOutputLimit(PidController_t *pid, float min, float max);
void PID_SetIntegralLimit(PidController_t *pid, float min, float max);
```

### 控制函数
```c
void PID_Enable(PidController_t *pid, bool enable);
void PID_Reset(PidController_t *pid);
float PID_Compute(PidController_t *pid, float current);
```

## 使用示例

### 1. 基本使用流程

```c
#include "pid_controller.h"

// 定义PID控制器实例
PidController_t motor_pid;

void setup() {
    // 初始化PID控制器（位置式）
    PID_Init(&motor_pid, PID_TYPE_POSITIONAL);
    
    // 设置PID参数
    PID_SetParam(&motor_pid, 0.8f, 0.1f, 0.05f);
    
    // 设置输出限制
    PID_SetOutputLimit(&motor_pid, -100.0f, 100.0f);
    
    // 设置积分限幅
    PID_SetIntegralLimit(&motor_pid, -50.0f, 50.0f);
}

void control_loop() {
    float target = 1000.0f;    // 目标值
    float current = get_current_value();  // 获取当前值
    
    // 设置目标值
    PID_SetTarget(&motor_pid, target);
    
    // 计算PID输出
    float output = PID_Compute(&motor_pid, current);
    
    // 应用输出到执行器
    apply_output(output);
}
```

### 2. 电机速度控制示例

参考 `pid_example.c` 中的 `PID_Example_MotorSpeedControl()` 函数。

### 3. 舵机位置控制示例

参考 `pid_example.c` 中的 `PID_Example_ServoPositionControl()` 函数。

## PID算法说明

### 位置式PID
```
u(k) = Kp*e(k) + Ki*∑e(k) + Kd*(e(k)-e(k-1))
```

### 增量式PID
```
Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2))
u(k) = u(k-1) + Δu(k)
```

## 参数调节建议

1. **比例系数(Kp)**: 影响系统的响应速度，过大会导致震荡
2. **积分系数(Ki)**: 消除稳态误差，过大会导致超调
3. **微分系数(Kd)**: 改善系统的动态特性，过大会放大噪声

建议调节顺序：
1. 先设置Ki=0, Kd=0，调节Kp至系统有轻微震荡
2. 适当减小Kp，然后增加Ki消除稳态误差
3. 最后适当增加Kd改善动态性能

## 注意事项

1. 在使用前必须调用 `PID_Init()` 初始化
2. 建议设置合适的输出限幅避免执行器饱和
3. 设置积分限幅防止积分饱和现象
4. 定期调用 `PID_Compute()` 函数更新PID输出
5. 可以通过 `PID_Enable()` 动态使能/禁用控制器

## 集成到项目

要在你的项目中使用PID控制器：

1. 在需要使用PID的文件中包含头文件：
   ```c
   #include "pid_controller.h"
   ```

2. 在 `app_tasks.c` 中集成PID控制任务：
   ```c
   #include "pid_example.h"
   
   static void Task_PIDControl(void)
   {
       // 调用你的PID控制函数
       PID_Example_MotorSpeedControl(target_speed, current_speed);
   }
   
   void AppTasks_Init(void)
   {
       // 初始化PID示例
       PID_Example_Init();
       
       // 添加PID控制任务
       TaskScheduler_AddTask(Task_PIDControl, 20, TASK_PRIORITY_NORMAL, "PID_Task");
   }
   ```
