/**
 ******************************************************************************
 * @file    app_key.h
 * @author  
 * @version V1.0.0
 * @date    2025-07-21
 * @brief   按键应用层模块头文件 (Header for Key Application Layer)
 *
 * @attention
 *
 * 本模块负责解析来自底层键盘驱动的键值，并将其转换为高层的系统状态切换、
 * 运动控制指令和参数调整等应用层逻辑。它充当了用户物理输入与机器
 * 行为之间的桥梁。
 *
 ******************************************************************************
 *
 * STM32CubeMX 配置要求 (为其依赖模块):
 * ------------------------------------------------
 * 1. GPIO (用于底层键盘驱动):
 * - 4个引脚配置为推挽输出 (ROW 1-4)
 * 使用PE12\13\14\15作为行引脚
 * - 4个引脚配置为输入，带上拉 (COLUMN 1-4)
 * 使用PE11\10\9\8作为列引脚
 * 
 *
 * 依赖的软件模块:
 * ------------------------------------------------
 * 1. 底层键盘驱动: 需要一个能提供稳定、消抖后键值的函数 (如 key_get_value())。
 * 2. 电机驱动模块 (`motor.h`): 提供电机启动、停止、设置PWM占空比等底层函数。
 * 3. PID控制器模块 (`pid.h`): 提供PID控制器的结构体和 `MotionMode_t` 枚举的定义。
 * 4. 运动控制模块 (`motion_control.h`): 提供运动模式切换、速度设定等高层控制函数。
 * 5. 陀螺仪驱动模块 (`wit.h`): 提供航向角清零和数据读取功能。
 *
 * 使用说明:
 * ------------------------------------------------
 * 1. 在 main.c 中包含本头文件 `#include "app_key.h"`。
 * 2. 在主循环 `while(1)` 中，周期性地调用 `app_key_handler()` 函数，在`app_key_handler()` 函数自定义按键操控
 * 并传入由底层键盘驱动获取的键值。
 * 示例:
 * uint8_t key = key_get_value();
 * app_key_handler(key);
 *
 ******************************************************************************
 */
#ifndef __APP_KEY_H
#define __APP_KEY_H

#include "main.h"
#include "key.h"

typedef enum {
    MODE_NORMAL_OPERATION,  // 正常运行模式 (这就是你 Key_Process 的功能)
    MODE_TUNE_MOTOR_A,      // Motor A PID 调参模式
    MODE_MAZE_TEST
} KeyAppMode_t;

extern KeyAppMode_t g_current_mode; // 访问当前模式

void app_key_handler(uint8_t key_value);


#endif