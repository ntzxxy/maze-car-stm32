/**
 * @file pid_example.h
 * @author Shiki
 * @brief PID控制器使用示例头文件
 *        展示如何在具体应用中使用PID控制器
 * @version 0.1
 * @date 2025-07-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __PID_EXAMPLE_H
#define __PID_EXAMPLE_H

#include "pid_controller.h"

/* 示例：电机速度PID控制器 */
extern PidController_t g_motor_speed_pid;

/* 示例：舵机位置PID控制器 */
extern PidController_t g_servo_position_pid;

/* 函数声明 */
void PID_Example_Init(void);
void PID_Example_MotorSpeedControl(float target_speed, float current_speed);
void PID_Example_ServoPositionControl(float target_angle, float current_angle);

#endif /* __PID_EXAMPLE_H */
