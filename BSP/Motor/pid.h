/**
 ******************************************************************************
 * @file    pid.h
 * @author  (你的名字)
 * @version V1.0.0
 * @date    (创建日期)
 * @brief   运动控制核心模块头文件 
 *
 * @attention
 *
 * 它集成了速度闭环PID和航向角闭环PID，
 * 实现了灵活的、可切换的运动控制模式。它可以将简单的“前进”、“转向”
 * 指令，解析为两个电机精确的速度目标，并能通过陀螺仪实现自主航向修正。
 *
 ******************************************************************************
 *
 * STM32CubeMX 配置要求:
 * ------------------------------------------------
 * 1. TIM (用于电机PWM):
 * - 选择一个高级或通用定时器 (如 TIM9)。
 * - 配置2个通道为PWM Generation模式 (如 Channel 1, Channel 2)。
 * - 设置合适的预分频器(Prescaler)和计数周期(Counter Period)以确定PWM频率和分辨率。
 *
 * 2. TIM (用于编码器接口):
 * - 选择2个通用定时器 (如 TIM3, TIM4)。
 * - 将它们配置为Combined Channels > Encoder Mode。
 * - Encoder Mode Tl1 and Tl2, Polarity: Rising Edge。
 *
 * 3. TIM (用于主控制循环):
 * - 选择一个基本定时器 (如 TIM6)。
 * - 配置其为周期性中断源，例如每1ms触发一次更新中断。
 * - 在NVIC中使能该定时器的中断。
 *
 * 4. USART (用于陀螺仪):
 * - 配置一个USART (如 USART2) 为异步模式。
 * - 波特率等参数需与陀螺仪模块匹配。
 * - 在NVIC中使能其全局中断，并配置DMA用于接收。
 *
 * 依赖的软件模块:
 * ------------------------------------------------
 * 1. `pid.h`: 提供了核心的PID控制器结构体和API，以及 `MotionMode_t` 枚举。
 * 2. `motor.h`: 提供了底层的电机驱动和编码器读值函数。
 * 3. `wit.h`: 提供了陀螺仪数据结构体 `g_wit_data`。
 *
 * 使用说明:
 * ------------------------------------------------
 * 1. 在主初始化流程中，调用 `PID_Motor_Init()` 来初始化所有PID控制器。
 * 2. 在主循环 `while(1)` 中，通过调用 `app_key_handler()` 来间接调用本模块的控制函数。
 * 3. 所有的实时计算都在 `TIM6` 的中断服务函数 `HAL_TIM_PeriodElapsedCallback()` 中自动执行。
 * 4. 上层应用（如按键处理）通过调用本文件声明的 `Motion_...` 系列函数来控制机器人的行为。
 * 
 * 
 *  * --- 模式使用示例 ---
 * 1.  **纯速度模式 (MOTION_MODE_SPEED_ONLY):** 用于底层测试，直接控制两轮速度。
 * // 步骤1: 切换到纯速度模式
 * Motion_SetControlMode(MOTION_MODE_SPEED_ONLY);
 * // 步骤2: 设置两轮的目标速度
 * Motor_SetTargetSpeed(50.0f, -50.0f); // A轮正转，B轮反转
 *
 * 2.  **差速驱动模式 (MOTION_MODE_DIFFERENTIAL):** 用于手动遥控。
 * // 步骤1: 切换到差速模式
 * Motion_SetControlMode(MOTION_MODE_DIFFERENTIAL);
 * // 步骤2: 发出前进或转向指令
 * Motion_Set_Speed(FORWARD_SPEED_FAST, 0); // 全速前进
 *
 * 3.  **航向角闭环模式 (MOTION_MODE_YAW_CORRECTED):** 用于自主走直线或转固定角度。
 * // --- 示例: 自主右转90度 ---
 * // 步骤1: 切换到航向角闭环模式
 * Motion_SetControlMode(MOTION_MODE_YAW_CORRECTED);
 * // 步骤2: 计算并设置目标角度
 * float target = current_yaw_angle + 90.0f;
 * Motion_Set_Target_Angle(target);
 * // 步骤3: 清除手动指令，让PID完全接管转向
 * Motion_Set_Speed(0, 0);
 *
 ******************************************************************************
 */

#ifndef __PID_H
#define __PID_H

#include "main.h"
#include "pid_controller.h"
#include "motor.h"

#define FORWARD_SPEED_SLOW 10.0f
#define FORWARD_SPEED_FAST 30.0f
#define TURN_SPEED_NORMAL  7.0f


typedef enum {
    MOTION_MODE_SPEED_ONLY,      ///< 纯速度环模式，直接控制两轮速度
    MOTION_MODE_DIFFERENTIAL,    ///< 差速驱动模式，通过前进和转向指令控制
    MOTION_MODE_YAW_CORRECTED,   ///< 航向角闭环模式，在差速驱动上增加航向修正
    MOTION_MODE_YAW_STRAIGHT,   // 【新增】直行辅助模式
} MotionMode_t;

static float normalize_angle_error(float target, float current);
void PID_Motor_Init(void);
void Motor_SetTargetSpeed(float speed_A, float speed_B);
void PID_MotorSpeedControl(void);
void PID_enable(PidController_t *pid, bool enable, float initial_output);
void Motion_SetControlMode(MotionMode_t mode);
void Motion_Set_Speed(float forward, float turn);
void Motion_Set_Target_Angle(float angle);
void ControlLoop_SpeedOnly(void);
void ControlLoop_Differential(void);
void ControlLoop_YawCorrected(void);
void ControlLoop_YawStraight(void);



extern PidController_t g_motorA_speed_pid;    /* 电机速度控制PID */
extern PidController_t g_motorB_speed_pid;    /* 电机速度控制PID */
extern PidController_t g_yaw_angle_pid;                     // 访问角度环的PID参数
extern PidController_t g_straight_assist_pid;
extern volatile MotionMode_t g_motion_mode;    // 当前运动模式(如速度环、差速驱动、航向角闭环等)
extern volatile float g_target_forward_speed;  // 前进速度目标
extern volatile float g_target_turn_speed;     // 手动转向目标
extern volatile float g_target_yaw_angle;      // 目标航向角
extern volatile uint8_t g_turn_complete;


#endif
