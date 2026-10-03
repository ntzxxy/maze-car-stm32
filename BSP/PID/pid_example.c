#include "pid_example.h"
#include <stdio.h>

/* 全局PID控制器实例 */
PidController_t g_motor_speed_pid;    /* 电机速度控制PID */
PidController_t g_servo_position_pid; /* 舵机位置控制PID */

/**
 * @brief PID示例初始化
 */
void PID_Example_Init(void)
{
    /* 初始化电机速度PID控制器（位置式） */
    PID_Init(&g_motor_speed_pid, PID_TYPE_POSITIONAL);
    PID_SetParam(&g_motor_speed_pid, 0.8f, 0.1f, 0.05f);     /* Kp=0.8, Ki=0.1, Kd=0.05 */
    PID_SetOutputLimit(&g_motor_speed_pid, -100.0f, 100.0f); /* PWM占空比限制 */
    PID_SetIntegralLimit(&g_motor_speed_pid, -50.0f, 50.0f); /* 积分限幅 */

    /* 初始化舵机位置PID控制器（位置式） */
    PID_Init(&g_servo_position_pid, PID_TYPE_POSITIONAL);
    PID_SetParam(&g_servo_position_pid, 1.2f, 0.05f, 0.1f);     /* Kp=1.2, Ki=0.05, Kd=0.1 */
    PID_SetOutputLimit(&g_servo_position_pid, -90.0f, 90.0f);   /* 角度限制 */
    PID_SetIntegralLimit(&g_servo_position_pid, -30.0f, 30.0f); /* 积分限幅 */

    printf("PID controllers initialized successfully\r\n");
}

/**
 * @brief 电机速度PID控制示例
 *
 * @param target_speed 目标速度 (RPM)
 * @param current_speed 当前速度 (RPM)
 */
void PID_Example_MotorSpeedControl(float target_speed, float current_speed)
{
    /* 设置目标速度 */
    PID_SetTarget(&g_motor_speed_pid, target_speed);

    /* 计算PID输出 */
    float pwm_output = PID_Compute(&g_motor_speed_pid, current_speed);

    /* 输出调试信息 */
    printf("Motor Speed Control - Target: %.1f, Current: %.1f, Output: %.1f\r\n",
           target_speed, current_speed, pwm_output);

    /* 在这里可以将pwm_output应用到电机控制 */
    /* 例如：Motor_SetPWM((int16_t)pwm_output); */
}

/**
 * @brief 舵机位置PID控制示例
 *
 * @param target_angle 目标角度 (度)
 * @param current_angle 当前角度 (度)
 */
void PID_Example_ServoPositionControl(float target_angle, float current_angle)
{
    /* 设置目标角度 */
    PID_SetTarget(&g_servo_position_pid, target_angle);

    /* 计算PID输出 */
    float servo_output = PID_Compute(&g_servo_position_pid, current_angle);

    /* 输出调试信息 */
    printf("Servo Position Control - Target: %.1f, Current: %.1f, Output: %.1f\r\n",
           target_angle, current_angle, servo_output);

    /* 在这里可以将servo_output应用到舵机控制 */
    /* 例如：Servo_SetAngle_CH1((int16_t)servo_output); */
}

/**
 * @brief PID参数在线调节示例
 *
 * @param pid_index PID控制器索引 (0:电机速度, 1:舵机位置)
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_Example_TuneParameters(uint8_t pid_index, float kp, float ki, float kd)
{
    switch (pid_index)
    {
    case 0: /* 电机速度PID */
        PID_SetParam(&g_motor_speed_pid, kp, ki, kd);
        printf("Motor Speed PID parameters updated: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", kp, ki, kd);
        break;

    case 1: /* 舵机位置PID */
        PID_SetParam(&g_servo_position_pid, kp, ki, kd);
        printf("Servo Position PID parameters updated: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", kp, ki, kd);
        break;

    default:
        printf("Invalid PID index: %d\r\n", pid_index);
        break;
    }
}

/**
 * @brief 重置所有PID控制器
 */
void PID_Example_ResetAll(void)
{
    PID_Reset(&g_motor_speed_pid);
    PID_Reset(&g_servo_position_pid);
    printf("All PID controllers reset\r\n");
}
