/**
 * @file pid_controller.h
 * @author Shiki
 * @brief PID控制器通用模板头文件
 *        提供标准的PID控制算法实现，支持位置式和增量式PID
 * @version 0.1
 * @date 2025-07-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* PID控制器类型定义 */
typedef enum {
    PID_TYPE_POSITIONAL = 0,    /* 位置式PID */
    PID_TYPE_INCREMENTAL        /* 增量式PID */
} PidType_t;

/* PID控制器参数结构体 */
typedef struct {
    float kp;                   /* 比例系数 */
    float ki;                   /* 积分系数 */
    float kd;                   /* 微分系数 */
    float target;               /* 目标值 */
    float current;              /* 当前值 */
    float error;                /* 当前误差 */
    float error_prev;           /* 前一次误差 */
    float error_sum;            /* 误差积分和 */
    float output;               /* PID输出值 */
    float output_max;           /* 输出最大限制 */
    float output_min;           /* 输出最小限制 */
    float integral_max;         /* 积分限幅 */
    float integral_min;         /* 积分限幅 */
    PidType_t type;             /* PID类型 */
    bool enable;                /* 使能标志 */
} PidController_t;

/* 函数声明 */
void PID_Init(PidController_t *pid, PidType_t type);
void PID_SetParam(PidController_t *pid, float kp, float ki, float kd);
void PID_SetTarget(PidController_t *pid, float target);
void PID_SetOutputLimit(PidController_t *pid, float min, float max);
void PID_SetIntegralLimit(PidController_t *pid, float min, float max);
void PID_Enable(PidController_t *pid, bool enable);
void PID_Reset(PidController_t *pid);
float PID_Compute(PidController_t *pid, float current);

#endif /* __PID_CONTROLLER_H */
