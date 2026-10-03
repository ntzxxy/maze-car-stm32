#include "pid_controller.h"
#include <string.h>

/**
 * @brief 初始化PID控制器
 *
 * @param pid PID控制器指针
 * @param type PID类型（位置式或增量式）
 */
void PID_Init(PidController_t *pid, PidType_t type)
{
    if (pid == NULL)
        return;

    /* 清零所有参数 */
    memset(pid, 0, sizeof(PidController_t));

    /* 设置默认参数 */
    pid->type = type;
    pid->output_max = 1000.0f;
    pid->output_min = -1000.0f;
    pid->integral_max = 100.0f;
    pid->integral_min = -100.0f;
    pid->enable = true;
}

/**
 * @brief 设置PID参数
 *
 * @param pid PID控制器指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_SetParam(PidController_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL)
        return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief 设置目标值
 *
 * @param pid PID控制器指针
 * @param target 目标值
 */
void PID_SetTarget(PidController_t *pid, float target)
{
    if (pid == NULL)
        return;

    pid->target = target;
}

/**
 * @brief 设置输出限制
 *
 * @param pid PID控制器指针
 * @param min 最小输出值
 * @param max 最大输出值
 */
void PID_SetOutputLimit(PidController_t *pid, float min, float max)
{
    if (pid == NULL)
        return;

    pid->output_min = min;
    pid->output_max = max;
}

/**
 * @brief 设置积分限制
 *
 * @param pid PID控制器指针
 * @param min 积分最小值
 * @param max 积分最大值
 */
void PID_SetIntegralLimit(PidController_t *pid, float min, float max)
{
    if (pid == NULL)
        return;

    pid->integral_min = min;
    pid->integral_max = max;
}

/**
 * @brief 使能/禁用PID控制器
 *
 * @param pid PID控制器指针
 * @param enable 使能标志
 */
void PID_Enable(PidController_t *pid, bool enable)
{
    if (pid == NULL)
        return;

    pid->enable = enable;

    /* 禁用时清零输出 */
    if (!enable)
    {
        pid->output = 0.0f;
        pid->error_sum = 0.0f;
    }
}

/**
 * @brief 重置PID控制器
 *
 * @param pid PID控制器指针
 */
void PID_Reset(PidController_t *pid)
{
    if (pid == NULL)
        return;

    pid->error = 0.0f;
    pid->error_prev = 0.0f;
    pid->error_sum = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 限制数值在指定范围内
 *
 * @param value 待限制的值
 * @param min 最小值
 * @param max 最大值
 * @return float 限制后的值
 */
static float PID_Limit(float value, float min, float max)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

/**
 * @brief PID计算函数
 *
 * @param pid PID控制器指针
 * @param current 当前反馈值
 * @return float PID输出值
 */
float PID_Compute(PidController_t *pid, float current)
{
    if (pid == NULL || !pid->enable)
        return 0.0f;

    /* 更新当前值 */
    pid->current = current;

    /* 计算误差 */
    pid->error = pid->target - pid->current;

    if (pid->type == PID_TYPE_POSITIONAL) {
        /* 位置式PID算法 */

        /* 积分项累加 */
        if(pid->ki != 0.0f)
        {
            pid->error_sum += pid->error;
        }
        else
        {
            pid->error_sum = 0.0f; // 如果Ki为0，清零积分和
        }

        /* 积分限幅 */
        pid->error_sum = PID_Limit(pid->error_sum, pid->integral_min, pid->integral_max);

        /* PID计算：u(k) = Kp*e(k) + Ki*∑e(k) + Kd*(e(k)-e(k-1)) */
        pid->output = pid->kp * pid->error +
                      pid->ki * pid->error_sum +
                      pid->kd * (pid->error - pid->error_prev);
    } else if (pid->type == PID_TYPE_INCREMENTAL) {
        /* 增量式PID算法 */

        /* PID计算：Δu(k) = Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2*e(k-1)+e(k-2)) */
        float delta_output = pid->kp * (pid->error - pid->error_prev) +
                             pid->ki * pid->error +
                             pid->kd * (pid->error - 2.0f * pid->error_prev + pid->error_sum);

        /* 累加输出 */
        pid->output += delta_output;

        /* 更新误差历史 */
        pid->error_sum = pid->error_prev; /* e(k-2) = e(k-1) */
    }

    /* 输出限幅 */
    pid->output = PID_Limit(pid->output, pid->output_min, pid->output_max);

    /* 更新误差历史 */
    pid->error_prev = pid->error;

    return pid->output;
}
