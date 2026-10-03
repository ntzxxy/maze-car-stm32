#include "pid.h"
#include "math.h"
#include "wit.h"

PidController_t g_motorA_speed_pid;    /* 电机速度控制PID */
PidController_t g_motorB_speed_pid;    /* 电机速度控制PID */
PidController_t g_yaw_angle_pid;                     // 访问角度环的PID参数
PidController_t g_straight_assist_pid;      // 直行辅助模式的PID控制器


volatile MotionMode_t g_motion_mode = MOTION_MODE_DIFFERENTIAL; // 默认使用差速模式
volatile float g_target_forward_speed = 0.0f;
volatile float g_target_turn_speed = 0.0f;
volatile float g_target_yaw_angle = 0.0f;
volatile uint8_t g_turn_complete = 0;

/**
 * @brief 计算两个角度之间的最短误差 (-180 to 180)
 * @param target 目标角度
 * @param current 当前角度
 * @return 归一化后的误差
 */
static float normalize_angle_error(float target, float current)
{
    float error = target - current;
    if (error > 180.0f) {
        error -= 360.0f;
    } else if (error < -180.0f) {
        error += 360.0f;
    }
    return error;
}


void PID_Motor_Init(void)
{
    /* 初始化电机A速度PID控制器（增量式） */
    PID_Init(&g_motorA_speed_pid, PID_TYPE_INCREMENTAL);
    PID_SetParam(&g_motorA_speed_pid, 1.21f, 0.19f, 0.0f);    /* Kp=0.0, Ki=0.19, Kd=0.0 */
    PID_SetOutputLimit(&g_motorA_speed_pid, -100.0f, 100.0f); /* PWM占空比限制 */
    PID_SetIntegralLimit(&g_motorA_speed_pid, -50.0f, 50.0f); /* 积分限幅 */

    /* 初始化电机B速度PID控制器（增量式） */
    PID_Init(&g_motorB_speed_pid, PID_TYPE_INCREMENTAL);
    PID_SetParam(&g_motorB_speed_pid, 1.22f, 0.19f, 0.0f);    /* Kp=0.0, Ki=0.19, Kd=0.0 */
    PID_SetOutputLimit(&g_motorB_speed_pid, -100.0f, 100.0f); /* PWM占空比限制 */
    PID_SetIntegralLimit(&g_motorB_speed_pid, -50.0f, 50.0f); /* 积分限幅 */

    /* 初始化角度环PID控制器（位置式） */
    PID_Init(&g_yaw_angle_pid, PID_TYPE_POSITIONAL);
    PID_SetParam(&g_yaw_angle_pid, 0.2f, 0.0f, 0.0f);
    PID_SetOutputLimit(&g_yaw_angle_pid, -TURN_SPEED_NORMAL, TURN_SPEED_NORMAL);        //根据实际转向快慢进行调节
    PID_SetIntegralLimit(&g_yaw_angle_pid, -100.0f, 100.0f);


    /* 【新增】初始化直行辅助环PID控制器（位置式） */
    // 这个环的目标是让两个轮子的速度差为0
    PID_Init(&g_straight_assist_pid, PID_TYPE_POSITIONAL);
    // Kp稍大，让它能快速响应偏差；Ki可以给一点，消除静差；Kd可以给一点防止震荡
    // 注意：这些参数需要根据你的小车实际情况进行整定！
    PID_SetParam(&g_straight_assist_pid, 0.8f, 0.0f, 5.0f); 
    // 输出限制的是施加在两个轮子上的“速度修正差值”，不宜过大
    PID_SetOutputLimit(&g_straight_assist_pid, -30.0f, 30.0f); 
    PID_SetIntegralLimit(&g_straight_assist_pid, -20.0f, 20.0f);
}

/**
 * @brief   设置当前的运动控制模式。
 * @details 切换小车的核心行为逻辑，例如是从手动遥控切换到自动航向保持。
 * @param   mode: 要设置的运动模式，定义在 `MotionMode_t` 枚举中。
 * @retval  None
 */
void Motion_SetControlMode(MotionMode_t mode)
{
    g_motion_mode = mode;
}

/**
 * @brief   设置高层的运动指令（前进和手动转向）。
 * @details 这是手动遥控小车的主要接口。
 * @param   forward: 目标前进速度。正值为前进，负值为后退。
 * @param   turn: 目标手动转向速度。正值为右转，负值为左转。
 * @retval  None
 */
void Motion_Set_Speed(float forward, float turn)
{
    g_target_forward_speed = forward;
    g_target_turn_speed = turn;
}

/**
 * @brief  设置目标航向角
 */
void Motion_Set_Target_Angle(float angle)
{
    g_target_yaw_angle = angle;
}

void Motor_SetTargetSpeed(float speed_A, float speed_B)
{
    // 这个函数内部其实就是调用我们之前讨论过的PID库函数
    PID_SetTarget(&g_motorA_speed_pid, speed_A);
    PID_SetTarget(&g_motorB_speed_pid, speed_B);
}

/**
 * @brief  内环速度PID控制执行
 */
void PID_MotorSpeedControl(void)
{
    float pwm_output_A = PID_Compute(&g_motorA_speed_pid, (float)speed_A);
    float pwm_output_B = PID_Compute(&g_motorB_speed_pid, (float)speed_B);

    if (pwm_output_A >= 0) {
         g_motorA_dir = 1;
         MotorA_Duty(pwm_output_A); 
        }
    else { 
        g_motorA_dir = 0;
         MotorA_Duty(-pwm_output_A);
         }

    if (pwm_output_B >= 0) {
         g_motorB_dir = 0;
         MotorB_Duty(pwm_output_B);
         }
    else { 
        g_motorB_dir = 1; 
        MotorB_Duty(-pwm_output_B);
         }
}

void PID_enable(PidController_t *pid, bool enable, float initial_output)
{
    if (pid == NULL) return;
    pid->enable = enable;
    if (enable) {
        pid->output = initial_output; // <-- 关键：设置初始输出值
        pid->error_prev = 0.0f;
    } else {
        // ... 禁用时的逻辑 ...
    }
}

/**
 * @brief 控制循环：纯速度模式
 */
void ControlLoop_SpeedOnly(void)
{
    // 在此模式下，目标速度由外部直接设置
    // 我们只负责执行PID闭环
    PID_MotorSpeedControl();
}

/**
 * @brief 控制循环：差速驱动模式
 */
void ControlLoop_Differential(void)
{
    // a. 混合前进和转向指令
    float final_speed_A = g_target_forward_speed + g_target_turn_speed;
    float final_speed_B = g_target_forward_speed - g_target_turn_speed;
    
    // b. 设置为内环目标
    Motor_SetTargetSpeed(final_speed_A, final_speed_B);

    // c. 执行PID闭环
    PID_MotorSpeedControl();
}

/**
 * @brief 控制循环：航向角闭环模式
 */

/**
 * @brief 控制循环：航向角闭环模式
 */
void ControlLoop_YawCorrected(void)
{
    // 这个静态分频变量保留，可以降低角度环的更新频率，保持系统稳定
    static float last_sensor_yaw = 0.0f;

    // 外环（角度环）不需要每次都计算，但内环（速度环）必须每次都执行
    if (g_wit_data.yaw != last_sensor_yaw)
    {
        last_sensor_yaw = g_wit_data.yaw;

       
        
        // 1. 获取目标值和当前值
        g_yaw_angle_pid.target = g_target_yaw_angle;
        g_yaw_angle_pid.current = g_wit_data.yaw;

        // 2. 计算归一化后的最短角度误差
        float normalized_error = normalize_angle_error(g_yaw_angle_pid.target, g_yaw_angle_pid.current);
			
				if (fabs(normalized_error) < 2.0f) 
					{
							// A. 物理停车：立刻切断电机输出，利用反电动势或短接刹车
							Motor_Stop(); 
							
							// B. 状态切换：进入空闲模式，停止主循环调用此PID函数
							// 这样就彻底切断了"震荡源"，方便主程序判断 g_motor_state == IDLE 后进入直行
							g_motor_state = MOTOR_STATE_IDLE; 
							
							// C. 复位PID：清除积分累积，为下一次动作做准备
							PID_Reset(&g_motorA_speed_pid);
							PID_Reset(&g_motorB_speed_pid);
							PID_Reset(&g_yaw_angle_pid);

							g_turn_complete = 1;
							// D. 立刻退出：不再执行后面任何电机控制代码
							return; 
					}
        
        // 3. 更新PID结构体内的误差值
        g_yaw_angle_pid.error_prev = g_yaw_angle_pid.error;
        g_yaw_angle_pid.error = normalized_error;

        // 4. 计算积分项并进行限幅
        g_yaw_angle_pid.error_sum += g_yaw_angle_pid.error;
        if (g_yaw_angle_pid.error_sum > g_yaw_angle_pid.integral_max) {
            g_yaw_angle_pid.error_sum = g_yaw_angle_pid.integral_max;
        } else if (g_yaw_angle_pid.error_sum < g_yaw_angle_pid.integral_min) {
            g_yaw_angle_pid.error_sum = g_yaw_angle_pid.integral_min;
        }
        
        // 5. 根据位置式PID公式，计算总输出
        g_yaw_angle_pid.output = 
        g_yaw_angle_pid.kp * g_yaw_angle_pid.error + 
        g_yaw_angle_pid.ki * g_yaw_angle_pid.error_sum +
        g_yaw_angle_pid.kd * (g_yaw_angle_pid.error - g_yaw_angle_pid.error_prev);
        
        // 6. 对总输出进行限幅
        if (g_yaw_angle_pid.output > g_yaw_angle_pid.output_max) {
            g_yaw_angle_pid.output = g_yaw_angle_pid.output_max;
        } else if (g_yaw_angle_pid.output < g_yaw_angle_pid.output_min) {
            g_yaw_angle_pid.output = g_yaw_angle_pid.output_min;
        }
				
        
		}
        // ---【PID计算结束，后续混合逻辑与原来保持一致】---

        // 提取PID的最终输出作为转向修正量
        float correction_turn_speed = g_yaw_angle_pid.output;

        // 混合：将用户前进/后退指令与PID修正量混合
        // g_target_turn_speed 在该模式下通常为0，让PID完全接管转向
        float final_turn_component = g_target_turn_speed + correction_turn_speed;
        float final_speed_A = g_target_forward_speed + final_turn_component;
        float final_speed_B = g_target_forward_speed - final_turn_component;

        // 将新计算出的速度设为内环（速度环）目标
        Motor_SetTargetSpeed(final_speed_A, final_speed_B);
   

    // 无论角度环是否计算，速度环PID都必须在每个周期执行，以保证电机平稳运行
    PID_MotorSpeedControl();
}


void ControlLoop_YawStraight(void)
{
    // 这个函数的逻辑和 ControlLoop_YawCorrected 几乎一模一样,
    // 唯一的区别是它使用了 g_straight_assist_pid
    
    // 1. 获取目标值和当前值
    g_straight_assist_pid.target  = g_target_yaw_angle;
    g_straight_assist_pid.current = g_wit_data.yaw;

    // 2. 计算归一化误差
    float normalized_error = normalize_angle_error(g_straight_assist_pid.target, g_straight_assist_pid.current);
    
    // 3. 更新误差
    g_straight_assist_pid.error_prev = g_straight_assist_pid.error;
    g_straight_assist_pid.error = normalized_error;

    // 4. 计算积分项
    g_straight_assist_pid.error_sum += g_straight_assist_pid.error;
    if (g_straight_assist_pid.error_sum > g_straight_assist_pid.integral_max) {
        g_straight_assist_pid.error_sum = g_straight_assist_pid.integral_max;
    } else if (g_straight_assist_pid.error_sum < g_straight_assist_pid.integral_min) {
        g_straight_assist_pid.error_sum = g_straight_assist_pid.integral_min;
    }
    
    // 5. 计算PID总输出
    g_straight_assist_pid.output = 
        g_straight_assist_pid.kp * g_straight_assist_pid.error + 
        g_straight_assist_pid.ki * g_straight_assist_pid.error_sum +
        g_straight_assist_pid.kd * (g_straight_assist_pid.error - g_straight_assist_pid.error_prev);
    
    // 6. 输出限幅
    if (g_straight_assist_pid.output > g_straight_assist_pid.output_max) {
        g_straight_assist_pid.output = g_straight_assist_pid.output_max;
    } else if (g_straight_assist_pid.output < g_straight_assist_pid.output_min) {
        g_straight_assist_pid.output = g_straight_assist_pid.output_min;
    }

    // 7. 混合速度
    float correction_turn_speed = g_straight_assist_pid.output;
    float final_turn_component = g_target_turn_speed + correction_turn_speed; // g_target_turn_speed 通常为0
    float final_speed_A = g_target_forward_speed - final_turn_component;
    float final_speed_B = g_target_forward_speed + final_turn_component;

    // 8. 设置速度环目标
    Motor_SetTargetSpeed(final_speed_A, final_speed_B);

    // 9. 执行速度环PID
    PID_MotorSpeedControl();
}

