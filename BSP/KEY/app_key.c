#include "app_key.h" // 假设你使用了独立的app文件
#include "motor.h" // 假设你有电机驱动的头文件
#include "pid.h" 
#include "wit.h"

KeyAppMode_t g_current_mode = MODE_NORMAL_OPERATION; // 程序启动时，默认为正常模式

/**
 * @brief 统一的按键应用处理函数
 * @param key_value 经过驱动层消抖后传入的稳定键值
 * @retval None
 */
void app_key_handler(uint8_t key_value)
{
    if (key_value == 0) {
        return; // 没有有效按键，直接返回
    }

    // 使用 switch 语句，根据当前模式来决定按键的功能
		switch (g_current_mode)
    {
        // --- 模式 0: 正常运行模式 ---
        case MODE_NORMAL_OPERATION:
        {
            switch (key_value)
            {
                case 1: // 按键1: 启动
                    if(g_motor_state != MOTOR_STATE_IDLE) {
                        Motor_Stop();
                        g_motor_state = MOTOR_STATE_IDLE;
                    }
                    g_current_mode = MODE_TUNE_MOTOR_A; // 切换到下一个模式
                    break;
                case 2: // 按键2: 停止电机
                    Motor_Stop(); // 停止电机
                    g_motor_state = MOTOR_STATE_IDLE; // 切换到空闲状态
                    break;
								case 3: 
                    Motion_SetControlMode(MOTION_MODE_SPEED_ONLY); // 切换到纯速度模式
                    Motor_SetTargetSpeed(15,15); // 设置目标速度
                    if (g_motor_state == MOTOR_STATE_IDLE) {
                        // 如果是空闲，则准备启动
                        Motor_On(); // 启动电机
                        g_motor_state = MOTOR_STATE_RUNNING;
                    }
                    break;
								case 4: // 按键9: 自主右转90度
                {
                    Motor_On(); // 启动电机
                    PID_Reset(&g_motorA_speed_pid);
                    PID_Reset(&g_motorB_speed_pid);
                    PID_Reset(&g_yaw_angle_pid);
                    // 1. 确保处于正确的控制模式
                    Motion_SetControlMode(MOTION_MODE_YAW_CORRECTED);
                    // 2. 计算目标角度
                    float target_angle = g_wit_data.yaw - 90.0f;
                    // 3. 设置目标
                    Motion_Set_Target_Angle(target_angle);
                    // 4. 清除手动指令，让PID完全接管
                    Motion_Set_Speed(0, 0);
                    // 5. 如果未启动，则启动电机执行任务
                    if (g_motor_state == MOTOR_STATE_IDLE) {
                        g_motor_state = MOTOR_STATE_RUNNING;
                    }
                    break;
                }
						}
				}
				break;
				case MODE_TUNE_MOTOR_A:
        {
            // 这里放入你之前写的 motorA 调参逻辑
            switch (key_value)
            {
                case 1: // 按键1: 启动
                    if(g_motor_state != MOTOR_STATE_IDLE) {
                        Motor_Stop();
                        g_motor_state = MOTOR_STATE_IDLE;
                    }
                    g_current_mode = MODE_MAZE_TEST; // 切换到下一个模式
                    break;
                case 2: // 按键2: 停止电机
                    Motor_Stop(); // 停止电机
                    g_motor_state = MOTOR_STATE_IDLE; // 切换到空闲状态
                    break;
								case 3: 
                    WIT_ZeroYaw();
                    break;
								case 4: // 按键9: 自主右转90度
                {
                    Motor_On(); // 启动电机
                    PID_Reset(&g_motorA_speed_pid);
                    PID_Reset(&g_motorB_speed_pid);
                    PID_Reset(&g_yaw_angle_pid);
                    // 1. 确保处于正确的控制模式
                    Motion_SetControlMode(MOTION_MODE_YAW_CORRECTED);
                    // 2. 计算目标角度
                    float target_angle = g_wit_data.yaw + 90.0f;
                    // 3. 设置目标
                    Motion_Set_Target_Angle(target_angle);
                    // 4. 清除手动指令，让PID完全接管
                    Motion_Set_Speed(0, 0);
                    // 5. 如果未启动，则启动电机执行任务
                    if (g_motor_state == MOTOR_STATE_IDLE) {
                        g_motor_state = MOTOR_STATE_RUNNING;
                    }
                    break;
                }
						}
            
        }
				break;
				case MODE_MAZE_TEST:
        {
            switch (key_value)
            {
								case 1: // 按键1: 启动
                    if(g_motor_state != MOTOR_STATE_IDLE) {
                        Motor_Stop();
                        g_motor_state = MOTOR_STATE_IDLE;
                    }
                    g_current_mode = MODE_NORMAL_OPERATION; // 切换到下一个模式
                    break;
                
                case 2: // 按键2: 紧急停止 / 复位
                    Motor_Stop();
                    g_motor_state = MOTOR_STATE_IDLE;
                    // 这里不需要 Maze_Odom_Reset，下次启动 Start 函数会自动清零
                    break;
								
								case 3: // 按键1: 启动“写死路径”测试
                    // 这会复位里程、设置 PID 模式、开始走第一段直行
                    if (g_motor_state == MOTOR_STATE_IDLE) {
                        Maze_Hardcoded_Start(); 
                    }
                    break;
                              
                case 4: // (可选) 按键4: 仅重置里程，不跑
                     if (g_motor_state == MOTOR_STATE_IDLE) {
                        Maze_Test_OneCell_Start(); 
                    }
										else {
                        Motor_Stop();
                        g_motor_state = MOTOR_STATE_IDLE;
                    }
                    break;
                     break;
            }
        }
        break;
		}

}
