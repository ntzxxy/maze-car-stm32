#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

extern uint8_t g_motorA_dir;
extern uint8_t g_motorB_dir;
extern int g_encoder_A;
extern int g_encoder_B;
extern int speed_A_fu; // Variable to store speed of motor A
extern int speed_B_fu; // Variable to store speed of motor B
extern int speed_A; // Variable to store speed of motor A
extern int speed_B; // Variable to store speed of motor B

void Motor_Init(void);
void Motor_On(void);
void Motor_Stop(void);
void MotorA_Duty(float duty);
void MotorB_Duty(float duty);
void Encoder_Init(void);
void Encoder_GetValue(void);

// --- 电机状态枚举 ---
typedef enum {
    MOTOR_STATE_IDLE,       // 空闲
    MOTOR_STATE_STARTING,   // 启动中 (如果需要渐进启动)
    MOTOR_STATE_RUNNING     // 运行中
} MotorState_t;

extern MotorState_t g_motor_state ;

#endif
