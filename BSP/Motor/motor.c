#include "motor.h"
#include "tim.h"
#include "gpio.h"

#define PWM_MAX_DUTY     (100)
#define MOTOR_TIM        htim1  

uint8_t g_motorA_dir = 0;
uint8_t g_motorB_dir = 0;
int g_encoder_A = 0;
int g_encoder_B = 0;
int speed_A = 0; // Variable to store speed of motor A
int speed_B = 0; // Variable to store speed of motor B
int speed_A_fu; // Variable to store speed of motor A
int speed_B_fu; // Variable to store speed of motor B
MotorState_t g_motor_state = MOTOR_STATE_IDLE;

void Motor_Init(void)
{
    // 启动定时器
    HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_1);  // ���A PWM
    HAL_TIM_PWM_Start(&MOTOR_TIM, TIM_CHANNEL_2);  // ���B PWM
}

void Motor_On(void)
{
    HAL_GPIO_WritePin(GPIOA, STBY_Pin, GPIO_PIN_SET);
}

void Motor_Stop(void)
{
    // 关闭使能
    HAL_GPIO_WritePin(GPIOA, STBY_Pin, GPIO_PIN_RESET);
    
    // 设置所有控制引脚为低电平
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
  /* 
    // 复位PID
    pid_reset(&g_motorA);
    pid_reset(&g_motorB);
	*/
}

static void Set_Duty(float duty, uint8_t channel)
{
    uint32_t compare = (uint32_t)(duty -1);
    
    if(channel == 0) {
        __HAL_TIM_SET_COMPARE(&MOTOR_TIM, TIM_CHANNEL_1, compare);
    }
    else if(channel == 1) {
        __HAL_TIM_SET_COMPARE(&MOTOR_TIM, TIM_CHANNEL_2, compare);
    }
}

void MotorA_Duty(float duty)		//注意PCB画反了，这边补救
{
    Set_Duty(duty, 0);
    if(g_motorA_dir == 1) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    }
    else if(g_motorA_dir == 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
}

void MotorB_Duty(float duty)
{
    Set_Duty(duty, 1);
    if(g_motorB_dir == 1) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    }
    else if(g_motorB_dir == 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
}

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    g_encoder_A = 0;
    g_encoder_B = 0;
}

void Encoder_GetValue(void)
{
    // 获取编码器计数值
    g_encoder_A = __HAL_TIM_GET_COUNTER(&htim3);
    g_encoder_B = __HAL_TIM_GET_COUNTER(&htim2);
    
}
