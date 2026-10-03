/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "oled.h"
#include "stdio.h"
#include "app_key.h"
#include "pid.h"
#include "math.h"
#include "wit.h"
#include "VL6180X.h"
#include "maze.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t oled_buffer[32]; // Declare the buffer for OLED display
uint8_t key_val = 0;
uint32_t Uart_Test = 0;
uint8_t uart_buffer[64]; 
uint8_t range_front = 0;
uint8_t range_left = 0;
uint8_t range_right = 0;
uint8_t g_need_read_tof;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void OLED_Display(void);
int fputc(int ch, FILE *f) 
{ 
HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 10); 
	return ch; 
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	Motor_Init();
	Encoder_Init();
	OLED_Init(); // Initialize OLED display
  OLED_Clear(); // Clear the OLED display
	WIT_Init(); // Initialize the WIT sensor
	VL6180X_Init_3Modules();
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	Motor_On();
	PID_Motor_Init(); // Initialize PID for motors
	HAL_TIM_Base_Start_IT(&htim4); // Start the timer for speed calculation
	/*
	// 1. 检查设备是否应答
  if (HAL_I2C_IsDeviceReady(&hi2c2, 0x29 << 1, 2, 50) == HAL_OK)
      printf("Device 0x29 OK\r\n");
  else {
      printf("Device 0x29 NOT found!\r\n");
      while (1);
  }

  // 2. 读取芯片 ID（期望 0xB4）
  uint8_t id = VL6180X_Read_ID(0x29);
  printf("Chip ID = 0x%02X\r\n", id);
  HAL_Delay(50);

  // 3. 初始化（加载寄存器表）
  if (VL6180X_Init(0x29) == 0)
      printf("VL6180X init OK\r\n");
  else
      printf("VL6180X init FAIL\r\n");
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		key_val = key_get_value(); // Get the key value
    app_key_handler(key_val);
		OLED_Display();
		if (g_need_read_tof) {
        g_need_read_tof = 0;
			/*
        uint8_t d_front = VL6180X_Read_Range(0x29); // Front：7bit=0x29 → 8bit=0x52
				uint8_t d_left  = VL6180X_Read_Range(0x2A); // Left ：7bit=0x2A → 8bit=0x54
				uint8_t d_right = VL6180X_Read_Range(0x2B); // Right：7bit=0x2B → 8bit=0x56

    printf("F=%3u  L=%3u  R=%3u mm\r\n", d_front, d_left, d_right);
        // 如需调试：printf("Front: %u mm\r\n", range_front);
			*/
			int rc;

			rc = VL6180X_ReadRange_Safe(0x29, &range_front);
			if (rc != 0) range_front = 255;  // 读失败就当很远

			rc = VL6180X_ReadRange_Safe(0x2A, &range_left);
			if (rc != 0) range_left = 255;

			rc = VL6180X_ReadRange_Safe(0x2B, &range_right);
			if (rc != 0) range_right = 255;

			// 把一次三方向原始数据“喂”给迷宫模块做滤波、判墙
			Maze_OnTofUpdate(range_front, range_left, range_right);
			if (rc != 0) range_right = 255;
			printf("F=%3u  L=%3u  R=%3u mm\r\n", range_front, range_left, range_right);
    }
		if (g_current_mode == MODE_MAZE_TEST) 
    {
        Maze_Hardcoded_Task(); 
    }
		//Maze_Task();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4){
    static uint16_t count = 0;
    static uint16_t line_count = 0;
    static int16_t last_raw_A = 0;     // 上次的编码器A原始值
    static int16_t last_raw_B = 0;     // 上次的编码器B原始值
    
    if(++count >= 20)
  {
        count = 0;
        //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2); // Toggle an LED for debugging
				
        // 获取当前编码器值
        Encoder_GetValue();

        // 计算速度 (每个采样周期的位移)
        speed_A_fu = g_encoder_A - last_raw_A;
        speed_B = g_encoder_B - last_raw_B;
        speed_A = -speed_A_fu;
        // 处理速度计算时的溢出情况
        if(speed_A > 32768) speed_A -= 65536;
        else if(speed_A < -32768) speed_A += 65536;
        
        if(speed_B > 32768) speed_B -= 65536;
        else if(speed_B < -32768) speed_B += 65536;
		
				Maze_Odom_Update((int16_t)speed_A, (int16_t)speed_B);
		
        
        // 更新上次值，为下次计算做准备
        last_raw_A = g_encoder_A;
        last_raw_B = g_encoder_B;
        
				switch (g_motor_state)
        {
            case MOTOR_STATE_IDLE:
                // 空闲状态，什么也不做
                break;
            
            case MOTOR_STATE_RUNNING:
                // 运行状态，执行PID闭环控制
                // 注意: 此处应该调用你封装好的PID控制函数
                // --- 【核心修改】根据模式调用不同的控制循环 ---
                switch(g_motion_mode)
                {
                    case MOTION_MODE_SPEED_ONLY:
                        ControlLoop_SpeedOnly();
                        break;
                    case MOTION_MODE_DIFFERENTIAL:
                        ControlLoop_Differential();
                        break;
										 case MOTION_MODE_YAW_CORRECTED:
                        ControlLoop_YawCorrected();
                        break;
                    case MOTION_MODE_YAW_STRAIGHT:
                        ControlLoop_YawStraight();
                        break;
                }
                break;
        }
        
    }
		if(++line_count >= 50)
		{
			line_count = 0;
			g_need_read_tof = 1;   // 通知主循环去读
		}
  }
}

void OLED_Display(void)
{
  static uint32_t oled_ms = 0;
  static char oled_buffer[32]; // ���� sprintf �Ļ�����

   // 这个静态变量用来记录上次显示时所处的模式
  static KeyAppMode_t last_display_mode = -1; // 初始化为一个无效模式
    if(HAL_GetTick() - oled_ms >= 200) // 每100ms更新一次OLED
    {
        oled_ms = HAL_GetTick();
        
        // **核心技巧: 如果模式发生了变化，就清屏一次**
        if (g_current_mode != last_display_mode) {
          OLED_Clear(); // 清除屏幕，防止内容重叠
          last_display_mode = g_current_mode; // 更新模式记录
      }
      
      // 根据当前模式，显示不同的界面
      switch (g_current_mode)
      {
          case MODE_NORMAL_OPERATION:
          {
						
							if (g_motion_mode == MOTION_MODE_YAW_CORRECTED)
              {                 
                  OLED_ShowString(0, 0, "Yaw:Turn", 16);
              }
              // 【新增】为新模式增加显示
              else if (g_motion_mode == MOTION_MODE_YAW_STRAIGHT)
              {
                  OLED_ShowString(0, 0, "Yaw:Straight", 16);
              }
              else if (g_motion_mode == MOTION_MODE_DIFFERENTIAL)
              {
                  OLED_ShowString(0, 0, "DIFFERENTIAL", 16);
              }
              else if(g_motion_mode == MOTION_MODE_SPEED_ONLY) {
                OLED_ShowString(0, 0, "SPEED_ONLY", 16); // 显示当前模式
              }
							
              // 这是你的“正常运行模式”，显示速度
              //OLED_ShowString(0, 0, "Mode: Normal", 16); // 显示当前模式
              sprintf(oled_buffer, "SpA:%3d", speed_A);
              OLED_ShowString(0, 2, (uint8_t*)oled_buffer, 16);
              sprintf(oled_buffer, "SpB:%3d", speed_B);
              OLED_ShowString(65, 2, (uint8_t*)oled_buffer, 16);
              sprintf(oled_buffer, "Yaw:%6.2f", g_wit_data.yaw);
              OLED_ShowString(0, 4, (uint8_t*)oled_buffer, 16);
							
							sprintf(oled_buffer, "R%3d L%3d F%3d", range_right, range_left, range_front);
							OLED_ShowString(0, 6, (uint8_t*)oled_buffer, 16);
							
							
							
              break;
          }

          case MODE_TUNE_MOTOR_A:
          {
              // 这是“电机A调参模式”，显示PID参数
              OLED_ShowString(0, 0, "Tune Motor A", 16);
              if (g_motion_mode == MOTION_MODE_YAW_CORRECTED)
              {                 
                  OLED_ShowString(0, 2, "Yaw:Turn", 16);
              }
              // 【新增】为新模式增加显示
              else if (g_motion_mode == MOTION_MODE_YAW_STRAIGHT)
              {
                  OLED_ShowString(0, 2, "Yaw:Straight", 16);
              }
              else if (g_motion_mode == MOTION_MODE_DIFFERENTIAL)
              {
                  OLED_ShowString(0, 2, "DIFFERENTIAL", 16);
              }
              else if(g_motion_mode == MOTION_MODE_SPEED_ONLY) {
                OLED_ShowString(0, 2, "SPEED_ONLY", 16); // 显示当前模式
              }
							sprintf(oled_buffer, "Yaw:%6.2f", g_wit_data.yaw);
              OLED_ShowString(0, 4, (uint8_t*)oled_buffer, 16);
							sprintf(oled_buffer, "P%.2f", g_yaw_angle_pid.kp);
              OLED_ShowString(0, 6, (uint8_t*)oled_buffer, 16);
              sprintf(oled_buffer, "I%.2f", g_yaw_angle_pid.ki);
              OLED_ShowString(40, 6, (uint8_t*)oled_buffer, 16);
              sprintf(oled_buffer, "D%.2f", g_yaw_angle_pid.kd);
              OLED_ShowString(80, 6, (uint8_t*)oled_buffer, 16);
              break;
          }

          case MODE_MAZE_TEST:
          {
              // “电机B调参模式”
              OLED_ShowString(0, 0, "Maze", 16);
							sprintf(oled_buffer, "TF:%1d", g_turn_complete);
              OLED_ShowString(50, 0, (uint8_t*)oled_buffer, 16);
              sprintf(oled_buffer, "SpA:%3d", speed_A);
              OLED_ShowString(0, 2, (uint8_t*)oled_buffer, 16);
              sprintf(oled_buffer, "SpB:%3d", speed_B);
              OLED_ShowString(65, 2, (uint8_t*)oled_buffer, 16);
              sprintf(oled_buffer, "Yaw:%6.2f", g_wit_data.yaw);
              OLED_ShowString(0, 4, (uint8_t*)oled_buffer, 16);
							
							sprintf(oled_buffer, "R%3d L%3d F%3d", range_right, range_left, range_front);
							OLED_ShowString(0, 6, (uint8_t*)oled_buffer, 16);
              break;
          }

         

         
          default:
          {
              // 默认情况，可以显示一个错误或未知状态
              OLED_ShowString(0, 0, "Unknown Mode!", 16);
              break;
          }
      }
    }   
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
