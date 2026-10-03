/*
 * WIT陀螺仪驱动 - STM32F407 HAL库版本
 * 
 * HAL配置说明:
 *   UART配置 (使用USART2):
 *     1. 波特率: 根据传感器模块配置调整
 *     2. 数据位: 8位
 *     3. 停止位: 1位
 *     4. 校验位: 无
 *     5. 硬件流控制: 无
 *     6. 接收中断: 使能UART接收中断
 *
 *   DMA配置:
 *     1. 使用DMA1_Stream5_Channel4 (USART2_RX)
 *     2. 传输方向: 外设到存储器
 *     3. 外设数据对齐: 字节
 *     4. 存储器数据对齐: 字节
 *     5. 模式: 循环模式
 *     6. 优先级: 中等
 *     7. 使能DMA传输完成中断
 *
 *   引脚连接:
 *     - USART2_RX: PA3 (连接WIT传感器TX)
 *     - VCC: 3.3V 或 5V (根据传感器模块要求)
 *     - GND: GND
 * 
 * 使用说明:
 *   1. 在main.c中包含该头文件
 *   2. 在主循环前调用WIT_Init()初始化
 *   3. 通过全局变量wit_data访问传感器数据
 *   4. 数据自动通过DMA和中断更新
 *   5. 修改stm32f4xx_it.c，在USART2_IRQHandler中先调用WIT_UART_IRQHandler
 */

#ifndef __WIT_H
#define __WIT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>

// WIT传感器数据结构体
typedef struct {
    float pitch;        // 俯仰角 (°)
    float roll;         // 横滚角 (°)
    float yaw;          // 偏航角 (°)
    float temperature;  // 温度 (°C)
    float ax;           // X轴加速度 (g)
    float ay;           // Y轴加速度 (g)
    float az;           // Z轴加速度 (g)
    float gx;           // X轴角速度 (°/s)
    float gy;           // Y轴角速度 (°/s)
    float gz;           // Z轴角速度 (°/s)
    int16_t version;    // 版本信息
} WIT_Data_t;

// 外部声明
extern UART_HandleTypeDef huart1;  // USART1句柄，需在main.c中定义
extern DMA_HandleTypeDef hdma_usart1_rx;  // DMA句柄，需在main.c中定义
extern volatile WIT_Data_t g_wit_data;

// 调试变量
extern volatile uint32_t wit_debug_rx_count;
extern volatile uint32_t wit_debug_valid_packet_count;

// 函数声明
void WIT_Init(void);
void WIT_UART_IRQHandler(void);
void WIT_GetDebugInfo(uint32_t *rx_count, uint32_t *valid_packet_count);
uint16_t WIT_GetRawBuffer(uint8_t *buffer, uint16_t size);
void WIT_ZeroYaw(void); // 发送命令清零航向角

#endif /* __WIT_H */
