#ifndef __VL6180X_H__
#define __VL6180X_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "i2c.h"
#include <stdint.h>
#include <stdio.h>
#include "VL6180X_Reg.h"

// ============================================================
// I2C 地址定义（7-bit）
// ============================================================
#define VL6180X_ADDRESS_7BIT          (0x29U)  // 默认地址
#define VL6180X_I2C_ADDRESS_L_7BIT    (0x2A)   // Left 模块新地址
#define VL6180X_I2C_ADDRESS_R_7BIT    (0x2B)   // Right 模块新地址
#define VL6180X_I2C_ADDRESS_8BIT(x)   ((uint16_t)((x) << 1))

// ============================================================
// GPIO 引脚定义（对应三个模块的 SHDN 引脚）
// ============================================================
// PB0 = Front，PB2 = Left，PB1 = Right
// PB0 = Front, PB1 = Right,  PA5 = Left
#define SHDN_FRONT_PIN        GPIO_PIN_0
#define SHDN_FRONT_GPIO_PORT  GPIOB

#define SHDN_RIGHT_PIN        GPIO_PIN_11
#define SHDN_RIGHT_GPIO_PORT  GPIOA
#define SHDN_LEFT_PIN         GPIO_PIN_4
#define SHDN_LEFT_GPIO_PORT   GPIOA

// ============================================================
// 外部句柄与状态变量
// ============================================================
extern I2C_HandleTypeDef hi2c2;
extern uint8_t ptp_offset;

// ============================================================
// 函数声明
// ============================================================

/**
 * @brief  向 16 位寄存器写入 8 位数据
 */
void     VL6180X_WriteByte(uint8_t add, uint16_t reg, uint8_t data);

/**
 * @brief  向 16 位寄存器写入 16 位数据（高字节在前）
 */
void     VL6180X_WriteByte_16Bit(uint8_t add, uint16_t reg, uint16_t data);

/**
 * @brief  从 16 位寄存器读取 8 位数据
 */
uint8_t  VL6180X_ReadByte(uint8_t add, uint16_t reg);

/**
 * @brief  从 16 位寄存器读取 16 位数据（高字节在前）
 */
uint16_t VL6180X_ReadBytee_16Bit(uint8_t add, uint16_t reg);

/**
 * @brief  设备初始化（写入推荐寄存器）
 * @return 0 = 成功；1 = 失败
 */
uint8_t  VL6180X_Init(uint8_t add);

/**
 * @brief  读取芯片 ID（期望返回 0xB4）
 */
uint8_t  VL6180X_Read_ID(uint8_t add);

/**
 * @brief  单次测距读取（单位 mm）
 */
uint8_t  VL6180X_Read_Range(uint8_t add);

/**
 * @brief  一次性初始化三模块（Front=0x29，Left=0x2A，Right=0x2B）
 */
void VL6180X_Init_3Modules(void);

// ============================================================
// 可选：等待上电自检完成函数（扩展用）
// ============================================================
int VL6180X_WaitBoot(uint8_t add7, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __VL6180X_H__ */
