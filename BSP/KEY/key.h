/**
 * @file key.h
 * @author 
 * @brief °´¼ü¶ÁÈ¡
 * @version 0.1
 * @date 2025-07-15
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef __KEY_H
#define __KEY_H

#include <stdint.h>
#include "main.h"

uint8_t Key_GetNum(void);
uint8_t Matrix_Key_Scan(void);       //¾ØÕó°´¼üÉ¨Ãè
uint8_t key_get_value(void);

#endif
