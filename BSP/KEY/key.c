#include "key.h"


uint8_t Independent_Key_Scan(void) // 独立按键扫描
{
 uint8_t current_raw_key = 0;

 // 独立按键不需要行扫描的复杂操作，直接读取 4 个引脚状态即可
 // 假设按键为低电平有效（按下时读取到 RESET）

 if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
		current_raw_key = 1;
 } else if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
		current_raw_key = 2;
 } else if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == GPIO_PIN_RESET) {
		current_raw_key = 3;
 } else if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == GPIO_PIN_RESET) {
		current_raw_key = 4;
 }
    
	 return current_raw_key;
}

uint8_t key_get_value(void)
{
    static uint8_t key_state = 0;
    static uint8_t key_raw = 0, key_final = 0;
    static uint16_t debounce_timer = 0;

    key_raw = Independent_Key_Scan();

    switch (key_state) {
        case 0:
            if (key_raw != 0) {
                key_final = key_raw;
                debounce_timer = 0;
                key_state = 1;
            }
            break;
        case 1:
            if (key_raw == key_final) {
                debounce_timer++;
                // 消抖时间可以根据调用频率调整
                if (debounce_timer >= 1	) {
                    key_state = 2;
                }
            } else {
                key_state = 0;
            }
            break;
        case 2:
            if (key_raw == 0) {
                key_state = 0;
                return key_final;
            }
            break;
    }
    return 0;
}

uint8_t key_mode = 0;

/*
void Key_Proc(void)
{
		static uint8_t Key_Val_Old = 18;
				
				
        uint8_t key_val = Key_GetNum();

        if (key_val == Key_Val_Old)
            return;
        if (key_mode == 0) 
				{ // 进入过渡模式
            switch (key_val) 
							{
							case 0:
									key_mode = 1;
							break;
							case 1:
									servox_angle += 1;
							break;
							case 2:
									servox_angle -= 1;
							break;
							}
				}
				else if (key_mode == 1) 
				{ // 进入过渡模式
            switch (key_val) 
							{
							case 0:
									key_mode = 0;
							break;
							case 1:
									servoy_angle += 1;
							break;
							case 2:
									servoy_angle -= 1;
							break;
							}
				}
				Servo_SetAngle_X(servox_angle);
				Servo_SetAngle_Y(servoy_angle);
				Key_Val_Old = key_val;
		
}
*/
