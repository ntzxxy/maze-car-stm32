#include "VL6180X.h"
#include "i2c.h" // 包含 I2C HAL 句柄声明
#include <string.h> // 需要包含 string.h
#include <stdio.h>  // 需要包含 stdio.h
#include "Delay.h"

// 外部引用 I2C1 句柄（假设 CubeMX 生成的句柄名为 hi2c2）
extern I2C_HandleTypeDef hi2c2; 

void VL6180X_WriteByte(uint8_t add,uint16_t reg,uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c2 ,(add<<1)|0,reg,I2C_MEMADD_SIZE_16BIT,&data,1,0xffff);
	
}

void VL6180X_WriteByte_16Bit(uint8_t add,uint16_t reg,uint16_t data)
{
	uint8_t data2[2]={0,0};
	data2[0]=data>>8;
	data2[1]=data;
	HAL_I2C_Mem_Write(&hi2c2 ,(add<<1)|0,reg,I2C_MEMADD_SIZE_16BIT,data2,2,0xffff);
	
}

uint8_t VL6180X_ReadByte(uint8_t add,uint16_t reg)
{
	uint8_t data=0;
	HAL_I2C_Mem_Read(&hi2c2 ,(add<<1)|1,reg,I2C_MEMADD_SIZE_16BIT,&data,1,0xffff);
	return data;
}

uint16_t VL6180X_ReadBytee_16Bit(uint8_t add,uint16_t reg)
{
	uint16_t data=0;
	uint8_t data2[2];
	HAL_I2C_Mem_Read(&hi2c2 ,(add<<1)|1,reg,I2C_MEMADD_SIZE_16BIT,data2,2,0xffff);
	data=data2[0];
	data=data<<8;
	data+=data2[1];
	
	return data;

}


uint8_t ptp_offset;
uint8_t VL6180X_Init(uint8_t add)
{
	ptp_offset=VL6180X_ReadByte(add,VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET);
	printf("ptp_offset=%d\n",ptp_offset);
//	if(VL6180X_Read_ID(add) == VL6180X_DEFAULT_ID)
	if(ptp_offset==0x01)
	{	
		VL6180X_WriteByte(add,0x0207, 0x01);
		VL6180X_WriteByte(add,0x0208, 0x01);
		VL6180X_WriteByte(add,0x0096, 0x00);
		VL6180X_WriteByte(add,0x0097, 0xfd);
		VL6180X_WriteByte(add,0x00e3, 0x00);
		VL6180X_WriteByte(add,0x00e4, 0x04);
		VL6180X_WriteByte(add,0x00e5, 0x02);
		VL6180X_WriteByte(add,0x00e6, 0x01);
		VL6180X_WriteByte(add,0x00e7, 0x03);
		VL6180X_WriteByte(add,0x00f5, 0x02);
		VL6180X_WriteByte(add,0x00d9, 0x05);
		VL6180X_WriteByte(add,0x00db, 0xce);
		VL6180X_WriteByte(add,0x00dc, 0x03);
		VL6180X_WriteByte(add,0x00dd, 0xf8);
		VL6180X_WriteByte(add,0x009f, 0x00);
		VL6180X_WriteByte(add,0x00a3, 0x3c);
		VL6180X_WriteByte(add,0x00b7, 0x00);
		VL6180X_WriteByte(add,0x00bb, 0x3c);
		VL6180X_WriteByte(add,0x00b2, 0x09);
		VL6180X_WriteByte(add,0x00ca, 0x09);
		VL6180X_WriteByte(add,0x0198, 0x01);
		VL6180X_WriteByte(add,0x01b0, 0x17);
		VL6180X_WriteByte(add,0x01ad, 0x00);
		VL6180X_WriteByte(add,0x00ff, 0x05);
		VL6180X_WriteByte(add,0x0100, 0x05);
		VL6180X_WriteByte(add,0x0199, 0x05);
		VL6180X_WriteByte(add,0x01a6, 0x1b);
		VL6180X_WriteByte(add,0x01ac, 0x3e);
		VL6180X_WriteByte(add,0x01a7, 0x1f);
		VL6180X_WriteByte(add,0x0030, 0x00);
		
		// Recommended : Public registers - See data sheet for more detail
		VL6180X_WriteByte(add,0x0011, 0x10);       // Enables polling for 'New Sample ready'
									// when measurement completes
		VL6180X_WriteByte(add,0x010a, 0x30);       // Set the averaging sample period
									// (compromise between lower noise and
									// increased execution time)
		VL6180X_WriteByte(add,0x003f, 0x46);       // Sets the light and dark gain (upper
									// nibble). Dark gain should not be
									// changed. !上半字节要写入0x4	默认增益是1.0
		VL6180X_WriteByte(add,0x0031, 0xFF);       // sets the # of range measurements after
									// which auto calibration of system is
									// performed
		VL6180X_WriteByte(add,0x0041, 0x63);       // Set ALS integration time to 100ms
		VL6180X_WriteByte(add,0x002e, 0x01);       // perform a single temperature calibration
									// of the ranging sensor

		// Optional: Public registers - See data sheet for more detail
		VL6180X_WriteByte(add,0x001b, 0x09);    //测量间隔	轮询模式
									// period to 100ms	每步10ms->0-10ms
		VL6180X_WriteByte(add,0x003e, 0x31);      //测量周期	ALS模式
									// to 500ms		
		VL6180X_WriteByte(add,0x0014, 0x24);       // Configures interrupt on 'New Sample
									// Ready threshold event'


//VL6180X_WriteByte(add,VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00); //不发送00那么读出来的数值就是01

		return 0;
	}
	else return 1;
}

uint8_t VL6180X_Read_ID(uint8_t add)
{
	return VL6180X_ReadByte(add,VL6180X_REG_IDENTIFICATION_MODEL_ID);
}

uint8_t VL6180X_Read_Range(uint8_t add)
{
	uint8_t range = 0;
	//等待设备准备好进行量程测量
	while(!(VL6180X_ReadByte(add,VL6180X_REG_RESULT_RANGE_STATUS) & 0x01));//VL6180X_REG_RESULT_RANGE_STATUS，0x0d， 自检0x01连续性测试
	//开始量程测量
	VL6180X_WriteByte(add,VL6180X_REG_SYSRANGE_START,0x01);//VL6180X_REG_SYSRANGE_START,0x018,开启测距，0x01，单次模
	//轮询到第2位被设置
	while(!(VL6180X_ReadByte(add,VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04));//VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO,中断等待，0x04，数据等待完毕
	//读数范围(毫米)
	range = VL6180X_ReadByte(add,VL6180X_REG_RESULT_RANGE_VAL);//RESULT__RANGE_VAL,0x062,显示检测长度
	//清除中断
	VL6180X_WriteByte(add,VL6180X_REG_SYSTEM_INTERRUPT_CLEAR,0x07);//VL6180X_REG_SYSTEM_INTERRUPT_CLEAR,0x015,清除状态位
	return range;
}

int VL6180X_ReadRange_Safe(uint8_t add7, uint8_t *mm) {
    uint16_t dev8 = (uint16_t)(add7 << 1);

    // 1) 触发一次测距
    if (HAL_I2C_Mem_Write(&hi2c2, dev8, 0x0018, I2C_MEMADD_SIZE_16BIT,
                          (uint8_t[]){0x01}, 1, 20) != HAL_OK) return -1;

    // 2) 等“新样本就绪”(RESULT__INTERRUPT_STATUS_GPIO bit2)，最多 50ms
    uint32_t t0 = HAL_GetTick();
    while (1) {
        uint8_t istat = 0;
        if (HAL_I2C_Mem_Read(&hi2c2, dev8, 0x004F, I2C_MEMADD_SIZE_16BIT,
                             &istat, 1, 20) != HAL_OK) return -2;
        if (istat & 0x04) break;
        if (HAL_GetTick() - t0 > 50) return -3;
    }

    // 3) 读距离
    if (HAL_I2C_Mem_Read(&hi2c2, dev8, 0x0062, I2C_MEMADD_SIZE_16BIT,
                         mm, 1, 20) != HAL_OK) return -4;

    // 4) 清中断
    if (HAL_I2C_Mem_Write(&hi2c2, dev8, 0x0015, I2C_MEMADD_SIZE_16BIT,
                          (uint8_t[]){0x07}, 1, 20) != HAL_OK) return -5;

    return 0;
}

int VL6180X_WaitBoot(uint8_t add7, uint32_t timeout_ms) {
    uint32_t t0 = HAL_GetTick();
    while (1) {
        uint8_t v = VL6180X_ReadByte(add7, VL6180X_REG_SYSTEM_FRESH_OUT_OF_RESET);
        if (v == 0x01) return 0;
        if (HAL_GetTick() - t0 > timeout_ms) return -1;
        HAL_Delay(1);
    }
}

// ---- 在你的初始化阶段调用一次 ----
 void VL6180X_Init_3Modules(void)
{
    /* === 三模块初始化（用你的宏/变量名）=== */
		HAL_GPIO_WritePin(SHDN_FRONT_GPIO_PORT, SHDN_FRONT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHDN_RIGHT_GPIO_PORT, SHDN_RIGHT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHDN_LEFT_GPIO_PORT,  SHDN_LEFT_PIN,  GPIO_PIN_RESET);
		uint8_t ii;
		uint8_t id;
		HAL_Delay(10);
	
		// === 1) 只开 Left，把默认 0x29 改到 0x2A ===
    HAL_GPIO_WritePin(SHDN_LEFT_GPIO_PORT, SHDN_LEFT_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    if (VL6180X_WaitBoot(0x29, 30) == 0) {
        VL6180X_Init(0x29);
        VL6180X_WriteByte(0x29, 0x0212, VL6180X_I2C_ADDRESS_L_7BIT); // ->0x2A
        HAL_Delay(2);
		}
				/*
        ii  = VL6180X_ReadByte(VL6180X_I2C_ADDRESS_L_7BIT, 0x0212);
        id  = VL6180X_Read_ID(VL6180X_I2C_ADDRESS_L_7BIT);
        printf("Left  addr=0x%02X  id=0x%02X\r\n", ii, id);
    } else {
        printf("Left boot timeout\r\n");
    }
		*/	
    // === 2) 再开 Right，把默认 0x29 改到 0x2B ===
    HAL_GPIO_WritePin(SHDN_RIGHT_GPIO_PORT, SHDN_RIGHT_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    if (VL6180X_WaitBoot(0x29, 30) == 0) {
        VL6180X_Init(0x29);
        VL6180X_WriteByte(0x29, 0x0212, VL6180X_I2C_ADDRESS_R_7BIT); // ->0x2B
        HAL_Delay(2);
		}
			/*
        ii  = VL6180X_ReadByte(VL6180X_I2C_ADDRESS_R_7BIT, 0x0212);
        id  = VL6180X_Read_ID(VL6180X_I2C_ADDRESS_R_7BIT);
        printf("Right addr=0x%02X  id=0x%02X\r\n", ii, id);
    } else {
        printf("Right boot timeout\r\n");
    }
			*/

    // === 3) 最后开 Front，保持 0x29 ===
    HAL_GPIO_WritePin(SHDN_FRONT_GPIO_PORT, SHDN_FRONT_PIN, GPIO_PIN_SET);
    HAL_Delay(5);
    if (VL6180X_WaitBoot(0x29, 30) == 0) {
        VL6180X_Init(0x29);
			/*
        ii  = VL6180X_ReadByte(VL6180X_ADDRESS_7BIT, 0x0212);
        id  = VL6180X_Read_ID(VL6180X_ADDRESS_7BIT);
        printf("Front addr=0x%02X  id=0x%02X\r\n", ii, id);
    } else {
        printf("Front boot timeout\r\n");
    }
			*/
		}
}