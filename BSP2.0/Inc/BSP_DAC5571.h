#pragma once
#ifndef __BSP_DAC5571_H
#define __BSP_DAC5571_H

#define DAC5571_WRITEADDR	0x98
#define DAC5571_READADDR	0x99

enum dac5571mode
{
	NormalOperationMode = 0x00U,
	OneKToAGNDMode		= 0x01U,
	TenKToAGNDMode		= 0x02U,
	HighImpedance		= 0x03U
};
typedef enum dac5571mode ModesOfOperationForDAC5571;
//Includes************************************************************************/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "bsp_Common.h"
#include "i2c.h"

#ifdef __cplusplus
extern "C"{
#endif

//Function**********************************************************************/
	
	
    BSP_StatusTypeDef BSP_DAC5571_Init(ModesOfOperationForDAC5571 mode);
    BSP_StatusTypeDef BSP_DAC5571_WriteValue(ModesOfOperationForDAC5571 mode,uint8_t vData);
    BSP_StatusTypeDef BSP_DAC5571_Check();
	
#ifdef __cplusplus
}
#endif // __cplusplus
#endif //__BSP_DAC5571_H

