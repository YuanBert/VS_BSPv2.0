#ifndef __BSP_REMOTECONTROL_H
#define __BSP_REMOTECONTROL_H

#ifdef __cplusplus
extern "C"
{
#endif
#include "main.h"
#include "gpio.h"
#include "bsp_common.h"

#define   MCU_WIRLESSOPEN_Pin			MCU_WIRLESSB_Pin
#define   MCU_WIRLESSOPEN_GPIO_Port		MCU_WIRLESSB_GPIO_Port
#define   MCU_WIRLESSCLOSE_Pin			MCU_WIRLESSA_Pin
#define   MCU_WIRLESSCLOSE_GPIO_Port	MCU_WIRLESSA_GPIO_Port
#define   MCU_WIRLESSSTOP_Pin			MCU_WIRLESS_Pin
#define   MCU_WIRLESSSTOP_GPIO_Port		MCU_WIRLESS_GPIO_Port

	struct tWIRLESS
	{
		uint8_t OpenFlag;
		uint8_t CloseFlag;
		uint8_t STopFlag;
	};
	
	typedef struct tWIRLESS WIRLESS;

BSP_StatusTypeDef BSP_RemoteControlInit(void);

//#define MCU_WIRLESSB_Pin GPIO_PIN_5
//#define MCU_WIRLESSB_GPIO_Port GPIOB
//#define MCU_WIRLESSA_Pin GPIO_PIN_6
//#define MCU_WIRLESSA_GPIO_Port GPIOB
//#define MCU_WIRLESS_Pin GPIO_PIN_7
//#define MCU_WIRLESS_GPIO_Port GPIOB

	
	
#ifdef __cplusplus	
}
#endif

#endif	/**/