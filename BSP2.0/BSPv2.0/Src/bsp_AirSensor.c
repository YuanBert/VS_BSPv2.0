#include "bsp_AirSensor.h"

extern AirSensor gAirSensor;

BSP_StatusTypeDef BSP_AirSensor_Init(uint8_t nFilterSum)
{
	BSP_StatusTypeDef state = BSP_OK;
	gAirSensor.FilterCntSum = nFilterSum;
	gAirSensor.FilterCnt =  0;
	gAirSensor.CheckedFlag = 0;	
	
	return state;
}

/**********************************END OF FILE ***************************************************/
