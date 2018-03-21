#include "bsp_Log.h"
#include "bsp_DataTransmissionLayer.h"

static uint8_t DoorBoardInfo[24];
static uint8_t DataChangedFlag;

static uint8_t getXORCode(uint8_t* pData, uint16_t len)
{
	uint8_t ret;
	uint16_t i;
	ret = pData[0];
	for (i = 1; i < len; i++)
	{
		ret ^= pData[i];
	}
	return ret;
}

BSP_StatusTypeDef  BSP_Log_Init(uint32_t id)
{
	BSP_StatusTypeDef state = BSP_OK;
	/*写入设备ID*/
	DoorBoardInfo[0] =(uint8_t)(id>>24);
	DoorBoardInfo[1] = (uint8_t)(id >> 16);
	DoorBoardInfo[2] = (uint8_t)(id >> 8);
	DoorBoardInfo[3] = (uint8_t)id;
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpICurrentValue(uint16_t iValue)
{
	BSP_StatusTypeDef state = BSP_OK;
	/*写入峰值电流*/
	DoorBoardInfo[4] = (uint8_t)(iValue >> 8);
	DoorBoardInfo[5] = (uint8_t) iValue;
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpAtmoshereStatus(uint8_t iStatus)
{
	BSP_StatusTypeDef state = BSP_OK;
	DoorBoardInfo[6] = iStatus;
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpOpenSpeed(uint8_t speed)
{
	BSP_StatusTypeDef state = BSP_OK;
	DoorBoardInfo[7] = speed;
	DataChangedFlag = 1;
	return state;
}

BSP_StatusTypeDef  BSP_Log_UpMotorSpeed(uint16_t iValue)
{
	BSP_StatusTypeDef state = BSP_OK;
	DoorBoardInfo[8] = (uint8_t)(iValue >> 8);
	DoorBoardInfo[9] = (uint8_t)(iValue);
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpMotorStatus(uint8_t motorStatus)
{
	BSP_StatusTypeDef state = BSP_OK;
	DoorBoardInfo[10] = motorStatus;
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpOpenMode(uint8_t openMode)
{
	BSP_StatusTypeDef state = BSP_OK;
	DoorBoardInfo[11] = openMode;
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpGentleStatus(uint8_t status)
{
	BSP_StatusTypeDef state = BSP_OK;
	DoorBoardInfo[12] = status;
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpAirSensorStatus(uint8_t status)
{
	BSP_StatusTypeDef state = BSP_OK;
	DoorBoardInfo[13] = status;
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpRaderParam(uint8_t* pDataBuf)
{	
	BSP_StatusTypeDef state = BSP_OK;
	
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_UpRaderStatus(uint8_t* pDataBuf)
{
	BSP_StatusTypeDef state = BSP_OK;
	
	DataChangedFlag = 1;
	return state;
}
BSP_StatusTypeDef  BSP_Log_WriteFlag(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	DataChangedFlag = 1;
	return state;
}

BSP_StatusTypeDef  BSP_Log_CheckReportInfo(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	if (0 == DataChangedFlag)
	{
		return state;
	}
	uint8_t i;
	uint8_t temp[31];
	temp[0] = 0x5B;
	temp[1] = 0xD2;
	temp[2] = 0x01;
	temp[3] = 0x00;
	temp[4] = 0x18;
	temp[30] = 0x5D;
	for (i = 0; i < 24; i++)
	{
		temp[5 + i] = DoorBoardInfo[i];
	}
	
	temp[29] = getXORCode(temp + 1, 28);
	state = BSP_SendDataToDriverBoard(temp, 31, 0xFFFF);
	DataChangedFlag = 0;
	return state;
}

/****************************************** END OF FILE **********************************/
