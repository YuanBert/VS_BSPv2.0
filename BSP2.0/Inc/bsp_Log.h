#ifndef __BSP_LOG_H
#define __BSP_LOG_H

#ifdef __cpluscplus
extern "C"
{
#endif
#include "bsp_common.h"
#include "main.h"

	BSP_StatusTypeDef  BSP_Log_Init(uint32_t id);
	BSP_StatusTypeDef  BSP_Log_UpICurrentValue(uint16_t iValue);
	BSP_StatusTypeDef  BSP_Log_UpAtmoshereStatus(uint8_t iStatus);
	BSP_StatusTypeDef  BSP_Log_UpOpenSpeed(uint8_t speed);
	BSP_StatusTypeDef  BSP_Log_UpMotorSpeed(uint16_t iValue);
	BSP_StatusTypeDef  BSP_Log_UpMotorStatus(uint8_t motorStatus);
	BSP_StatusTypeDef  BSP_Log_UpOpenMode(uint8_t openMode);
	BSP_StatusTypeDef  BSP_Log_UpGentleStatus(uint8_t status);
	BSP_StatusTypeDef  BSP_Log_UpAirSensorStatus(uint8_t status);
	BSP_StatusTypeDef  BSP_Log_UpRaderParam(uint8_t* pDataBuf);
	BSP_StatusTypeDef  BSP_Log_UpRaderStatus(uint8_t* pDataBuf);
	BSP_StatusTypeDef  BSP_Log_CheckReportInfo(void);
	BSP_StatusTypeDef  BSP_Log_WriteFlag(void);

#ifdef __cpluscplus
}
#endif

#endif /* bsp_log.h*/