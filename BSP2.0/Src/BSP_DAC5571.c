#include "BSP_DAC5571.h"

extern uint16_t	gCtrlSpeedCnt;

static uint8_t ctrlMS_Byte;
static uint8_t ctrlLS_Byte;
static uint8_t writebuffer[2];

BSP_StatusTypeDef BSP_DAC5571_Init(ModesOfOperationForDAC5571 mode)
{
	BSP_StatusTypeDef status = BSP_OK;
	ctrlMS_Byte = 0;
	ctrlLS_Byte = 0;
	
	ctrlMS_Byte |= (uint8_t)(mode << 4);
	writebuffer[0] = ctrlMS_Byte;
	writebuffer[1] = ctrlLS_Byte;
	status = (BSP_StatusTypeDef)HAL_I2C_Master_Transmit(&hi2c2, DAC5571_WRITEADDR,writebuffer,2,0xFFFF);
	return status;
}

BSP_StatusTypeDef BSP_DAC5571_WriteValue(ModesOfOperationForDAC5571 mode, uint8_t vData)
{
	BSP_StatusTypeDef status = BSP_OK;
	uint8_t ctrlMS = 0;
	uint8_t ctrlLS = 0;
	
	ctrlMS |= (uint8_t)(mode << 4);
	ctrlMS &= 0xF0;
	ctrlMS |= ((vData & 0xF0) >> 4);
	ctrlLS |= ((vData & 0x0F) << 4);
	writebuffer[0] = ctrlMS;
	writebuffer[1] = ctrlLS;	
	status = (BSP_StatusTypeDef)HAL_I2C_Master_Transmit(&hi2c2, DAC5571_WRITEADDR, writebuffer, 2, 0xFFFF);
	return status;
}

BSP_StatusTypeDef BSP_DAC5571_Check()
{
  BSP_StatusTypeDef status = BSP_OK;
  gCtrlSpeedCnt++;
  if (gCtrlSpeedCnt > 300)
  {
    gCtrlSpeedCnt = 300;
  }
  if (gCtrlSpeedCnt < 70)
  {
    BSP_DAC5571_WriteValue(NormalOperationMode, 45 + 3 * gCtrlSpeedCnt);
  }
  if (gCtrlSpeedCnt > 230)
  {
    BSP_DAC5571_WriteValue(NormalOperationMode, 45 + 3 * (300 - gCtrlSpeedCnt));
  }
  if ((70 <= gCtrlSpeedCnt) && (230 >= gCtrlSpeedCnt))
  {
    BSP_DAC5571_WriteValue(NormalOperationMode, 0xFF);
  }
  return status;
}

/******************* END OF FILE ************************/
