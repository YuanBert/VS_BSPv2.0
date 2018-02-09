/**
******************************************************************************
* File Name          : ds_ProcotolLayer.h
* Description        : 
*
******************************************************************************
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether 
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
*
* COPYRIGHT(c) 2017 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DS_PROCOTOLLAYER_H
#define __DS_PROCOTOLLAYER_H
#ifdef __cplusplus
extern "C" {
#endif
  
  
#define REQUESTFIXEDCOMMANDLEN        7         //Header + CmdType + CmdParam + DataLength + XOR8Bits + End
#define ACKFIXEDCOMMANDLEN            6         //Header + AckCmdCode + AckCodeH + XOR8Bits  + End
 
#define DATABUFLEN      512
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "bsp_common.h"
#include "bsp_DataTransmissionLayer.h"
  

  struct t_HandingFlag{
    uint8_t     Flag;
    uint8_t     position;
    uint8_t     ptrDataBuf;
  };
  struct t_RevDataStruct{
    
    uint8_t     CmdType;
    uint8_t     CmdParam;
    uint8_t     XOR8BIT;
    uint16_t    DataLength;
    uint16_t    TotalLength;
    uint8_t     NumberOfBytesReceived;
    uint8_t     RevOKFlag;
    uint8_t     *pRevDataBuf;  
  };
  
  struct t_NeedToAckStruct{
    uint8_t     AckCmdCode[16];
    uint8_t     AckCodeH[16];
    uint8_t     AckCodeL[16];    
    uint8_t     CmdType[16];
    uint8_t     CmdParam[16];
    uint8_t     DeviceType[16];// 0-None or Error 1-CoreBoard   2-LeftDoorBoard 3-RightDoorBoard
    uint8_t     TableID[16];
  };
  
  struct t_SendDataStruct{
    uint8_t     CmdType;
    uint8_t     CmdParam;
    uint8_t     SendOKFlag;
    uint8_t     RevAckedOK;
    uint8_t     *pSendDataBuf; 
  };
  
  struct t_RevACkStruct{
    uint8_t     AckCmdCode[5];
    uint8_t     AckCodeH[5];
    uint8_t     AckCodeL[5];
    uint8_t     CheckedAckFlag[5];
    uint8_t     AckCnt;
  };
  
//  struct t_SentOrdersStruct{
//    uint8_t     CmdType[5];
//    uint8_t     CmdParam[5];
//    uint8_t     
//  
//  };
  typedef struct {
    uint8_t tab[16];
    uint8_t tabCnt;
  }tTable;

  typedef struct t_HandingFlag      HandingFlag,          *pHandingFlag;
  typedef struct t_RevACkStruct     AckedStruct,          *pAckedStruct;
  typedef struct t_RevDataStruct    RevDataStruct,        *pRevDataStruct;
  typedef struct t_SendDataStruct   SendDataStrct,        *pSendDataStruct;
  typedef struct t_NeedToAckStruct  NeedToAckStruct,      *pNeedToAckStruct;
  

BSP_StatusTypeDef BSP_HandingUartDataFromDriverBoard(void);

BSP_StatusTypeDef BSP_HandingDriverBoardRequest(void);
	
BSP_StatusTypeDef BSP_SendAckData(void);

/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
}
#endif
#endif /*__DS_PROCOTOLLAYER_H */