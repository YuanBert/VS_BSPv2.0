/**
******************************************************************************
* File Name          : ds_ProtocolLayer.c
* Description        : 
******************************************************************************
* This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether 
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
*
* Copyright (c) 2017 STMicroelectronics International N.V. 
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted, provided that the following conditions are met:
*
* 1. Redistribution of source code must retain the above copyright notice, 
*    this list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
* 3. Neither the name of STMicroelectronics nor the names of other 
*    contributors to this software may be used to endorse or promote products 
*    derived from this software without specific written permission.
* 4. This software, including modifications and/or derivative works of this 
*    software, must execute solely and exclusively on microcontroller or
*    microprocessor devices manufactured by or for STMicroelectronics.
* 5. Redistribution and use of this software other than as permitted under 
*    this license is void and will automatically terminate your rights under 
*    this license. 
*
* THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
* PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
* RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
* SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
* OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
* NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
* EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "bsp_ProtocolLayer.h"
#include "bsp_DataTransmissionLayer.h"
#include "bsp_motor.h"

extern uint8_t         gComingCarFlag;
extern MOTORMACHINE gMotorMachine;

extern USARTRECIVETYPE     DriverBoardUsartType;
extern USARTRECIVETYPE     LeftDoorBoardUsartType;

HandingFlag     SendOpenFlag;


AckedStruct    DriverBoardAckedData;
AckedStruct    LeftDoorBoardAckedData;

RevDataStruct   DriverBoardRevDataStruct;
RevDataStruct   LeftDoorBoardRevDataStruct;

SendDataStrct   DriverBoardSendDataStruct;
SendDataStrct   LeftDoorBoardSendDataStruct;


NeedToAckStruct sNeedToAckStruct;


static uint8_t DriverRevDataBuf[DATABUFLEN];
static uint8_t DriverSenddataBuf[DATABUFLEN];    

static uint8_t LeftDoorRevDataBuf[DATABUFLEN];
static uint8_t LeftDoorSendDataBuf[DATABUFLEN];


tTable Table = {{0},0};

static uint8_t GetAvailableTableID(void)
{
  uint8_t reCode = 0xFF;
  uint8_t i = 0;
  
  for(i = 0; i < 16; i++)
  {
    if(0 == Table.tab[i])
    {
      return i;
    }
  }
  return reCode;
}

static uint8_t WriteStatusToTable(uint8_t tableID,uint8_t statusCode)//statusCode : 0-表示空，1-表示正在处理，2-表示处理完成可以回复
{
    uint8_t reCode = 0xFF;
    if(tableID > 16)
    {
      return reCode;
    }
    if(0 != statusCode && 0 != Table.tab[tableID])
    {
      return reCode;
    }
    Table.tab[tableID] = statusCode;
    reCode = tableID;
    return reCode;
}




static uint8_t getXORCode(uint8_t* pData,uint16_t len)
{
  uint8_t ret;
  uint16_t i;
  ret = pData[0];
  for(i = 1; i < len; i++)
  {
    ret ^= pData[i];
  }
  return ret;
}

static BSP_StatusTypeDef BSP_HandingUartData(pRevDataStruct pRevData,pAckedStruct pAckedData,pUSARTRECIVETYPE pUsartType ,uint8_t* pRevDataBuf)
{
  BSP_StatusTypeDef state = BSP_OK;
  uint8_t xorTemp;
  uint16_t i;
	
  /* 判断串口数据是否接收完成 */
  if(!(pUsartType->RX_Flag))
  {
    return state;
  }
  
  pUsartType->RX_Flag = 0;
	
  /*  Handling the ACK Cmd */
  if(0xA0 == (*(pUsartType->RX_pData + 1) & 0xF0))
  {
	  if(pAckedData->AckCnt > 5)
      {
        return state;
      }
      if(0x5B != *(pUsartType->RX_pData) || 0x5D != *(pUsartType->RX_pData + ACKFIXEDCOMMANDLEN -1))
      {
          return state;
      }
      xorTemp = getXORCode(pUsartType->RX_pData + 1,3);
      if(xorTemp != *(pUsartType->RX_pData + 4))
      {
        return state;
      }
      
      pAckedData->AckCmdCode[pAckedData->AckCnt]     = *(pUsartType->RX_pData + 1);
      pAckedData->AckCodeH[pAckedData->AckCnt]       = *(pUsartType->RX_pData + 2);
      pAckedData->AckCodeL[pAckedData->AckCnt]       = *(pUsartType->RX_pData + 3);
	  pAckedData->CheckedAckFlag[pAckedData->AckCnt] = 1;
      
      pAckedData->AckCnt++;
      
      return state;
  }
  /* 在ACK命令处理时需要对ACkCnt进行间操作，每进行一次操作进行一次减操作 */
  
  
  /* Handling Request Cmd Data */
  if(pRevData->RevOKFlag)
  {
    return state;
  }
  
	
  if((pRevData->NumberOfBytesReceived) < (pRevData->DataLength) && 0 != (pRevData->NumberOfBytesReceived))
  {
    for(i = 0; i < pUsartType->RX_Size; i++)
    {
      pRevDataBuf[5 + pRevData->NumberOfBytesReceived] = *(pUsartType->RX_pData + i);
      pRevData->NumberOfBytesReceived ++;
      if(pRevData->DataLength == pRevData->NumberOfBytesReceived)
      {
        pRevData->XOR8BIT = *(pUsartType->RX_pData + i + 1);
        if(0x5D != *(pUsartType->RX_pData + i + 2))
        {
          pRevData->NumberOfBytesReceived = 0;
          pRevData->DataLength = 0;
          pRevData->TotalLength = 0;
          return state;
        }
        pRevData->TotalLength = pRevData->DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to check XOR code */
        xorTemp = getXORCode(pRevDataBuf + 1, pRevData->TotalLength - 3);
        if(pRevData->XOR8BIT != xorTemp)
        {
          pRevData->NumberOfBytesReceived = 0;
          pRevData->DataLength = 0;
          pRevData->TotalLength = 0;      
          return state;
        }
        pRevDataBuf[5 + pRevData->NumberOfBytesReceived] = xorTemp;
        pRevDataBuf[5 + pRevData->NumberOfBytesReceived + 1] = 0x5D;
        pRevData->pRevDataBuf = pRevDataBuf;
        pRevData->RevOKFlag = 1;
      }
    }
    return state;
  }
	
  
  if(0 == pRevData->TotalLength)
  { 
    if(0x5B != *(pUsartType->RX_pData))
    {
	    return state;
    }
	  	  
    pRevData->CmdType      =*(pUsartType->RX_pData + 1);
    pRevData->CmdParam     =*(pUsartType->RX_pData + 2);
    
    pRevData->DataLength   =(*(pUsartType->RX_pData + 3)<< 8) + *(pUsartType->RX_pData + 4);
    
    if(0 == pRevData->DataLength)
    {
      if(0x5D != *(pUsartType->RX_pData + REQUESTFIXEDCOMMANDLEN - 1))
      {
        return state;
      }
	    
      pRevData->XOR8BIT         =*(pUsartType->RX_pData + 5);
      pRevData->TotalLength     = REQUESTFIXEDCOMMANDLEN;
	  pRevDataBuf[0]           = 0x5B;
	  pRevDataBuf[1]           = pRevData->CmdType;
	  pRevDataBuf[2]           = pRevData->CmdParam;
	  pRevDataBuf[3]           = *(pUsartType->RX_pData + 3);
	  pRevDataBuf[4]           = *(pUsartType->RX_pData + 4);
	  xorTemp = getXORCode(pRevDataBuf + 1, REQUESTFIXEDCOMMANDLEN - 3);
	  
	  if((pRevData->XOR8BIT) != xorTemp)
	  {
	    pRevData->TotalLength = 0;
	    return state;
	  }

      pRevDataBuf[5]           = pRevData->XOR8BIT;
      pRevDataBuf[6]           = 0x5D;
      
      pRevData->RevOKFlag = 1;
      
      return state;
    }
    
    for(i = 5; i < pUsartType->RX_Size; i++)
    {
      pRevDataBuf[i] = *(pUsartType->RX_pData + i);
      pRevData->NumberOfBytesReceived ++;
      if(pRevData->DataLength == pRevData->NumberOfBytesReceived)
      {
        if(0x5D != *(pUsartType->RX_pData + REQUESTFIXEDCOMMANDLEN + pRevData->NumberOfBytesReceived - 1))
        {
          pRevData->DataLength = 0;
          pRevData->NumberOfBytesReceived = 0;
          pRevData->TotalLength = 0;
        }        
        pRevData->XOR8BIT = *(pUsartType->RX_pData + i + 1 );
        pRevData->TotalLength = pRevData->DataLength + REQUESTFIXEDCOMMANDLEN;
        /* here to XOR check */
        xorTemp = getXORCode(pUsartType->RX_pData + 1, pRevData->TotalLength - 3);
        /* 校验，如果校验不正确，退出 */
        if(pRevData->XOR8BIT != xorTemp)
        {
          pRevData->TotalLength = 0;
          return state;
        }
        
        pRevDataBuf[0]           = 0x5B;
        pRevDataBuf[1]           = pRevData->CmdType;
        pRevDataBuf[2]           = pRevData->CmdParam;
        pRevDataBuf[3]           = *(pUsartType->RX_pData + 3);
        pRevDataBuf[4]           = *(pUsartType->RX_pData + 4);
        pRevDataBuf[i + 1]           = pRevData->XOR8BIT;
        pRevDataBuf[i + 2]           = 0x5D;
        pRevData->RevOKFlag = 1;
        return state;
      }
    }
    
  }
  return state;  
}

/*******************************************************************************
*
*       Function        :DS_HandingUartDataFromCoreBoard()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/1/31
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_HandingUartDataFromDriverBoard(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	state = BSP_HandingUartData(&DriverBoardRevDataStruct, &DriverBoardAckedData, &DriverBoardUsartType, DriverRevDataBuf);
	return state;
}


/*******************************************************************************
*
*       Function        :DS_HandingCoreBoardRequest()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/2/2
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_HandingDriverBoardRequest(void)
{
  BSP_StatusTypeDef state = BSP_OK;
  uint8_t tempTableID;
  //uint8_t tempBuf[512];
	
  if(DriverBoardRevDataStruct.RevOKFlag)
  {
    tempTableID =  GetAvailableTableID();
	  
    if(0xFF == tempTableID)
    {
      return state;
    }
    
	  switch ((DriverBoardRevDataStruct.CmdType) & 0xF0)
    {
	    
    case 0xB0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAB;
	    if (0xB2 == DriverBoardRevDataStruct.CmdType && 0x01 == DriverBoardRevDataStruct.CmdParam)
	    {
		    sNeedToAckStruct.AckCodeH[tempTableID] = 0x02;
		    sNeedToAckStruct.DeviceType[tempTableID] = 0x01;
		    SendOpenFlag.Flag = 1;
		    SendOpenFlag.position = tempTableID;
		    Table.tab[tempTableID] = 0x01;    //处理中
		    Table.tabCnt++; break;		    
	    }

    case 0xC0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAC;//处理系统设置命令
	    if (0xC1 == DriverBoardRevDataStruct.CmdType)//设置本机
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x01;
                Table.tab[tempTableID] = 0x01;
                Table.tabCnt++;
                /* 此处需要处理相关的命令 */
              }
	    if (0xC2 == DriverBoardRevDataStruct.CmdType)//设置左道闸机
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x02;
                Table.tab[tempTableID] = 0x01;
                Table.tabCnt++;
                /* 此处需要处理相关的命令*/
              }
	    if (0xC3 == DriverBoardRevDataStruct.CmdType)//设置右道闸机
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x03;
                Table.tab[tempTableID] = 0x01;
                Table.tabCnt++;
                /* 此处需要处理相关的命令 */
              }
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;break;
              
    case 0xD0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAD;
	    if (0xD0 == DriverBoardRevDataStruct.CmdType)
              {
                sNeedToAckStruct.AckCodeH[tempTableID] = 0x00;
	              if (0x01 == DriverBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x01;
                  Table.tab[tempTableID] = 0x02;//完成处理，可以回复
                  Table.tabCnt++;
                }
	              if (0x02 == DriverBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x02;
                  Table.tab[tempTableID] = 0x02;   
                  Table.tabCnt++;
                }
	              if (0x03 == DriverBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x03;
                  Table.tab[tempTableID] = 0x02;
                  Table.tabCnt++;
                }
	              if (0x04 == DriverBoardRevDataStruct.CmdParam)
                {
                    //
                  sNeedToAckStruct.AckCodeL[tempTableID] = 0x04;
                  Table.tab[tempTableID] = 0x02;
                  Table.tabCnt++;
                }
              }
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;break;
              
    case 0xE0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAE;
              sNeedToAckStruct.AckCodeH[tempTableID] = 0xFF;
              sNeedToAckStruct.AckCodeL[tempTableID] = 0x00;
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;
              Table.tab[tempTableID] = 0x02;
              Table.tabCnt++;break;
              
    case 0xF0:sNeedToAckStruct.AckCmdCode[tempTableID] = 0xAF;
              sNeedToAckStruct.AckCodeH[tempTableID] = 0xFF;
              sNeedToAckStruct.AckCodeL[tempTableID] = 0x00;
              sNeedToAckStruct.DeviceType[tempTableID] = 0x01;
              Table.tab[tempTableID] = 0x02;
              Table.tabCnt++;break;
    
    default: state = BSP_NOCMD;break;
	  
    }
	
	  if (1 == SendOpenFlag.Flag)
	  {
		  if (0 == gMotorMachine.VerticalRasterState)
		  {
			  gMotorMachine.StartFlag = 1;
			  gMotorMachine.OpenFlag = 1;
			  gMotorMachine.CloseFlag = 0;
			  gMotorMachine.RunDir = UPDIR;
			  gComingCarFlag = 1;
			  sNeedToAckStruct.AckCodeL[Table.tab[SendOpenFlag.position]] = 0x00;
		  }
		  else
		  {
			  sNeedToAckStruct.AckCodeL[SendOpenFlag.position] = 0x01;
		  }
		  
		  Table.tab[SendOpenFlag.position] = 0x02;
	  }
  
	  DriverBoardRevDataStruct.NumberOfBytesReceived = 0;  
	  DriverBoardRevDataStruct.DataLength  = 0;
	  DriverBoardRevDataStruct.TotalLength = 0;
	  DriverBoardRevDataStruct.RevOKFlag   = 0;
	  
  }
  return state; 
}

/*******************************************************************************
*
*       Function        :DS_SendAckData()
*
*       Input           :void
*
*       Return          :DS_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/2/5
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_SendAckData(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	uint8_t tempAck[6];
	uint8_t i = 0;
	if (Table.tabCnt < 1)
	{
		return state;
	}
	tempAck[0] = 0x5B;
	tempAck[5] = 0x5D;
	for (i = 0; i < 16; i++)
	{
		if (0x02 == Table.tab[i])
		{
			tempAck[1] = sNeedToAckStruct.AckCmdCode[i];
			tempAck[2] = sNeedToAckStruct.AckCodeH[i];
			tempAck[3] = sNeedToAckStruct.AckCodeL[i];
			
			tempAck[4] = getXORCode(tempAck + 1,3);
			
			if (0x01 == sNeedToAckStruct.DeviceType[i])
			{
				/* Send to Core Board */
				sNeedToAckStruct.DeviceType[i] = 0;
				BSP_SendDataToDriverBoard(tempAck, 6, 0xFFFF);
			}
			if (0x02 == sNeedToAckStruct.DeviceType[i])
			{
				/* Send to Left DoorBoard */
				sNeedToAckStruct.DeviceType[i] = 0;
			}
			
			Table.tab[i] = 0;
			Table.tabCnt--;
		}
	}
	return state;
}



/**
* @}
*/
/**
* @}
*/
/*****************************END OF FILE**************************************/
