/**
******************************************************************************
  * File Name          : bsp_motor.c
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
#include "bsp_motor.h"
#include "tim.h"
#include "bsp_AirSensor.h"
#include "BSP_DAC5571.h"

extern uint8_t      gComingCarFlag;
extern AirSensor    gAirSensor;
extern MOTORMACHINE gMotorMachine;
extern GPIOSTATUSDETECTION gGentleSensorStatusDetection;
extern GPIOSTATUSDETECTION gMCUAIRInputStatusGpio;
/*�رյ�բ��ʱ�����*/
extern uint8_t gCloseFlag;
extern uint32_t gCloseTimCnt;
/*******************************************************************************
*
*       Function        :BSP_MotorInit()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorInit(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
  
  gMotorMachine.RunningState     = 0;
  gMotorMachine.RunDir           = DOWNDIR;
  gMotorMachine.HorFilterCnt     = 0;
  gMotorMachine.VerFilterCnt     = 0;
  gMotorMachine.EncounteredFlag  = 0;
  gMotorMachine.Motor_Error      = Motor_OK;
  gMotorMachine.RemoteControFlag = 0;
  
  HAL_Delay(100);
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorOpen()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorOpen(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);  
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorClose()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorClose(void)
{
  BSP_StatusTypeDef state  = BSP_OK;  
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorRun()
*
*       Input           :uint8_t nDir
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorRun(uint8_t nDir)
{
  BSP_StatusTypeDef state  = BSP_OK;
  if(UPDIR == nDir) //��ʱ��
  {
	BSP_DAC5571_WriteValue(NormalOperationMode, 150);
	HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);
  }
  
  if(DOWNDIR == nDir) //˳ʱ��
  {
    HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_RESET);    
  }
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorStop()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorStop(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  HAL_GPIO_WritePin(MotorBRKCtrl_GPIO_Port,MotorBRKCtrl_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorFRCtrl_GPIO_Port,MotorFRCtrl_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MotorENCtrl_GPIO_Port,MotorENCtrl_Pin,GPIO_PIN_SET);
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorCheckA()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2018/02/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorCheckA(void)
{
	BSP_StatusTypeDef state  = BSP_OK;
	
	//λ�ô���
	if (gMotorMachine.HorizontalRasterState && gMotorMachine.VerticalRasterState)
	{
		BSP_MotorStop();
		state = BSP_ERROR;
		return state;	
	}
	
	//�ڴ�ֱλ��
	if (0 == gMotorMachine.HorizontalRasterState && gMotorMachine.VerticalRasterState)
	{
		/* ����ظл��״�̽�⵽�ź�ʱ������բ */
		if (gCloseFlag || gGentleSensorStatusDetection.GpioCheckedFlag)
		{
			return state;
		}
		
		if (gMotorMachine.OpenFlag)
		{
			gMotorMachine.OpenFlag = 0;
		}
		
		if (1 == gMotorMachine.CloseFlag && 0 == gMotorMachine.RunningState)
		{
			if (DOWNDIR == gMotorMachine.RunDir)
			{
				gMotorMachine.StartFlag = 1;
				return state;
			}
			return state;
		}
		gMotorMachine.OpenFlag  = 0;
		gMotorMachine.CloseFlag = 1;
		return state;
	}
	
	//��ˮƽλ��
	if(gMotorMachine.HorizontalRasterState && 0 == gMotorMachine.VerticalRasterState)
	{
		if (gMotorMachine.OpenFlag)
		{
			if (UPDIR == gMotorMachine.RunDir)
			{
				gMotorMachine.StartFlag = 1;
				gMotorMachine.CloseFlag = 0;
				return state;
			}
		}
		if (gMotorMachine.CloseFlag)
		{
			gMotorMachine.CloseFlag = 0;
			gMotorMachine.OpenFlag	= 0;
			return state;
		}
	}
	
	//���м�λ��
	if(0 == gMotorMachine.HorizontalRasterState && 0 == gMotorMachine.VerticalRasterState)
	{
		//��բ��������
		if (gMotorMachine.RunningState && DOWNDIR == gMotorMachine.RunDir)
		{
			/* �����ظ� �״� ѹ���� �ź�ʱ ֹͣת�� */
			if (gGentleSensorStatusDetection.GpioCheckedFlag || gAirSensor.CheckedFlag)
			{
				BSP_MotorStop();
				gMotorMachine.RunningState = 0;
				gMotorMachine.OpenFlag = 1;
				gMotorMachine.EncounteredFlag = 1;
				
			}
			return state;
		}
		/* ���跴������ */
		if (0 == gMotorMachine.RunningState && gMotorMachine.OpenFlag)
		{
			gMotorMachine.RunDir = UPDIR;
			gMotorMachine.EncounteredFlag = 1;
			gMotorMachine.OpenFlag = 0;
			gMotorMachine.CloseFlag = 1;
			gMotorMachine.StartFlag = 1;
		}
		
		/* ��ʼ��⣬��������м�λ�ã����й�բ���� */
		if (0 == gMotorMachine.RunningState && DOWNDIR == gMotorMachine.RunDir)
		{
			gMotorMachine.StartFlag = 1;
		}
		
	}
	return state;
}
/*******************************************************************************
*
*       Function        :BSP_MotorActionA()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorActionA(void)
{
	BSP_StatusTypeDef state = BSP_OK;
	if (gMotorMachine.StartFlag)
	{
		if (gMotorMachine.RunningState)
		{
			gMotorMachine.StartFlag = 0;
			return state;
		}
		gMotorMachine.RunningState = 1;
		BSP_MotorRun(gMotorMachine.RunDir);
//		if (UPDIR == gMotorMachine.RunDir && 0 == gMotorMachine.EncounteredFlag)
//		{
//			HAL_TIM_Base_Start_IT(&htim6);
//		}
		gMotorMachine.StartFlag = 0;
	}
	return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorCheck()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorCheck(void)
{
  BSP_StatusTypeDef state  = BSP_OK;
  
  //ʹ��ң��ʱ���˳�
  if(gMotorMachine.RemoteControFlag)
  {
    return state;
  }
  //��δ�˳�ʱֱ�ӣ�����
  if(gComingCarFlag)
  {
    return state;
  }
  if(gMotorMachine.VerticalRasterState && gMotorMachine.HorizontalRasterState)
  {
    BSP_MotorStop();
    state = BSP_ERROR;
    return state;
  }
  //��ֱλ��
  if(gMotorMachine.VerticalRasterState && 0 == gMotorMachine.HorizontalRasterState)
  {
    if(gMotorMachine.GentleSensorFlag || gMotorMachine.RadarSensorFlag) //�����ڵظк��״��ʱ�򣬲���բ
    {
      return state;
    }
    
    if(gMotorMachine.OpenFlag)
    {
      gMotorMachine.OpenFlag = 0;
    }
    
    if(1 == gMotorMachine.CloseFlag && 0 == gMotorMachine.RunningState)
    {
      if(DOWNDIR == gMotorMachine.RunDir)
      {
        gMotorMachine.StartFlag = 1;
        return state;
      }
      return state;
    }
    
    gMotorMachine.OpenFlag  = 0;
    gMotorMachine.CloseFlag = 1;
    return state;
  }
  /* ˮƽλ�� */
  if(0 == gMotorMachine.VerticalRasterState && gMotorMachine.HorizontalRasterState)
  {
    if(gMotorMachine.OpenFlag)
    {
      if(UPDIR == gMotorMachine.RunDir)
      {
        gMotorMachine.StartFlag = 1;
        gMotorMachine.CloseFlag = 0;
        return state;
      }
    }
    if(gMotorMachine.CloseFlag)
    {
      gMotorMachine.CloseFlag = 0;
      gMotorMachine.OpenFlag  = 0;
      return state;
    }
  }
  /* ����λ�� */
  if(0 == gMotorMachine.VerticalRasterState && 0 == gMotorMachine.HorizontalRasterState)
  {
    
	  //���բ��������
	  if (gMotorMachine.RunningState && DOWNDIR == gMotorMachine.RunDir)
	  {
		  if (gMotorMachine.GentleSensorFlag || gMotorMachine.RadarSensorFlag || gMotorMachine.AirSensorFlag) //��⵽�ظк��״�󲻹�բ��
			  {
				  BSP_MotorStop();
				  gMotorMachine.RunningState = 0;
				  gMotorMachine.OpenFlag = 1;
				  gGentleSensorStatusDetection.GpioCheckedFlag = 0;
				  return state;
			  }
		  return state;
	  }
	  //��բ�ķ�������
	  if(gMotorMachine.RunningState && UPDIR == gMotorMachine.RunDir)
	  {
		  //��������
		  return state;
	  }
	  
	  //������ڷ�����״̬���Ҵ�����Ҫ����״̬
	  if(0 == gMotorMachine.RunningState && gMotorMachine.OpenFlag)
	  {
		  gMotorMachine.RunDir = UPDIR;
		  gMotorMachine.EncounteredFlag = 1;
		  gMotorMachine.OpenFlag = 0;
		  gMotorMachine.CloseFlag = 1;
		  gMotorMachine.StartFlag = 1; 
		  
	  }
	  
	//////////////////////////////////////////////////////////////////////////////  
	  if(gMotorMachine.OpenFlag)
    {
      if(gMotorMachine.RunningState)
      {
        gMotorMachine.CloseFlag = 0;
        return state;
      }
      
      gMotorMachine.RunDir = UPDIR;
      gMotorMachine.OpenFlag = 1;
      gMotorMachine.CloseFlag = 0;
      gMotorMachine.StartFlag = 1;
      return state;
    }
    
    if(gMotorMachine.CloseFlag)
    {
      if(gMotorMachine.RunningState)
      {
        if(gMotorMachine.GentleSensorFlag ||gMotorMachine.RadarSensorFlag || gMotorMachine.AirSensorFlag) //��⵽�ظк��״�󲻹�բ��
        {
          BSP_MotorStop();
          gMotorMachine.RunningState = 0;
          gMotorMachine.OpenFlag = 1;
          gGentleSensorStatusDetection.GpioCheckedFlag = 0;
          return state;
        }
        return state;
      }
      else
      {
        gMotorMachine.RunDir = DOWNDIR;
        gMotorMachine.StartFlag = 1;
        return state;
      }
    }
	  
    if(0 == gMotorMachine.RunningState)
    {
      gMotorMachine.RunDir = DOWNDIR;
      //gMotorMachine.OpenFlag = 1;
      gMotorMachine.CloseFlag = 0;
      gMotorMachine.StartFlag = 1;    
      return state;
    }
  }
  return state;
}

/*******************************************************************************
*
*       Function        :BSP_MotorAction()
*
*       Input           :void
*
*       Return          :BSP_StatusTypeDef
*
*       Description     :--
*
*
*       Data            :2017/12/28
*       Author          :bertz
*******************************************************************************/
BSP_StatusTypeDef BSP_MotorAction(void)
{
  BSP_StatusTypeDef state = BSP_OK;
  if(gMotorMachine.StartFlag)
  {
    if(gMotorMachine.RunningState)
    {
      gMotorMachine.StartFlag = 0;
      return state;
    }
    gMotorMachine.RunningState = 1;
    BSP_MotorRun(gMotorMachine.RunDir);
    gMotorMachine.StartFlag = 0;
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
