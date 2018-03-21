/**
******************************************************************************
  * File Name          : bsp_led.c
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
#include "bsp_led.h"
#include "bsp_motor.h"
extern MOTORMACHINE gMotorMachine;
extern uint8_t gComingCarFlag;
 /*******************************************************************************
    *
    *       Function        :BSP_RUNNINGLED_ON()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_RUNNINGLED_ON(void)
    {
      HAL_GPIO_WritePin(RunningLED_GPIO_Port, RunningLED_Pin,RunningLED_ON);
    }
    /*******************************************************************************
    *
    *       Function        :BSP_RUNNINGLED_OFF()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_RUNNINGLED_OFF(void)
    {
        HAL_GPIO_WritePin(RunningLED_GPIO_Port, RunningLED_Pin,RunningLED_OFF);
    }
    
    /*******************************************************************************
    *
    *       Function        :BSP_RUNNINGLED_TORGGLE()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_RUNNINGLED_TORGGLE(void)
    {
      HAL_GPIO_TogglePin(RunningLED_GPIO_Port,RunningLED_Pin);
    }
    
    /*******************************************************************************
    *
    *       Function        :DS_COMMUNICATIONLED_ON()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_COMMUNICATIONLED_ON(void)
    {
      HAL_GPIO_WritePin(CommunicationLED_GPIO_Port, CommunicationLED_Pin,CommunicationLED_ON);
    }
    
    /*******************************************************************************
    *
    *       Function        :BSP_COMMUNICATIONLED_OFF()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_COMMUNICATIONLED_OFF(void)
    {
      HAL_GPIO_WritePin(CommunicationLED_GPIO_Port, CommunicationLED_Pin,CommunicationLED_OFF);
    }
    
    /*******************************************************************************
    *
    *       Function        :BSP_COMMUNICATIONLED_TORGGLE()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_COMMUNICATIONLED_TORGGLE(void)
    {
       HAL_GPIO_TogglePin(CommunicationLED_GPIO_Port, CommunicationLED_Pin);
    }
    
    /*******************************************************************************
    *
    *       Function        :DS_ATMOSPHERELED1_ON()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_ATMOSPHERELEDG_ON(void)
    {
      HAL_GPIO_WritePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin,MCUAtmosphereLEDG_ON);
    }
    
    /*******************************************************************************
    *
    *       Function        :BSP_ATMOSPHERELEDG_OFF()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_ATMOSPHERELEDG_OFF(void)
    {
      HAL_GPIO_WritePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin,MCUAtmosphereLEDG_OFF);
    }
    
    /*******************************************************************************
    *
    *       Function        :BSP_ATMOSPHERELEDG_TOGGLE()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_ATMOSPHERELEDG_TOGGLE(void)
    {
       HAL_GPIO_TogglePin(MCUAtmosphereLEDG_GPIO_Port,MCUAtmosphereLEDG_Pin);
    }
    
        /*******************************************************************************
    *
    *       Function        :BSP_ATMOSPHERELEDR_ON()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_ATMOSPHERELEDR_ON(void)
    {
      HAL_GPIO_WritePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin,MCUAtmosphereLEDR_ON);
    }
    
    /*******************************************************************************
    *
    *       Function        :BSP_ATMOSPHERELEDR_OFF()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_ATMOSPHERELEDR_OFF(void)
    {
      HAL_GPIO_WritePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin,MCUAtmosphereLEDR_OFF);
    }
    
    /*******************************************************************************
    *
    *       Function        :BSP_ATMOSPHERELEDR_TOGGLE()
    *
    *       Input           :void
    *
    *       Return          :void
    *
    *       Description     :--
    *
    *
    *       Data            :2017/12/25
    *       Author          :bertz
    *******************************************************************************/
    void BSP_ATMOSPHERELEDR_TOGGLE(void)
    {
       HAL_GPIO_TogglePin(MCUAtmosphereLEDR_GPIO_Port,MCUAtmosphereLEDR_Pin);
    }
    
    void BSP_LEDCheck(void)
    {
	    if (gComingCarFlag)
      {
        BSP_ATMOSPHERELEDG_TOGGLE();
        BSP_ATMOSPHERELEDR_TOGGLE();
        BSP_COMMUNICATIONLED_TORGGLE();
      }
      else
      {
        BSP_ATMOSPHERELEDG_ON();
        BSP_ATMOSPHERELEDR_OFF();
        BSP_COMMUNICATIONLED_OFF();
      }
    }
   
  /**
  * @}
  */
  /**
  * @}
  */
  /*****************************END OF FILE**************************************/
