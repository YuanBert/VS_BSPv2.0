/**
  ******************************************************************************
  * File Name          : bsp_motor.h
  * Description        : this file is about motor control
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
#ifndef __bsp_motor_H
#define __bsp_motor_H
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "usart.h" 
#include "bsp_common.h"
	 
#define UPDIR                   0
#define DOWNDIR                 1
   
   enum e_MOTOR_ERROR_CODE{
     Motor_OK = 0,
     Motor_RunErr = 1,
     Motor_VerRasterErr = 2,
     Motor_HorRasterErr = 3,
     Motor_OtherErr = 4
   };   
   typedef enum e_MOTOR_ERROR_CODE MOTOR_ERROR_CODE;
   
   typedef struct s_MotorMachine{
     uint8_t  VerticalRasterState;  
     uint8_t  HorizontalRasterState;
     uint8_t  RunningState;
     uint8_t  RunDir;
     uint16_t SetRunSpeed;
     uint16_t CurrentRunSpeed;
     uint8_t  StartFlag;
     uint8_t  VerFilterCnt;
     uint8_t  HorFilterCnt;
     uint8_t  Motor_Error;
     uint8_t  GentleSensorFlag;
     uint8_t  RadarSensorFlag;
     uint8_t  AirSensorFlag;
     uint8_t  RemoteControFlag;
     uint8_t  OpenFlag;
     uint8_t  CloseFlag;
     uint8_t  EncounteredFlag;
	 uint16_t  StepCnt;
   }MOTORMACHINE,*pMOTORMACHINE;
   
BSP_StatusTypeDef      BSP_MotorInit(void);
BSP_StatusTypeDef      BSP_MotorOpen(void);
BSP_StatusTypeDef      BSP_MotorClose(void);
BSP_StatusTypeDef      BSP_MotorRun(uint8_t nDir);
BSP_StatusTypeDef      BSP_MotorStop(void);
BSP_StatusTypeDef      BSP_MotorCheck(void);
BSP_StatusTypeDef      BSP_MotorAction(void);
BSP_StatusTypeDef	   BSP_MotorCheckA(void);
BSP_StatusTypeDef	   BSP_MotorActionA(void);
#ifdef __cplusplus
}
#endif
#endif /*__bsp_motor_H */