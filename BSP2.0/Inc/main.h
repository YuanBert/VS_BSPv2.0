/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CommunicationLED_Pin GPIO_PIN_13
#define CommunicationLED_GPIO_Port GPIOC
#define CTR485_EN1_Pin GPIO_PIN_1
#define CTR485_EN1_GPIO_Port GPIOC
#define CTR485_EN2_Pin GPIO_PIN_2
#define CTR485_EN2_GPIO_Port GPIOC
#define ISampleChanle_Pin GPIO_PIN_3
#define ISampleChanle_GPIO_Port GPIOC
#define Radar_TX_Pin GPIO_PIN_2
#define Radar_TX_GPIO_Port GPIOA
#define Radar_RX_Pin GPIO_PIN_3
#define Radar_RX_GPIO_Port GPIOA
#define W25Q64_SCK_Pin GPIO_PIN_5
#define W25Q64_SCK_GPIO_Port GPIOA
#define W25Q64_MISO_Pin GPIO_PIN_6
#define W25Q64_MISO_GPIO_Port GPIOA
#define W25Q64_MOSI_Pin GPIO_PIN_7
#define W25Q64_MOSI_GPIO_Port GPIOA
#define GentleSensor_Pin GPIO_PIN_4
#define GentleSensor_GPIO_Port GPIOC
#define RadarInput_Pin GPIO_PIN_0
#define RadarInput_GPIO_Port GPIOB
#define DAC_SCK_Pin GPIO_PIN_10
#define DAC_SCK_GPIO_Port GPIOB
#define DAC_SDA_Pin GPIO_PIN_11
#define DAC_SDA_GPIO_Port GPIOB
#define MCUAtmosphereLEDR_Pin GPIO_PIN_12
#define MCUAtmosphereLEDR_GPIO_Port GPIOB
#define MCUAtmosphereLEDG_Pin GPIO_PIN_13
#define MCUAtmosphereLEDG_GPIO_Port GPIOB
#define MCU_AIR_Pin GPIO_PIN_14
#define MCU_AIR_GPIO_Port GPIOB
#define MotorFRCtrl_Pin GPIO_PIN_15
#define MotorFRCtrl_GPIO_Port GPIOB
#define MotorENCtrl_Pin GPIO_PIN_6
#define MotorENCtrl_GPIO_Port GPIOC
#define MotorBRKCtrl_Pin GPIO_PIN_7
#define MotorBRKCtrl_GPIO_Port GPIOC
#define MotorSpeedInput_Pin GPIO_PIN_8
#define MotorSpeedInput_GPIO_Port GPIOC
#define MotorAlmInput_Pin GPIO_PIN_9
#define MotorAlmInput_GPIO_Port GPIOC
#define CoreBoard_TX_Pin GPIO_PIN_9
#define CoreBoard_TX_GPIO_Port GPIOA
#define CoreBoard_RX_Pin GPIO_PIN_10
#define CoreBoard_RX_GPIO_Port GPIOA
#define OpenBoxInput_Pin GPIO_PIN_2
#define OpenBoxInput_GPIO_Port GPIOD
#define HorRasterInput_Pin GPIO_PIN_3
#define HorRasterInput_GPIO_Port GPIOB
#define VerRasterInput_Pin GPIO_PIN_4
#define VerRasterInput_GPIO_Port GPIOB
#define MCU_WIRLESSB_Pin GPIO_PIN_5
#define MCU_WIRLESSB_GPIO_Port GPIOB
#define MCU_WIRLESSA_Pin GPIO_PIN_6
#define MCU_WIRLESSA_GPIO_Port GPIOB
#define MCU_WIRLESS_Pin GPIO_PIN_7
#define MCU_WIRLESS_GPIO_Port GPIOB
#define RunningLED_Pin GPIO_PIN_9
#define RunningLED_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
