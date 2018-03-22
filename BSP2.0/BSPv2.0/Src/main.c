/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_gentlesensor.h"
#include "bsp_common.h"
#include "bsp_motor.h"
#include "bsp_led.h"
#include "BSP_DAC5571.h"
#include "bsp_DataTransmissionLayer.h"
#include "bsp_ProtocolLayer.h"
#include "bsp_AirSensor.h"
#include "bsp_Log.h"
#include "bsp_RemoteControl.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/*软件版本号*/
#define  CODEVERSION		0x0201

WIRLESS gWIRLESS;
MOTORMACHINE gMotorMachine;
GPIOSTATUSDETECTION gGentleSensorStatusDetection;
AirSensor gAirSensor;

//开闸控速矩阵
uint8_t Speed15SBuf[300] = {
 47 , 52 , 56 , 59 , 62 , 64 , 67 , 69 , 71 , 73 ,
 75 , 77 , 79 , 80 , 82 , 83 , 85 , 86 , 88 , 89 ,
 91 , 92 , 93 , 94 , 96 , 97 , 98 , 99 ,100 ,102 ,
103 ,104 ,105 ,106 ,107 ,108 ,109 ,110 ,111 ,112 ,
113 ,114 ,115 ,116 ,117 ,118 ,118 ,119 ,120 ,121 ,
122 ,123 ,124 ,124 ,125 ,126 ,127 ,128 ,129 ,129 ,
130 ,131 ,132 ,132 ,133 ,134 ,135 ,135 ,136 ,137 ,
138 ,138 ,139 ,140 ,141 ,141 ,142 ,143 ,143 ,144 ,
145 ,145 ,146 ,147 ,147 ,148 ,149 ,149 ,150 ,151 ,
151 ,152 ,153 ,153 ,154 ,154 ,155 ,156 ,156 ,157 ,
158 ,158 ,159 ,159 ,160 ,161 ,161 ,162 ,162 ,163 ,
164 ,164 ,165 ,165 ,166 ,166 ,167 ,168 ,168 ,169 ,
169 ,170 ,170 ,171 ,171 ,172 ,173 ,173 ,174 ,174 ,
175 ,175 ,176 ,176 ,177 ,177 ,178 ,178 ,179 ,179 ,
180 ,180 ,181 ,181 ,182 ,182 ,183 ,183 ,184 ,184 ,
184 ,183 ,183 ,182 ,182 ,181 ,181 ,180 ,180 ,179 ,
179 ,178 ,178 ,177 ,177 ,176 ,176 ,175 ,175 ,174 ,
174 ,173 ,173 ,172 ,171 ,171 ,170 ,170 ,169 ,169 ,
168 ,168 ,167 ,166 ,166 ,165 ,165 ,164 ,164 ,163 ,
162 ,162 ,161 ,161 ,160 ,159 ,159 ,158 ,158 ,157 ,
156 ,156 ,155 ,154 ,154 ,153 ,153 ,152 ,151 ,151 ,
150 ,149 ,149 ,148 ,147 ,147 ,146 ,145 ,145 ,144 ,
143 ,143 ,142 ,141 ,141 ,140 ,139 ,138 ,138 ,137 ,
136 ,135 ,135 ,134 ,133 ,132 ,132 ,131 ,130 ,129 ,
129 ,128 ,127 ,126 ,125 ,124 ,124 ,123 ,122 ,121 ,
120 ,119 ,118 ,118 ,117 ,116 ,115 ,114 ,113 ,112 ,
111 ,110 ,109 ,108 ,107 ,106 ,105 ,104 ,103 ,102 ,
100 , 99 , 98 , 97 , 96 , 94 , 93 , 92 , 91 , 89 ,
 88 , 86 , 85 , 83 , 82 , 80 , 79 , 77 , 75 , 73 ,
 71 , 69 , 67 , 64 , 62 , 59 , 56 , 52 , 47 , 35 
};


uint8_t	  gWirlessFlag;

uint8_t   gWirlessOpenCurrentReadVal;
uint8_t   gWirlessOpenLastReadVal;
uint16_t  gWirlessOpenCnt;

uint8_t gwirlessCloseCurrentReadVal;
uint8_t gWirlessCloseLastReadVal;

uint8_t	gWirlessStopCurrentReadVal;
uint8_t gWirlessStopLastReadVal;


uint8_t	  gTimeoutFlag;	//开闸超时标记位，如果超时发送车未进场
uint8_t   gCarEnteredFlag; //车辆已经进场标记位

uint16_t  OpenSpeedCnt;
uint8_t   OpenSpeedFlag;

uint32_t gXDataBuffer[10];
uint32_t gADCBuffer[10];
uint32_t gICurrentValue;
uint16_t gICurrent;
uint8_t	 gICurrentValueBuff[2];

/*存放用于读取的电流数据*/
uint16_t gXDataBuf[10];
uint8_t  xCnt;
uint8_t  xSensorCnt;
uint16_t Xavg;
uint16_t AboveXAvg = 0x0FF;//2A电流时认为触发电子防砸

uint8_t gComingCarFlag;

uint8_t gHorCurrentRedVal;
uint8_t gHorLastCurrentRedVal;

uint8_t gVerCurrentRedVal;
uint8_t gVerLastRedVal;

uint8_t gStepCnterCurrentRedVal;
uint8_t gStepCnterLastRedVal;

uint8_t gLastStepValue;
uint8_t gCurrentStepValue;

uint8_t gTIM6Cnt;
uint8_t gTIM6Flag;
uint8_t gTIM6TwoFlag;
uint16_t gSpeed;

/*氛围灯标记以及氛围灯控制计时*/
uint8_t gAtmosphereTimFlag;
uint16_t gAtmosphereTimCnt;

/*关闭道闸定时器标记*/
uint8_t gCloseFlag;
uint32_t gCloseTimCnt;

/*定时日志上报*/
uint8_t  gReportLogFlag;
uint32_t gReportLogTimCnt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void FilterADCSignals(uint16_t xData);
void CheckCarEnteredFlag(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t i = 0;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim4);//50us
	HAL_TIM_Base_Start_IT(&htim5);//1ms
	HAL_TIM_Base_Start_IT(&htim6);//5ms
	/*获取电流值和温度信息*/
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&gADCBuffer,10);
	
	
	BSP_MotorInit();
	BSP_RUNNINGLED_ON(); 
	BSP_AirSensor_Init(5);
	BSP_GentleSensorInit();
	BSP_Log_Init(0x01020304);
	BSP_DriverBoardProtocolInit();
	BSP_LeftDoorBoardProtocolInit();
	BSP_DAC5571_Init(NormalOperationMode);
	
	BSP_DAC5571_WriteValue(NormalOperationMode, 200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  BSP_HandingUartDataFromDriverBoard();
	  BSP_HandingDriverBoardRequest();
	  BSP_SendAckData();
	  
	  BSP_MotorCheckA();
	  BSP_MotorActionA();
	  
	  /*氛围灯控制*/
	  if (gAtmosphereTimFlag)
	  {
		  BSP_LEDCheck(); 
		  BSP_Log_UpAtmoshereStatus(gComingCarFlag);
		  gAtmosphereTimFlag = 0;
	  }
	  
	  
	  CheckCarEnteredFlag();
	  
	  /*定时上报日志*/
	  if (gReportLogFlag)
	  {
		  BSP_Log_UpMotorStatus(gMotorMachine.RunningState);
		  BSP_Log_UpGentleStatus(gGentleSensorStatusDetection.GpioCheckedFlag);
		  BSP_Log_UpAirSensorStatus(gAirSensor.CheckedFlag);
		  BSP_Log_CheckReportInfo();
		  gReportLogFlag = 0;
	  }
	  
          /* 在开闸的过程中每50ms更新一次数据值，进行调速 */
          if(gTIM6TwoFlag)
          {
            if(OpenSpeedFlag)
            {
              BSP_DAC5571_WriteValue(NormalOperationMode,Speed15SBuf[gSpeed]);
              gSpeed++;
              if(gSpeed > 299)
              {
                gSpeed = 299;
              }
            }
            else
            {
              gSpeed = 0;
            }
            gTIM6TwoFlag = 0;
          }
          
	  /* 在开闸的过程中 每五个毫秒获取一次电流值 */
	  if (gTIM6Flag)
	  {
		  gTIM6Flag = 0;
		  for (i = 0; i < 10;)
		  {
			  gICurrentValue += gADCBuffer[i++];
			  i++;
		  }
		  
		  gICurrentValue /= 5;
		  gICurrentValueBuff[0] = (uint8_t)(gICurrentValue >> 8);
		  gICurrentValueBuff[1] = (uint8_t)(gICurrentValue);
		  if(DOWNDIR == gMotorMachine.RunDir && gMotorMachine.RunningState)
		  {
			  FilterADCSignals(gICurrentValue);
		  }
		  
		  if (UPDIR == gMotorMachine.RunDir && gMotorMachine.RunningState)
		  {
			  if (gICurrentValue > gICurrent)
			  {
				  gICurrent = gICurrentValue;
				  BSP_Log_UpICurrentValue(gICurrent);
			  }
		  }
		  
		  
	  }
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* I2C2_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(htim);
	/* NOTE : This function Should not be modified, when the callback is needed,
	          the __HAL_TIM_PeriodElapsedCallback could be implemented in the user file
	*/
	/* 50us */
	if (htim4.Instance == htim->Instance)
	{
		
		gWirlessOpenCurrentReadVal = HAL_GPIO_ReadPin(MCU_WIRLESSOPEN_GPIO_Port, MCU_WIRLESSOPEN_Pin);
		if (0 == gWirlessOpenCurrentReadVal && 0 == gWirlessOpenLastReadVal)
		{
			if (gWirlessFlag)
			{
				if (0 == gWIRLESS.OpenFlag)
				{
					gWirlessOpenCnt++;
					if (gWirlessOpenCnt > 20)
					{
						gWIRLESS.OpenFlag = 1;
						gWIRLESS.STopFlag  = 1;
						gWirlessOpenCnt = 0;
						gWirlessFlag = 0;
					}
				}
				else
				{
					gWirlessOpenCnt++;
					if (gWirlessOpenCnt > 20)
					{
						gWIRLESS.OpenFlag = 0;
						gWIRLESS.STopFlag  = 0;
						gWirlessOpenCnt = 0;
						gWirlessFlag = 0;
					}
				}
			}
		}
		else
		{
			gWirlessFlag = 1;
		}
		gWirlessOpenLastReadVal = gWirlessOpenCurrentReadVal;
		
		/**/
		gHorCurrentRedVal = HAL_GPIO_ReadPin(HorRasterInput_GPIO_Port, HorRasterInput_Pin);
		gVerCurrentRedVal = HAL_GPIO_ReadPin(VerRasterInput_GPIO_Port, VerRasterInput_Pin);
		
		if (0 == gHorCurrentRedVal && 0 == gHorLastCurrentRedVal)
		{
			gMotorMachine.HorFilterCnt++;
			if (gMotorMachine.HorFilterCnt > 10)
			{
				gMotorMachine.HorizontalRasterState = 1;
				gMotorMachine.HorFilterCnt = 0;
				if (DOWNDIR == gMotorMachine.RunDir)	//检测到杆在水平位置时值运动
				{
					gMotorMachine.RunDir = UPDIR;
					gMotorMachine.RunningState = 0;
					gMotorMachine.StepCnt = 0;
					BSP_MotorStop();
					gICurrent = 0;
				}
			}
		}
		else
		{
			gMotorMachine.HorizontalRasterState = 0;
			gMotorMachine.HorFilterCnt = 0;
		}
		gHorLastCurrentRedVal = gHorCurrentRedVal;
		
		//垂直位杆监测
		if (0 == gVerCurrentRedVal && 0 == gVerLastRedVal)
		{
			gMotorMachine.VerFilterCnt++;
			if (gMotorMachine.VerFilterCnt > 10)
			{
				gMotorMachine.VerticalRasterState = 1;
				gMotorMachine.VerFilterCnt = 0;
				if (UPDIR == gMotorMachine.RunDir)
				{
					gMotorMachine.RunDir = DOWNDIR;
					gMotorMachine.RunningState = 0;
					gMotorMachine.StepCnt = 0;
					BSP_MotorStop();
					gICurrent = 0;
					//HAL_TIM_Base_Stop_IT(&htim6); //垂直到位关闭定时器中断
					gMotorMachine.EncounteredFlag = 0;//将遇阻反弹标记位清零
					gMotorMachine.DigitalAntiSmashingFlag = 0;//数字防砸标记位清空
					gCloseFlag = 1;//垂直到位，打开定时器，进行延时，延时后进行关闸
                                        OpenSpeedFlag = 0;
				}
			}
		}
		else
		{
			gMotorMachine.VerticalRasterState = 0;
			gMotorMachine.VerFilterCnt = 0;
		}
		gVerLastRedVal = gVerCurrentRedVal;
		
	}
	
	/* 1ms */
	if (htim5.Instance == htim->Instance)
	{
		/*地感检测*/
		gGentleSensorStatusDetection.GpioCurrentReadVal = HAL_GPIO_ReadPin(GentleSensor_GPIO_Port, GentleSensor_Pin);
		if (0 == gGentleSensorStatusDetection.GpioCurrentReadVal && 0 == gGentleSensorStatusDetection.GpioLastReadVal)
		{
			if (0 == gGentleSensorStatusDetection.GpioCheckedFlag)
			{
				gGentleSensorStatusDetection.GpioFilterCnt++;
				if (gGentleSensorStatusDetection.GpioFilterCnt > gGentleSensorStatusDetection.GpioFilterCntSum)
				{
					gGentleSensorStatusDetection.GpioStatusVal = 1;
					gGentleSensorStatusDetection.GpioFilterCnt = 0;
					gGentleSensorStatusDetection.GpioCheckedFlag = 1;
					gMotorMachine.GentleSensorFlag	 = 1;
					gComingCarFlag = 1;
				}
			}

		}
		else
		{
			if (gGentleSensorStatusDetection.GpioCheckedFlag)
			{
				gCloseFlag = 0; //车经过之后立即关闸
				gCarEnteredFlag = 1;
			}
			gMotorMachine.GentleSensorFlag = 0;
			gGentleSensorStatusDetection.GpioCheckedFlag  = 0;
			gGentleSensorStatusDetection.GpioStatusVal	  = 0;
			gGentleSensorStatusDetection.GpioFilterCnt    = 0;
			gGentleSensorStatusDetection.GpioSendDataFlag = 1;
			gComingCarFlag = 0;
		}
		gGentleSensorStatusDetection.GpioLastReadVal = gGentleSensorStatusDetection.GpioCurrentReadVal;
		
		if (gCloseFlag)
		{
			gCloseTimCnt++;
			if (gCloseTimCnt > 25000)//25s延时
			{
				gCloseFlag = 0;
				gCloseTimCnt = 0;
				gTimeoutFlag = 1;
			}
		}
		else
		{
			gCloseTimCnt = 0;
		}
		
		
		/* 雷达侦测 */
		
		/* 压力波探测 */
		gAirSensor.CurrentReadValue = HAL_GPIO_ReadPin(MCU_AIR_GPIO_Port, MCU_AIR_Pin);
		if (0 == gAirSensor.CurrentReadValue && 0 == gAirSensor.LastReadValue)
		{
			if (0 == gAirSensor.CheckedFlag)
			{
				gAirSensor.FilterCnt++;
				if (gAirSensor.FilterCnt > gAirSensor.FilterCntSum)
				{
					gAirSensor.CheckedFlag = 1;
					gAirSensor.FilterCnt = 0;
				}
			}
		}
		else
		{
			gAirSensor.CheckedFlag = 0;
			gAirSensor.FilterCnt = 0;
		}
		gAirSensor.LastReadValue = gAirSensor.CurrentReadValue;
		
		/* 数字防砸 */
		
		/* 日志计时定时器 */
		gReportLogTimCnt++;
		if (gReportLogTimCnt > 10000)
		{
			gReportLogFlag = 1;
			gReportLogTimCnt = 0;
		}
		/* 氛围灯 */
		gAtmosphereTimCnt++;
		if (gAtmosphereTimCnt > 200)
		{
			gAtmosphereTimFlag = 1;
			gAtmosphereTimCnt = 0;
		}
		
	}
	/* 5ms */
	if (htim6.Instance == htim->Instance)
	{
		gTIM6Flag = 1;
                gTIM6Cnt++;
                if(gTIM6Cnt > 1)
                {
                  gTIM6TwoFlag = 1;
                  gTIM6Cnt = 0;
                }
		
	}
}

void FilterADCSignals(uint16_t xData)
{
	/*如果电机处在运行状态且向下运行，则进行下一步的处理*/
	uint8_t  i;
	uint32_t temple;
	if (0 == xCnt)
	{
		Xavg = xData;
		return ;
	}
	
	if ((xData - Xavg) > AboveXAvg || (Xavg - xData) > AboveXAvg)
	{
		xSensorCnt++;
		if (xSensorCnt > 100)
		{
			/* 遇到阻碍，遇阻写标记位 */
			xSensorCnt = 0;
			gMotorMachine.DigitalAntiSmashingFlag = 1;
		}
	}
	else
	{
		xCnt++;
		gXDataBuffer[xCnt] = xData;
		if (xCnt > 10)
		{
			for (i = 0; i < 10; i++)
			{
				temple += gXDataBuffer[i];
			}
			temple /= 10;
			Xavg = temple;
		}
		xCnt = 0;
	}
}

void CheckCarEnteredFlag(void)
{
	//uint8_t i = 0;
	uint8_t pData[7];
	pData[0] = 0x5B;
	pData[1] = 0xE3;
	pData[3] = 0x00;
	pData[4] = 0x00;
	pData[6] = 0x5D;
	
	if (gCarEnteredFlag)
	{
		pData[2] = 0x00;//车辆入场
		pData[5] = 0xE3;
		gCarEnteredFlag = 0;
		BSP_SendDataToDriverBoard(pData, 7, 0xFFFF);
		return ;
	}
	
	if (gTimeoutFlag)
	{
		pData[2] = 0x01;//入场超时
		pData[5] = 0xE2;
		gTimeoutFlag = 0;
		BSP_SendDataToDriverBoard(pData, 7, 0xFFFF);
		return;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
