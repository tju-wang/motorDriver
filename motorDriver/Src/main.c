/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *	@author			: Mrwang_tju  2019.12.20
  *	@brief			: Motor driver for STM32F405RG
  ******************************************************************************
	BSD 3-Clause License

	Copyright (c) 2019, MrWang
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.

	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	3. Neither the name of the copyright holder nor the names of its
	   contributors may be used to endorse or promote products derived from
	   this software without specific prior written permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
	FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
	SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
	OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
	OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint8_t RXBuffer[1];
extern Motor_t M1;
extern MotorCtrl_t M1Ctrl;
extern float debugData1[DEBUG_DATA_NUM];
extern float debugData2[DEBUG_DATA_NUM];
extern debugStruct_t debugVar;
extern unsigned char debugFLAG;
extern unsigned char gUsartFlag;

unsigned int gErrorStatus;	//全局变量   用来显示错误提示
uint32_t FLASH_Address = 0, PageError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

unsigned int FLASH_Store[FLASHSIZE];	//修改FLASH时临时存储数组
unsigned int FLASH_Init[FLASHSIZE]={
	1,	//产品型号		1
	01,	//软件版本		2
	00,	//机械结构版本	3
	01,	//电路硬件版本	4
	
	1,	//Ctrl_P  		5		控制过程P参数
	1,	//Ctrl_I		6
	0,	//Ctrl_D 		7
	10,	//DebugFreq		8		调试状态下的采样频率
	99,//PWM_MAX		9		电机最大pwm
	0,	//PWM_MIN		10		最小pwm
	0,	//				11								
	0,	//				12
	0,	//				13
	
	0,	//				14
	
	0,//				15	
	0,//				16
	0,//				17
	0,//				18

	
	0x00,	//HardWare Version	19
	0x00,	//SoftWare Version	20
	0x00,	//HardWare Version	21
	0x00,	//SoftWare Version	22
	0x00,	//HardWare Version	23
	0x00	//SoftWare Version	24
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  StartUp();
  unsigned int numm,kk;
  
  
  HAL_Delay(1000);
  PlanTraj(M1.EnCounter+59230,70,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(gUsartFlag==1)
	{
		gUsartFlag = 0;
		ProtocAnalsis();
	}
	if(debugVar.debugPrintFlag!=0)
	{
		if(debugVar.debugPrintFlag==2)	//开始输出数据
		{
			debugFLAG = 0;
			debugVar.debugPrintFlag = 0;
			//关闭中断
			__disable_irq(); //off INT

			printf("*********输出开始************\n");
			for(numm=0;numm<DEBUG_DATA_NUM;numm++)	{
				printf("     %3d      %6.3f       %6.3f      \n",numm,debugData1[numm],debugData2[numm]);
			}
			printf("*********输出完毕************\n");
			ClearFloatArr(debugData1,DEBUG_DATA_NUM);
			ClearFloatArr(debugData2,DEBUG_DATA_NUM);
			__enable_irq(); // on INT 
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void StartUp(void)
{
	ParaInit();
	
	HAL_UART_Receive_IT(&huart2, (uint8_t *)RXBuffer, 1); //打开中断接收

	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,4000);	//设置DAC输出
	HAL_DAC_Start(&hdac,DAC1_CHANNEL_2);

	HAL_TIM_Base_Start_IT(&htim3);

	HAL_GPIO_WritePin(EnMotor_GPIO_Port,EnMotor_Pin,GPIO_PIN_SET);
	M1.motor_id = GetID();
	
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

	M1.EnCounter = 0;
	M1.EnOverflowNum = 0;
	M1.motorMode = CTRL_MODE;
	M1Ctrl.CtrlLastPWM = 0;
	M1Ctrl.CtrlPWM = 0;
	
	//中断定时器
	HAL_TIM_Base_Start_IT(&htim14);	//开启500us中断
}

void ParaInit(void)
{
	EncoderClear();
	
	char mm,Flag_blank = 0;	//FLASH是否为空白  标志
	int Flash_Value;
//	 HAL_StatusTypeDef FlashStatus=HAL_OK;
	FLASH_Address = FLASH_USER_START_ADDR;
	for(mm=0;mm<FLASHSIZE;mm++)	{
			Flash_Value = *(int*)(FLASH_USER_START_ADDR+(mm)*4);
			if((Flash_Value==-1)&&(Flag_blank==0))
				Flag_blank = 0;		//此FLASH区域为空白
			else 	{
				Flag_blank = 1;  //非空白	
				break;
			}
		}
	if(Flag_blank==0)	{
		STMFLASH_Write(FLASH_USER_START_ADDR,(u32*)FLASH_Init,FLASHSIZE);
	}
}


void EncoderClear(void)
{
	//结构体初始化
	M1.EnCounter = M1.EnOverflowNum = 0;
	__HAL_TIM_SET_COUNTER(&htim3,0);
	//溢出值需要清零 在打开TIM3中断后会进入一次全局中断
	M1.EnOverflowNum= 0;
}

//清除数组
void ClearFloatArr(float *data,unsigned int len)
{
	while(len--)
	{
		(*data++) = 0;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
