/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#define ADC_OUT_NUM			2
#define ADC_OUT_ARR_2		3
#define	ADC_OUT_ARR			6

float outAdcData[2];	//外部两路adc值
float currAdcData[2];	//currAdcData[0]为未滤波adc值   currAdcData[1]为滤波之后的adc值
float tempValue[10];


/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration    
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    GPIO_InitStruct.Pin = ADC1Cur_Pin|ADC2FCur_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC3 GPIO Configuration    
    PA0-WKUP     ------> ADC3_IN0
    PA1     ------> ADC3_IN1 
    */
    GPIO_InitStruct.Pin = ADCOUT1_Pin|ADCOUT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9 
    */
    HAL_GPIO_DeInit(GPIOB, ADC1Cur_Pin|ADC2FCur_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PA0-WKUP     ------> ADC3_IN0
    PA1     ------> ADC3_IN1 
    */
    HAL_GPIO_DeInit(GPIOA, ADCOUT1_Pin|ADCOUT2_Pin);

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
//传感器数据采样   返回用作控制的adc的值
/*
转换时间： (56 cycles + 12) *3 *3 / 21M  = 29.14 us
*/
char GetOutAdc(float *sensorData)  //传入两个数组
{
//	int cnt = 0;
	char status = HAL_OK;
	status = GetAdc3(sensorData,2);
	
	return status;
}
//返回平均后的三路adc的值
char GetAdc3(float *data,int len)
{
	float adc[ADC_OUT_ARR];	//两组
	unsigned int adcTemp[ADC_OUT_NUM];	//临时存储数据
	float adcSum[ADC_OUT_NUM] = {0};
	int i = 0;
	int cnt = 0;
	for(cnt=0;cnt<ADC_OUT_ARR_2;cnt++)
	{
		if(Adc3Read(&adcTemp[0],2)==HAL_OK)
		{
			for(i=0;i<ADC_OUT_NUM;i++)
			{
				adc[cnt*ADC_OUT_NUM+i] = adcTemp[i];
			}
		}
		else	{
			return HAL_ERROR;
		}
	}
	for(cnt=0;cnt<ADC_OUT_ARR;cnt++)
	{
		adcSum[cnt%ADC_OUT_NUM] += adc[cnt];
	}
	for(cnt=0;cnt<ADC_OUT_NUM;cnt++)
	{
		data[cnt] = adcSum[cnt]/ADC_OUT_ARR_2;
	}
	return HAL_OK;
}
//触发单次的adc转换   单次转换一遍
char Adc3Read(unsigned int *adc,unsigned int len)
{
	int cnt = 0;
	for(cnt=0;cnt<len;cnt++)
	{
		HAL_ADC_Start(&hadc3);
		if(HAL_ADC_PollForConversion(&hadc3,0xFF)==HAL_OK)	//一次转换一组  需要读取三次数据
		{
			adc[cnt] = HAL_ADC_GetValue(&hadc3);
			tempValue[cnt] = adc[cnt];
		}
		else
			return HAL_ERROR;
	}
	return HAL_OK;
}


//获取电流adc数据
char GetCurrAdc(float *sensorData)  //传入两个数组
{
//	int cnt = 0;
	char status = HAL_OK;
	status = GetAdc1(sensorData,2);
	
	return status;
}
//返回平均后的两路adc的值
char GetAdc1(float *data,int len)
{
	float adc[ADC_OUT_ARR];	//两组
	unsigned int adcTemp[ADC_OUT_NUM];	//临时存储数据
	float adcSum[ADC_OUT_NUM] = {0};
	int i = 0;
	int cnt = 0;
	for(cnt=0;cnt<ADC_OUT_ARR_2;cnt++)
	{
		if(Adc1Read(&adcTemp[0],2)==HAL_OK)
		{
			for(i=0;i<ADC_OUT_NUM;i++)
			{
				adc[cnt*ADC_OUT_NUM+i] = adcTemp[i];
			}
		}
		else	{
			return HAL_ERROR;
		}
	}
	for(cnt=0;cnt<ADC_OUT_ARR;cnt++)
	{
		adcSum[cnt%ADC_OUT_NUM] += adc[cnt];
	}
	for(cnt=0;cnt<ADC_OUT_NUM;cnt++)
	{
		data[cnt] = adcSum[cnt]/ADC_OUT_ARR_2;
	}
	return HAL_OK;
}
//触发单次的adc转换   单次转换一遍  内部电流adc
char Adc1Read(unsigned int *adc,unsigned int len)
{
	int cnt = 0;
	for(cnt=0;cnt<len;cnt++)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1,0xFF)==HAL_OK)	//一次转换一组  需要读取三次数据
		{
			adc[cnt] = HAL_ADC_GetValue(&hadc1);
			tempValue[cnt] = adc[cnt];
		}
		else
			return HAL_ERROR;
	}
	return HAL_OK;
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
