/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "./../User/include.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADCOUT1_Pin GPIO_PIN_0
#define ADCOUT1_GPIO_Port GPIOA
#define ADCOUT2_Pin GPIO_PIN_1
#define ADCOUT2_GPIO_Port GPIOA
#define DAC_Pin GPIO_PIN_5
#define DAC_GPIO_Port GPIOA
#define EncoderA_Pin GPIO_PIN_6
#define EncoderA_GPIO_Port GPIOA
#define EncoderB_Pin GPIO_PIN_7
#define EncoderB_GPIO_Port GPIOA
#define EncoderI_Pin GPIO_PIN_4
#define EncoderI_GPIO_Port GPIOC
#define EncoderI_EXTI_IRQn EXTI4_IRQn
#define ADC1Cur_Pin GPIO_PIN_0
#define ADC1Cur_GPIO_Port GPIOB
#define ADC2FCur_Pin GPIO_PIN_1
#define ADC2FCur_GPIO_Port GPIOB
#define FAU_Pin GPIO_PIN_8
#define FAU_GPIO_Port GPIOC
#define FAU_EXTI_IRQn EXTI9_5_IRQn
#define EnMotor_Pin GPIO_PIN_9
#define EnMotor_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_8
#define PWM2_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOC
#define CAN_RS_Pin GPIO_PIN_3
#define CAN_RS_GPIO_Port GPIOB
#define ID1_Pin GPIO_PIN_4
#define ID1_GPIO_Port GPIOB
#define ID2_Pin GPIO_PIN_5
#define ID2_GPIO_Port GPIOB
#define ID3_Pin GPIO_PIN_6
#define ID3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void startUp(void);
void EncoderClear(void);
void CtrlStruct_Init(void);
void ParaInit(void);
void StartUp(void);

void clearArr(unsigned char *arr,unsigned int len);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
