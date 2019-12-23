/**
  ******************************************************************************
  * @file           : tim_ctrl.c
  * @brief          : control motor
  *	@author			: Mrwang_tju  2019.12.22
  *	@brief			: Motor control for STM32F405RG
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
#include "tim_ctrl.h"

Motor_t M1;
unsigned long int IRQ_Counter = 0;


//定时器中断控制函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	if(htim->Instance == htim3.Instance)	{
		EncoderFlow(&M1,htim3);
	}
	if(htim->Instance == htim14.Instance)	//250us 中断
	{
		IRQ_Counter++;
		if(IRQ_Counter>=0xFFFFFFFF)
			IRQ_Counter = 0;
		
		EncoderUpdate(&M1,htim3);
	}
}


/**************************************编码器计数****************************************/

void EncoderUpdate(Motor_t *pEncoder, TIM_HandleTypeDef htim)	//编码器计数值更新  存入编码器数组当中	
{
	pEncoder->EnCounter = (long int)((pEncoder->EnOverflowNum)*ENCODER_NUM+__HAL_TIM_GET_COUNTER(&htim));
	pEncoder->EnCoterArr[pEncoder->pArr] = pEncoder->EnCounter;
	pEncoder->pArr +=1;
	pEncoder->pArr %=EncoderStoreNum;
}

unsigned char EncoderFlow(Motor_t* pEncoder, TIM_HandleTypeDef htim)
{
	pEncoder->tEnCounter = __HAL_TIM_GET_COUNTER(&(htim)); 	
	if(pEncoder->tEnCounter<ENCODER_NUM_2)	{
		pEncoder->EnOverflowNum ++;
	}
	else 	{
		pEncoder->EnOverflowNum --;
	}
	pEncoder->EnCounter = (long int)((pEncoder->EnOverflowNum)*ENCODER_NUM+__HAL_TIM_GET_COUNTER(&htim));
	return 1;
}




