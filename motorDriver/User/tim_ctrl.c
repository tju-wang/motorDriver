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
extern float outAdcData[2];	//外部两路adc值
extern float currAdcData[2];	//内部  电流adc值
float tempVal[10] = {0};

//debug相关变量  都初始化为float类型  数据由串口输出
float debugData1[DEBUG_DATA_NUM] = {0};
float debugData2[DEBUG_DATA_NUM] = {0};
unsigned char debugFLAG=0;
unsigned int debugNum = 0;
unsigned char dataRecordFlag = 0;
debugStruct_t debugVar;


//定时器中断控制函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	if(htim->Instance == htim3.Instance)	{
		EncoderFlow(&M1,htim3);
	}
	if(htim->Instance == htim14.Instance)	//0.5ms 中断
	{
		IRQ_Counter++;
		if(IRQ_Counter>=0xFFFFFFFF)	//0.5ms中断  计时约596小时更新一次
			IRQ_Counter = 0;
		
		EncoderUpdate(&M1,htim3);
		GetOutAdc(outAdcData);
		GetCurrAdc(currAdcData);
		MotorState(&M1);
		DebugFun(debugFLAG);
		//HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		if(IRQ_Counter%2==0)	//暂定1ms控制周期
			CtrlMotor(M1.motorMode);	//控制模式
	}
}

/**************************************debug数据记录****************************************/
//可调参数  采样频率（时间）（最大深度提前设定）  采样数据类型：编码器 pwm 电机速度  加速度  采样开始时间可调
unsigned char DebugFun(unsigned char debugflag)		//debugflag  调试的开关
{
	if(dataRecordFlag==1)	{	//记录数据
		if(debugNum<DEBUG_DATA_NUM-2 && IRQ_Counter%DebugFreq==0)	{	//DebugFreq由flash参数指定
			debugData1[debugNum] = M1.EnCounter;
			debugData2[debugNum] = M1.speed;	
			debugNum++;			
		}
		else {
			if(debugNum>=DEBUG_DATA_NUM-2)	{
				dataRecordFlag = 0;
				debugVar.debugPrintFlag = 1;	//输出数据
				debugFLAG = 0;		//关闭调试模式
				debugNum = 0;
			}
		}
	}
	if(debugData1[DEBUG_DATA_NUM-2]!=0 || debugNum>=DEBUG_DATA_NUM-2)	{
			dataRecordFlag = 0;
			debugVar.debugPrintFlag = 1;	//输出数据
			debugFLAG = 0;		//关闭调试模式
			debugNum = 0;
	}
	//开启条件  暂定为延时条件 由flash参数指定
	if(debugflag)	//定义计时部分  在时间到达后打开数据记录标志、清空需要检测到倒数第二位debugData1[]数组
	{
		if(debugVar.timeFlag == 0)	//关闭flag 记录当前时间
		{
			debugVar.timeNow = IRQ_Counter;
			debugVar.timeFlag = 1;	//只有在打开debugflag时，才将 timeFlag 设置为1
			if(debugVar.timeDiff!=0)
			{
				debugVar.timeTar = debugVar.timeNow + debugVar.timeDiff*2;	//转换为ms延时
			}
		}
		
		if(debugVar.timeTar==IRQ_Counter&&debugVar.timeFlag==1)	//到达等待时间 开始记录数据
		{
			dataRecordFlag = 1;
//			debugVar.timeFlag = 0;
		}
	}
	return HAL_OK;
	
}


/**************************************电机状态检测****************************************/
//检测电机状态  运动方向、速度、加速度方向、加速度检测
unsigned char MotorState(Motor_t *pMotor)	//电机状态监测  改变runstate及speed值  计算加速度值
{
	long long int Sum1,Sum2;
	float speedSum1,speedSum2;
	char parr,kk;
	parr = pMotor->pArr+EncoderStoreNum;	//在每一次编码器值更新的时候  pArr ++ 
	Sum1 = Sum2 = 0;
	for(kk=0; kk<5; kk++)	{
		Sum1 += pMotor->EnCoterArr[(parr-kk)%EncoderStoreNum];
		Sum2 += pMotor->EnCoterArr[(parr+kk+1)%EncoderStoreNum];
	}	
	pMotor->speed = (float)((Sum1-Sum2)*2*1000*60/(REDUCTION*5*15.0f*4095));	//1ms 转过的counter数
	if((pMotor->speed)<-StateRef)	{
		pMotor->runstate = -1;
		pMotor->speed = -pMotor->speed;	 //在计算完加速度后 再改变速度的符号
	}
	else if((pMotor->speed)>StateRef)	{
		pMotor->runstate = 1;
	}
	else	{
		pMotor->runstate = 0;
	}
	if(IRQ_Counter%ACCE_NUM==0)
	{
		pMotor->pSpArr +=1;
		pMotor->pSpArr %=EncoderStoreNum;
		pMotor->MoterSpeedArr[pMotor->pSpArr] = pMotor->speed;	//存储之前的速度数据
		//计算加速度
		parr = pMotor->pSpArr+EncoderStoreNum;	//在每一次编码器值更新的时候  pArr ++ 
		speedSum1 = speedSum2 = 0;
		for(kk=0; kk<5; kk++)	{
			speedSum1 += pMotor->MoterSpeedArr[(parr-kk)%EncoderStoreNum];
			speedSum2 += pMotor->MoterSpeedArr[(parr+kk+11)%EncoderStoreNum];
		}
		tempVal[0] = speedSum1 - speedSum2;
		pMotor->accelration = (speedSum1 - speedSum2)*ACCEN_NUM/(5*15.0f);	//单位  rpm/s
		if(pMotor->accelration<0)	{
			pMotor->accelration = -pMotor->accelration;
			pMotor->accDir = 0;
		}
		else {
			pMotor->accDir = 1;
		}
	}
	
	return 1;
	
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

/**************************************更新ID状态****************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	//按键检测优先级为低  （4,1）
{
  /* Prevent unused argument(s) compilation warning */
	UNUSED(GPIO_Pin);
		
	M1.motor_id = GetID();
}

unsigned char GetID(void)
{
	static int tt = 10000;
	unsigned char id = 0;
	unsigned char temp = 0;
//	tt = 1000;
//	while(tt)	{	//debug 时间约32ms  按键检测
//		tt--;
//	}
	temp = HAL_GPIO_ReadPin(ID1_GPIO_Port,ID1_Pin);
	id += (temp<<0);
	temp = HAL_GPIO_ReadPin(ID2_GPIO_Port,ID2_Pin);
	id += (temp<<1);
	temp = HAL_GPIO_ReadPin(ID3_GPIO_Port,ID3_Pin);
	id += (temp<<2);

	return id;
}
	




