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
extern float outAdcData[2];	//�ⲿ��·adcֵ
extern float currAdcData[2];	//�ڲ�  ����adcֵ
float tempVal[10] = {0};

//debug��ر���  ����ʼ��Ϊfloat����  �����ɴ������
float debugData1[DEBUG_DATA_NUM] = {0};
float debugData2[DEBUG_DATA_NUM] = {0};
unsigned char debugFLAG=0;
unsigned int debugNum = 0;
unsigned char dataRecordFlag = 0;
debugStruct_t debugVar;
extern long long int gPosTar;

//��ʱ���жϿ��ƺ���
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
{
	if(htim->Instance == htim3.Instance)	{
		EncoderFlow(&M1,htim3);
	}
	if(htim->Instance == htim14.Instance)	//0.5ms �ж�
	{
		IRQ_Counter++;
		if(IRQ_Counter>=0xFFFFFFFF)	//0.5ms�ж�  ��ʱԼ596Сʱ����һ��
			IRQ_Counter = 0;
		
		EncoderUpdate(&M1,htim3);
		GetOutAdc(outAdcData);
		GetCurrAdc(currAdcData);
		MotorState(&M1);
		DebugFun(debugFLAG);
		if(IRQ_Counter%2==0)	//�ݶ�1ms��������
			CtrlMotor(M1.motorMode);	//����ģʽ
		if(IRQ_Counter%1000==0)
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	}
}

/**************************************debug���ݼ�¼****************************************/
//�ɵ�����  ����Ƶ�ʣ�ʱ�䣩����������ǰ�趨��  �����������ͣ������� pwm ����ٶ�  ���ٶ�  ������ʼʱ��ɵ�
unsigned char DebugFun(unsigned char debugflag)		//debugflag  ���ԵĿ���
{
	if(dataRecordFlag==1)	{	//��¼����
		if(debugNum<DEBUG_DATA_NUM-2 && IRQ_Counter%DebugFreq==0)	{	//DebugFreq��flash����ָ��
			debugData1[debugNum] = M1.EnCounter;
			debugData2[debugNum] = M1.speed;	
			debugNum++;			
		}
		else {
			if(debugNum>=DEBUG_DATA_NUM-2)	{
				dataRecordFlag = 0;
				debugVar.debugPrintFlag = 1;	//�������
				debugFLAG = 0;		//�رյ���ģʽ
				debugNum = 0;
			}
		}
	}
	if(debugData1[DEBUG_DATA_NUM-2]!=0 || debugNum>=DEBUG_DATA_NUM-2)	{
			dataRecordFlag = 0;
			debugVar.debugPrintFlag = 1;	//�������
			debugFLAG = 0;		//�رյ���ģʽ
			debugNum = 0;
	}
	//��������  �ݶ�Ϊ��ʱ���� ��flash����ָ��
	if(debugflag)	//�����ʱ����  ��ʱ�䵽�������ݼ�¼��־�������Ҫ��⵽�����ڶ�λdebugData1[]����
	{
		if(debugVar.timeFlag == 0)	//�ر�flag ��¼��ǰʱ��
		{
			debugVar.timeNow = IRQ_Counter;
			debugVar.timeFlag = 1;	//ֻ���ڴ�debugflagʱ���Ž� timeFlag ����Ϊ1
			if(debugVar.timeDiff!=0)
			{
				debugVar.timeTar = debugVar.timeNow + debugVar.timeDiff*2;	//ת��Ϊms��ʱ
			}
		}
		
		if(debugVar.timeTar==IRQ_Counter&&debugVar.timeFlag==1)	//����ȴ�ʱ�� ��ʼ��¼����
		{
			dataRecordFlag = 1;
//			debugVar.timeFlag = 0;
		}
	}
	return HAL_OK;
	
}


/**************************************���״̬���****************************************/
//�����״̬  �˶������ٶȡ����ٶȷ��򡢼��ٶȼ��
unsigned char MotorState(Motor_t *pMotor)	//���״̬���  �ı�runstate��speedֵ  ������ٶ�ֵ
{
	long long int Sum1,Sum2;
	float speedSum1,speedSum2;
	char parr,kk;
	parr = pMotor->pArr+EncoderStoreNum;	//��ÿһ�α�����ֵ���µ�ʱ��  pArr ++ 
	Sum1 = Sum2 = 0;
	for(kk=0; kk<5; kk++)	{
		Sum1 += pMotor->EnCoterArr[(parr-kk)%EncoderStoreNum];
		Sum2 += pMotor->EnCoterArr[(parr+kk+1)%EncoderStoreNum];
	}	
	pMotor->speed = (float)((Sum1-Sum2)*2*1000*60/(REDUCTION*5*15.0f*4095));	//1ms ת����counter��
	if((pMotor->speed)<-StateRef)	{
		pMotor->runstate = -1;
		pMotor->speed = -pMotor->speed;	 //�ڼ�������ٶȺ� �ٸı��ٶȵķ���
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
		pMotor->MoterSpeedArr[pMotor->pSpArr] = pMotor->speed;	//�洢֮ǰ���ٶ�����
		//������ٶ�
		parr = pMotor->pSpArr+EncoderStoreNum;	//��ÿһ�α�����ֵ���µ�ʱ��  pArr ++ 
		speedSum1 = speedSum2 = 0;
		for(kk=0; kk<5; kk++)	{
			speedSum1 += pMotor->MoterSpeedArr[(parr-kk)%EncoderStoreNum];
			speedSum2 += pMotor->MoterSpeedArr[(parr+kk+11)%EncoderStoreNum];
		}
		tempVal[0] = speedSum1 - speedSum2;
		pMotor->accelration = (speedSum1 - speedSum2)*ACCEN_NUM/(5*15.0f);	//��λ  rpm/s
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
/**************************************����������****************************************/
void EncoderUpdate(Motor_t *pEncoder, TIM_HandleTypeDef htim)	//����������ֵ����  ������������鵱��	
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

/**************************************����ID״̬****************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)	//����������ȼ�Ϊ��  ��4,1��
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
//	while(tt)	{	//debug ʱ��Լ32ms  �������
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
	




