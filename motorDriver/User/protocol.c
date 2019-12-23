#include "protocol.h"

//解析电脑返回的力的数据
//char AnalysisForce(unsigned char *data)
//{
//	//检验通过后 直接赋给三路重力补偿项的PWM
//	
//}

//extern MotorGriverForce_t MotorForce;
//extern float mCos[3];
////发送杆长数据给电脑
//char SendPoleLen(var_q poleLen)
//{
//	unsigned char data[SEND_SIZE] = {0};
//	char status = HAL_OK;
//	data[0] = 0x7B;
//	data[SEND_SIZE-1] = 0x7D;
//	data[1] = 0;
//	data[2] = READPoleLen;
//	PoleLen2char(poleLen.q1,&data[3],&data[4]);
//	PoleLen2char(poleLen.q2,&data[5],&data[6]);
//	PoleLen2char(poleLen.q3,&data[7],&data[8]);
//	data[9] = 0;
//	data[10] = 0;
//	data[11] = 0;
//	data[12] = checkCalc(data,SEND_SIZE);
//	status = CDC_Transmit_FS(data, SEND_SIZE);
//	return status;
//	
//}

/*

//数据转换函数  杆长长度转换为发送的unsigned char类型
unsigned char PoleLen2char(float len,unsigned char *data1, unsigned char *data2)
{
	*data1 = (unsigned char)(len/10.0);
	*data2 = (unsigned char)((unsigned int)(len*10)%100);
	return 0;
}


unsigned char checkCalc(unsigned char* data, int len)
{
	unsigned char ret = HAL_OK;
	unsigned long sum = 0;
	for(int i=1;i<len-2;i++)
	{
		sum += data[i];
	}
	sum = sum%100;
	ret = sum;
	return ret;
}

void clearArr(unsigned char *arr,unsigned int len)
{
	for(int i=0;i<len;i++)
	{
		arr[i] = 0;
	}
}
////解析上位机发来的数据
//unsigned char AnalysisForceData(unsigned char *data,unsigned char len)
//{
//	float f[3] = {0.0};
//	unsigned int cnt = 0;
//	for(cnt=0;cnt<3;cnt++)
//	{
//		ForceDouble2Char(&f[cnt],data[cnt*3+3],data[cnt*3+4],data[cnt*3+5]);
//	}
//	MotorForce.F1 = f[0];
//	MotorForce.F2 = f[1];
//	MotorForce.F3 = f[2];
//	//解析cos  支链与动平台间的夹角
//	for(cnt=0;cnt<3;cnt++)
//	{
//		CosFloat2Char(&(mCos[cnt]),data[cnt*2+12],data[cnt*2+13]);
//	}
//	return 0;
//}
float CosFloat2Char(float *mCos,unsigned char data1,unsigned char data2)
{
	(*mCos) = (((float)data1)/100.0f) + (((float)data2)/10000.0f);
//	(*mCos) = 1;
	return *mCos;
}
float ForceDouble2Char(float *force,unsigned char data1,unsigned char data2,unsigned char data3)
{
	char negativeNumFlag = 0;
	if(data1&(0x80))
	{
		negativeNumFlag = 1;
		data1 &= ~(0x80);		//置最高位为0
	}
	*force = (float)(data1*1000 + data2*10 + (float)data3*0.1f);
	if(negativeNumFlag)
		*force = -(*force);
	return (*force);
}

*/


//板子上电 使能
void BoardEn(char ch)
{
	if(ch==1)		{
		HAL_GPIO_WritePin(EnMotor_GPIO_Port,EnMotor_Pin,GPIO_PIN_SET);
	}
	else 	{
		HAL_GPIO_WritePin(EnMotor_GPIO_Port,EnMotor_Pin,GPIO_PIN_RESET);
	}
}
