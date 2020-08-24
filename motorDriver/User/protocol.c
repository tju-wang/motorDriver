#include "protocol.h"

extern Motor_t M1;
extern DAC_HandleTypeDef hdac;
unsigned char UartFeedBackData[DataFdbkNum];
extern unsigned int gErrorStatus;


extern void    FLASH_PageErase(uint32_t PageAddress);
static FLASH_EraseInitTypeDef EraseInitStruct;
extern uint32_t FLASH_Address,PageError;
extern unsigned int FLASH_Store[FLASHSIZE];
extern unsigned int FLASH_Init[FLASHSIZE];

//adc数据
extern float outAdcData[2];	//外部两路adc值
extern float currAdcData[2];	//内部  电流adc值

extern unsigned char debugFLAG;
extern debugStruct_t debugVar;
//extern unsigned char debugPrintFlag;

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

char MotorDataFdbk(unsigned char ch)
{
	switch(ch)	
	{
		case CTL_EncoderFDBK:	{
			EncoderPWMFdbk();
		}break;
		case CTL_DAC_ADCFDBK:	{
			DacAdcVlueFdbk();
		}break;
		case CTL_SensorDataFDBK:	{
			//SensorFdbk(CTL_SensorDataFDBK);
		}break;
		case CTL_SensorPWMFDBK:	{
			//SensorFdbk(CTL_SensorPWMFDBK);
		}
		case FORCE_SWITCHFDBK:	{
			//ForceSwitchFdbk();
		}break;
	}
	return HAL_OK;
}
char EncoderPWMFdbk(void)
{
	unsigned char numm1,numm2,numm3,numm4;
	unsigned char state = 0;
	state = M1.runstate;
	state |= (M1.accDir<<2);
	ClearArr(UartFeedBackData,DataFdbkNum);
	UartFeedBackData[0] = 0x7B;
	UartFeedBackData[1]	= CTL_EncoderFDBK;
	DataLong2Char(&numm1,&numm2,&numm3,&numm4,M1.EnCounter);
	UartFeedBackData[2]	= (unsigned char)numm1; 
	UartFeedBackData[3] = (unsigned char)numm2;
	UartFeedBackData[4]	= (unsigned char)numm3;	
	UartFeedBackData[5]	= (unsigned char)numm4;	
	//pwm部分
	UartFeedBackData[6] = (unsigned char)((float)(TIM1->CCR1)/100.0f);
	UartFeedBackData[7] = (unsigned char)((TIM1->CCR1)%100);

	UartFeedBackData[8] = (unsigned char)((float)(TIM1->CCR2)/100.0f);
	UartFeedBackData[9] = (unsigned char)((TIM1->CCR2)%100);

	//方向  速度  加速度部分  
	UartFeedBackData[10]= (unsigned char)state;
	UartFeedBackData[11]= (unsigned char)((int)M1.speed);
	UartFeedBackData[12]= (unsigned char)((unsigned int)(M1.speed*100)%100);
	UartFeedBackData[13]= (unsigned char)(M1.accelration*10);
	UartFeedBackData[14]= (unsigned char)((unsigned int)(M1.accelration*1000)%100);

	return (UartSendData(UartFeedBackData,DataFdbkNum));		//发送数据
}

char DacAdcVlueFdbk(void)
{
	unsigned int tempDAC = 0;
	ClearArr(UartFeedBackData,DataFdbkNum);
	UartFeedBackData[0] = 0x7B;
	UartFeedBackData[1]	= CTL_DAC_ADCFDBK;
	
	tempDAC = HAL_DAC_GetValue(&hdac,DAC_CHANNEL_2);
	
	UartFeedBackData[2]	= (unsigned char)((float)tempDAC/100.f); 
	UartFeedBackData[3] = (unsigned char)(tempDAC%100);

	//adc返回
	UartFeedBackData[4] = (unsigned char)((float)outAdcData[0]/100.0f);
	UartFeedBackData[5] = (unsigned char)((unsigned int)outAdcData[0]%100);
	UartFeedBackData[6] = (unsigned char)((float)outAdcData[1]/100.0f);
	UartFeedBackData[7] = (unsigned char)((unsigned int)outAdcData[1]%100);
	
	UartFeedBackData[8] = (unsigned char)((float)currAdcData[0]/100.0f);
	UartFeedBackData[9] = (unsigned char)((unsigned char)currAdcData[0]%100);
	UartFeedBackData[10] = (unsigned char)((float)currAdcData[1]/100.0f);
	UartFeedBackData[11] = (unsigned char)((unsigned char)currAdcData[1]%100);

	UartFeedBackData[12]=  0;
	UartFeedBackData[13]= 0;
	
	return (UartSendData(UartFeedBackData,DataFdbkNum));		//发送数据
}

void ChangeDebug(unsigned char ch,unsigned int timeMicroSec)	//ch=1  打开调试   ch=0关闭调试 
{
	if(ch==1)
	{
		debugFLAG = 1;
		debugVar.timeFlag = 0;
		//解析延迟时间
		if(timeMicroSec==0)
			debugVar.timeDiff = timeMicroSec+1;	//不能 == 0
		else
			debugVar.timeDiff = timeMicroSec;
	}
	else if(ch==0)
	{
		debugFLAG = 0;
		debugVar.timeDiff = 0;
	}
	else if(ch==2)	//查询数据状态  0x10 未准备好   0x11 准备好
	{
		UartFeedBackData[0] = 0x7B;
		UartFeedBackData[1]	= CTL_DEBUGFDBK;
		UartFeedBackData[2]	= (unsigned char)debugVar.debugPrintFlag; 
		UartSendData(UartFeedBackData,DataFdbkNum);		//发送数据
	}
	else if(ch==3)
	{
		debugVar.debugPrintFlag = 2;	//开始输出
	}
}
void FlashChange(unsigned int data,unsigned int num)
{
	unsigned char mm;
//		(unsigned int)(UartRxData[1]%100+UartRxData[2]/100);	//分高低位
		//检测 初始化FLASH的值
		if(num>4)	{	//小于4 不允许修改
			for(mm=0;mm<FLASHSIZE;mm++)	{
				FLASH_Store[mm] = (*(unsigned int *)(FLASH_USER_START_ADDR+4*mm));	//mm为定位变量
			}
			mm = num-1;
			FLASH_Store[mm] = data;
		
			HAL_FLASH_Unlock();
			EraseInitStruct.Banks=FLASH_BANK_1;
			EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
			EraseInitStruct.Sector = FLASH_SECTOR_6;	//擦除扇区1  参考原子代码
			EraseInitStruct.NbSectors = 1;
			EraseInitStruct.VoltageRange=FLASH_VOLTAGE_RANGE_3;	//电压范围区间  2.7~3.6V之间
			
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)	{
			STMFLASH_Write(FLASH_USER_START_ADDR,(u32*)FLASH_Store,FLASHSIZE);
		}
		else	{
			gErrorStatus |= Error_FLASHErash;
		}
		HAL_FLASH_Lock();	//上锁
	
	}
}

void FlashInit(void)
{
	HAL_FLASH_Unlock();
	
	EraseInitStruct.Banks=FLASH_BANK_1;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Sector = FLASH_SECTOR_6;	//擦除扇区1  参考原子代码
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.VoltageRange=FLASH_VOLTAGE_RANGE_3;	//电压范围区间  2.7~3.6V之间
	
	
	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK)	{
		STMFLASH_Write(FLASH_USER_START_ADDR,(u32*)FLASH_Init,FLASHSIZE);
	}
	else	{
		gErrorStatus |= Error_FLASHErash;
	}
	HAL_FLASH_Lock();	//上锁
	
}
void FlashFDBK(char num)
{
	char mm = 0,part;
	unsigned char numm1,numm2,ii;
	int Summ;
	if(num>0&&num<=6)
		part = 1;
	else if(num>6&&num<=12)
		part = 2;
	else if(num>12&&num<=18)
		part = 3;
	else if(num>18&&num<=24)
		part = 4;
	
	for(mm=0;mm<FLASHSIZE;mm++)	{
				FLASH_Store[mm] = (*(unsigned int *)(FLASH_USER_START_ADDR+4*mm));	//mm为定位变量
			}
	if(part>0&&part<5)	{	//part 1-4
		
		for(mm=0;mm<6;mm++)	{
			DataCoverInt2Char(&numm1,&numm2,(int)FLASH_Store[(part-1)*6 + mm]);
			UartFeedBackData[mm*2+2]	= (unsigned char)numm1; 
			UartFeedBackData[mm*2+3] = (unsigned char)numm2;
		}
	
		}
	else	{
		for(mm=0;mm<6;mm++)	{
			UartFeedBackData[mm*2+2]= (unsigned char)0; 
			UartFeedBackData[mm*2+3] = (unsigned char)0;
		}
	}
	
	UartFeedBackData[0] = 0x7B;
	UartFeedBackData[1]	= 0x30;		//FLASH回读
	Summ = 0;
	for(ii=1; ii<DataFdbkNum-2;ii++)	{
		Summ +=UartFeedBackData[ii];
	}
	Summ = Summ%100;
	UartFeedBackData[14] = Summ;
	UartFeedBackData[15] = 0x7D;
	UartSendData(UartFeedBackData,16);
	//HAL_UART_Transmit(&huart2, (uint8_t *)&UartFeedBackData, DataFdbkNum, 0xFFFF);
}
void DataCoverInt2Char(unsigned char *pNum1,unsigned char *pNum2,int pNum_Int)
{
	if(pNum_Int<0)	//unaigned int
	{
		pNum_Int = -pNum_Int;
	}
	*pNum1 = (unsigned char)(pNum_Int/256);
	*pNum2 = (unsigned char)(pNum_Int%256);
}

//清除数组
void ClearArr(unsigned char *data,unsigned int len)
{
	while(len--)
	{
		(*data++) = 0;
	}
}

//返回数据   将long型数据转换为协议想要的格式
void DataLong2Char(unsigned char *pNum1,unsigned char *pNum2,unsigned char *pNum3,unsigned char *pNum4,long int Data)
{
	if(Data<0)
	{
		Data = -Data;
		*pNum1 = (unsigned char)(Data/16777216);	
		*pNum1 = *pNum1 | 0x80;
	}
	else	{
		*pNum1 = (unsigned char)(Data/16777216);
	}
	*pNum2 = (unsigned char)((float)(Data%16777216)/65536);
	*pNum3 = (unsigned char)(Data%65536/256);
	*pNum4 = (unsigned char)(Data%256);
}
