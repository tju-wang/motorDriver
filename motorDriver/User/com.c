#include "com.h"

extern DAC_HandleTypeDef hdac;
extern Motor_t M1;
extern MotorCtrl_t M1Ctrl;
extern unsigned char gUsartFlag;
//通讯硬件部分  数据收发
static unsigned char Uart_counter = 0;  //计数
static char UartStartFlag = 0;
unsigned char UartRxData[9]; //缓存数据数组
unsigned char UartRxData_2[9]; //缓存数据数组
static unsigned int upwm;


/****************串口数据收发******************/
void USART2Interrupt(char UartRxBuf)		//参数  状态控制串口
{
	static unsigned char i =0;
	unsigned char counter = 0;	
	static unsigned int scheck = 0;
	
	if(UartRxBuf=='}')	{
		if(Uart_counter==9) 	{
			scheck = 0;
			for(i=0;i<8;i++)	{
				scheck +=UartRxData[i];
			}
			scheck = scheck%100;
			if(scheck==UartRxData[8])	{
			//将数据存入二级缓存
			for(counter=0;counter<9;counter++)
			{
				UartRxData_2[counter] = UartRxData[counter];
			}
			gUsartFlag = 1;

		}	
		}
		Uart_counter = 0;
		UartStartFlag = 0;
	}

	if(UartStartFlag==1)
	{
		if(Uart_counter>8)  UartStartFlag = 0;
		else 
		{
			UartRxData[Uart_counter] = UartRxBuf;  //存入数组
			Uart_counter++;
		}	
	}
	if(UartRxBuf=='{')
	{
		UartStartFlag = 1;
		Uart_counter = 0;
	}
}

//协议处理函数
char ProtocAnalsis(void)
{
	unsigned int tempValue = 0;
	switch(UartRxData_2[1])
	{
		case CMD_EN:	{
			if(UartRxData_2[2]==0x11)		{
				BoardEn(1);
			}
			else 	{
				BoardEn(0);
			}
		}break;
		case CMD_DEBUG:	{
			ChangeDebug(UartRxData_2[2],UartRxData_2[3]*100+UartRxData_2[4]);
		}break;
		case CMD_PWMSET:	{
			upwm = UartRxData_2[2]*100+UartRxData_2[3];
			//正反转  PWM占空比   设置寄存器
			if(UartRxData_2[0]!=MOTORSTOP)
			{
				if(UartRxData_2[0]==CLOCKWISE)	//顺时针
					setPWMClockwise(upwm);
				else if(UartRxData_2[0]==ANTICLOCKWISE)	//逆时针
					setPWMAitClockwise(upwm);
			}
			else	//阻尼
			{
				setZeroPWM();
			}
		}break;
		case CMD_SPEEDSET:
		{	//目标速度
			M1Ctrl.speedTar = UartRxData_2[2]*10+UartRxData_2[3]*0.1;
			//正反转  PWM占空比   设置寄存器
//					if(UartRxData_2[0]!=MOTORSTOP)
//					{
//						if(UartRxData_2[0]==CLOCKWISE)	//顺时针
//							setPWMClockwise(upwm);
//						else if(UartRxData_2[0]==ANTICLOCKWISE)	//逆时针
//							setPWMAitClockwise(upwm);
//					}
//					else	//阻尼
//					{
//						setZeroPWM();
//					}
		}break;
//		case CMD_PLANTRAJ:
//		{
//			
//			PlanTraj(gPosTar,70,1);
//		}break;
		case CMD_DACSET:
		{
			tempValue = UartRxData_2[2]*100+UartRxData_2[3];
			if(tempValue<=4095)	{  //判断DAC取值范围
				HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2,DAC_ALIGN_12B_R,tempValue);	//设置DAC输出
			}
			
		}break;
		case CMD_FDBK:	{
			MotorDataFdbk(UartRxData_2[2]);
		}break;
		case CMD_FLASH:	{
			switch(UartRxData_2[2])	{
				case CTL_FLASHCHANGE:	{
					FlashChange((UartRxData_2[4]*256+UartRxData_2[5]),UartRxData_2[3]);
				}break;
				case CTL_FLASHFDBK:	{
					FlashFDBK(UartRxData_2[3]);
				}break;
				case CTL_FLASHINIT:	{
					FlashInit();
				}break;
				default :
					break;
			}	
		}break;
		case CMD_SENSOR:
		{
//				if(UartRxData_2[2]==0x10)		{	//不需要将传感器结果返回上位机
//					SensorCtrl(UartRxData_2[3],UartRxData_2[4]);
//				}
			switch (UartRxData_2[2]){
				case CTL_CALIBRATE:	//校准传感器
				{
				}break;
				default :
					break;
			}
			
		}break;				
		default :
		{}break;
	}
	return HAL_OK;
	
}

//对长度与 DataFdbkNum 相等的做数据校验  不相等直接发送
char UartSendData(unsigned char *data,unsigned int len)
{
	unsigned int i,summ = 0;
	if(len==DataFdbkNum)
	{
		for(i=1;i<DataFdbkNum-2;)	{
			summ += data[i++];
		}
		summ = summ%100;
		data[14] = summ;
		data[0]  = 0x7B;
		data[15] = 0x7D;
	}
	return (HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 0xFFFF));
}



