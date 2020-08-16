#ifndef __COM_H
#define __COM_H

#include "include.h"
#include "protocol.h"


#define DataFdbkNum		(16)	//协议返回 数据长度

//协议相关宏
//CMD为大类的指令    
#define 	CMD_EN 			0x10	//是否使能驱动芯片
#define		CMD_PWMSET		0x20	//直接外部指定PWM
#define		CMD_DACSET		0x21	//DAC决定流过驱动芯片的最大电流  过大  截断
#define		CMD_SPEEDSET	0x22	//外部设置目标速度
#define		CMD_PLANTRAJ	0x23	//外部指定轨迹规划参数
#define     CMD_PRE_PLANTRAJ 0x26	//轨迹规划预设
#define		CMD_PRE_EXEC	0x27	//轨迹规划执行

#define		CMD_FDBK		0x30
//#define		CMD_PWMFDBK		0x31
#define 	CMD_DEBUG		0x80


#define		MOTOR_1			0x01
#define     BOARDCAST		0x79

#define 	CMDD_EN			0x11
#define		CMDD_DEN		0x10


#define 	CMD_CTRLMODE	0x40	//控制模式  
#define		CTL_FREE		0x10	//free Mode
#define		CTL_ENABLE		0x11	//board enable
#define 	CTL_PUSHPULL	0x12	//
#define		CTL_SINGMOTOR	0x13	//单个电机补偿 single

#define		CMD_FLASH		0x50	//FLASH相关
#define		CMD_SENSOR		0x60	//传感器相关指令

#define		CTL_FLASHFDBK		0x10	//FLASH回读
#define		CTL_FLASHCHANGE		0x11
#define		CTL_FLASHINIT		0x13
#define		CTL_FLASHINIT_CHANGE	0x14  //修改FLASH_Init 数组

#define		CMD_ForceSwitch		0x41	//不同种类力补偿类型
#define		CTL_StaFreFlag		0x11
#define		CTL_DynFreFlag		0x12
#define		CTL_GraFlag			0x13
#define		CTL_InerFlag		0x14
#define 	CTL_SensFlag		0x15	//基于传感器数据

#define 	CTL_EncoderFDBK		0x20
#define		CMD_PWMFDBK			0x40	
#define 	CTL_DAC_ADCFDBK		0x60
#define		CTL_DEBUGFDBK		0x81

#define 	CTL_SensorPWMFDBK	0x30	//返回传感器  PID运算后的电机数据
#define		CTL_SensorDataFDBK	0x31	//返回传感器  原始数据
#define		FORCE_SWITCHFDBK	0x41	//返回当前补偿力的状况

#define		CTL_CALIBRATE		0x61	//传感器校准


void USART2Interrupt(char UartRxBuf);
char UartSendData(unsigned char *data,unsigned int len);
char ProtocAnalsis(void);

#endif

