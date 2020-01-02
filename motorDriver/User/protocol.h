#ifndef _PROTOCOL_H
#define _PROTOCOL_H


#include "include.h"
#include "com.h"
//#include "./../User/tim_ctrl.h"

#define Error_FLASHErash		(1<<20)	//第20位  置1表示FLASH擦除错误

#define	RECEIVE_SIZE	25		//单片机接受协议长度
#define	SEND_SIZE		14		//单片机发送协议长度

void BoardEn(char ch);
char EncoderPWMFdbk(void);
char DacAdcVlueFdbk(void);
void ClearArr(unsigned char *data,unsigned int len);
char MotorDataFdbk(unsigned char ch);

void FlashFDBK(char num);
void FlashInit(void);
void FlashChange(unsigned int data,unsigned int num);

void ChangeDebug(unsigned char ch,unsigned int timeMicroSec);

void DataCoverInt2Char(unsigned char *pNum1,unsigned char *pNum2,int pNum_Int);
void DataLong2Char(unsigned char *pNum1,unsigned char *pNum2,unsigned char *pNum3,unsigned char *pNum4,long int Data);
/*************协议相关宏****************/
/*
#define		READPoleLen			0x30		//读取杆长
#define 	GETForceMessage		0x20		//获得反馈力的信息


unsigned char checkCalc(unsigned char* data, int len);
void clearArr(unsigned char *arr,unsigned int len);
unsigned char PoleLen2char(float len,unsigned char *data1, unsigned char *data2);
//char SendPoleLen(var_q poleLen);

//解析上位机数据相关
unsigned char AnalysisForceData(unsigned char *data,unsigned char len);
float ForceDouble2Char(float *force,unsigned char data1,unsigned char data2,unsigned char data3);
float CosFloat2Char(float *mCos,unsigned char data1,unsigned char data2);
*/

#endif



