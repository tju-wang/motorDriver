#ifndef __tim_ctrl_H
#define __tim_ctrl_H

#include "include.h"

#define EncoderStoreNum		20		//计算速度值  需要参考的位置值的数量
#define 	ENCODER_NUM		(int)(65535)
#define		ENCODER_NUM_2	((int)(ENCODER_NUM/2.0f))

typedef struct Motor{
	signed char EnOverflowNum;	//编码器溢出的次数
	signed long int EnCounter;	//编码器计数值
	int tEnCounter;
	
	char pArr;	//当前新数据的存储位置
	signed long int EnCoterArr[EncoderStoreNum];	//计算速度用
	signed char runstate;	//1 正转 -1 反转 0 静止  以编码器读值变大的方向为正
	int speed;		//电机速度
	signed int MoterSpeedArr[EncoderStoreNum];
	char pSpArr;
	int accelration;	//电机加速度
	
}Motor_t;


void EncoderUpdate(Motor_t *pEncoder, TIM_HandleTypeDef htim);	//编码器计数值更新  存入编码器数组当中
unsigned char EncoderFlow(Motor_t* pEncoder, TIM_HandleTypeDef htim);

#endif
