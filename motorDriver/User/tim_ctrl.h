#ifndef __tim_ctrl_H
#define __tim_ctrl_H

#include "include.h"

#define EncoderStoreNum		20		//计算速度值  需要参考的位置值的数量
#define 	ENCODER_NUM		(int)(65535)
#define		ENCODER_NUM_2	((int)(ENCODER_NUM/2.0f))
#define StateRef			(0.5f)	//转动或静止  参考计数值
#define ACCE_NUM			(20)	//加速度常数  加速度更新时间  单位 ms   <10  加速度值浮动范围较大  >50 加速度更新频率过慢
#define	ACCEN_NUM			(100)	//与加速度常数之  积  为 2000

#define DEBUG_DATA_NUM		(2000)
#define DebugFreq			(10)


typedef struct Motor{
	signed char EnOverflowNum;	//编码器溢出的次数
	signed long int EnCounter;	//编码器计数值
	int tEnCounter;
	
	char pArr;	//当前新数据的存储位置
	signed long int EnCoterArr[EncoderStoreNum];	//计算速度用
	signed char runstate;	//1 正转 -1 反转 0 静止  以编码器读值变大的方向为正
	float speed;		//电机速度
	float MoterSpeedArr[EncoderStoreNum];
	char pSpArr;
	float accelration;	//电机加速度
	signed char accDir;		//加速与减速方向   -1  减速   1  加速
	unsigned char motor_id;
	
}Motor_t;

unsigned char MotorState(Motor_t *pMotor);
void EncoderUpdate(Motor_t *pEncoder, TIM_HandleTypeDef htim);	//编码器计数值更新  存入编码器数组当中
unsigned char EncoderFlow(Motor_t* pEncoder, TIM_HandleTypeDef htim);
unsigned char GetID(void);
#endif
