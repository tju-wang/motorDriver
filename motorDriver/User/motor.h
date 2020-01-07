#ifndef __MOTER_H
#define __MOTER_H

#include "include.h"

#define FULLPWM			1000
#define CLOCKWISE		0x21
#define ANTICLOCKWISE	0x22
#define	MOTORSTOP		0x20
#define	REDUCTION		62
#define PI				(3.1415926535)


char setPWMAitClockwise(unsigned int pwm);		//一路PWM   一路高电平
char setPWMClockwise(unsigned int pwm);		//一路PWM   一路高电平
char setFullPWM(void);
char setZeroPWM(void);

char CtrlMotor(unsigned char mode);
char CalcSpeedPID(int *ctrl,float speedTar,float speedCur);
char SetMotorPWM(char dir,int pwm,int lastPWM,int *last);

char CalcSpeedTar(float *speedTar,unsigned char *dir);

#define vMax  (70.0)
#define	a1	(10.0)
#define	a2	(10.0)

enum Process
{
	ProStop = 0,
	ProAcce,
	ProUniform,
	ProReduce
};

typedef struct MotorCtrl{
	
	float speedTar;
	float speedMax;
	int CtrlPWM;
	int CtrlLastPWM;
	unsigned char ctrlDir;
	
	signed long int PosTarget;	//编码器计数值
	int ctrlStep;		//控制的总step
	int stepCounter;	//在控制过程中剩余的step
	int reduceStep;		//开始减速的step
	int acceStep;		//加速完成step
	int ctrlProcess;	//控制过程  0 停止  1加速  2匀速  3减速
	float speedAcce;	//只在收到数据时计算一次
	float speedReduce;	//减速阶段的减速速度

	//
}MotorCtrl_t;
//枚举



#endif
