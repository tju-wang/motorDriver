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
char SetMotorPWM(char dir,float pwm,float lastPWM,float *last);

char CalcSpeedPID(float *ctrl,float speedTar,float speedCur);
char CalcSpeedTar(float *speedTar,unsigned char *dir);

//单位转换函数
float RPreMin2CounterPerSec(float sp);
float CounterPerSec2RPreMin(float sp);

unsigned char PlanTraj(signed long long int posTar,float speedMax,char ch);
unsigned char PosPID(long long int Curpos,long long int PosTar);
#define vMax  (70.0)
#define	a1	(60000)
#define	a2	(10.0)
#define PlanTime	(10)	//单位 ms  每次更新速度目标的时间

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
	float acce;		//加速度  减速度
	float CtrlPWM;
	float CtrlLastPWM;
	unsigned char ctrlDir;
	
	signed long long int PosTarget;	//编码器计数值
	int ctrlStep;		//控制的总step
	int stepCounter;	//在控制过程中剩余的step
	int reduceStep;		//开始减速的step
	int acceStep;		//加速完成step
	int ctrlProcess;	//控制过程  0 停止  1加速  2匀速  3减速
	float speedAcce;	//只在收到数据时计算一次
	float speedReduce;	//减速阶段的减速速度
	int stepArr[3];		//三个阶段的时间
	long long int distance[3];	//分别是加速段  匀速段 减速段的位移

	//
}MotorCtrl_t;
//枚举



#endif
