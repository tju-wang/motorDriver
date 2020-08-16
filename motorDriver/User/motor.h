#ifndef __MOTER_H
#define __MOTER_H

#include "include.h"

#define FULLPWM			1000
#define CLOCKWISE		0x21
#define ANTICLOCKWISE	0x22
#define	MOTORSTOP		0x20
#define	REDUCTION		62
#define PI				(3.1415926535)


char setPWMAitClockwise(unsigned int pwm);		//һ·PWM   һ·�ߵ�ƽ
char setPWMClockwise(unsigned int pwm);		//һ·PWM   һ·�ߵ�ƽ
char setFullPWM(void);
char setZeroPWM(void);

char CtrlMotor(unsigned char mode);
char SetMotorPWM(char dir,float pwm,float lastPWM,float *last);

char CalcSpeedPID(float *ctrl,float speedTar,float speedCur);
char CalcSpeedTar(float *speedTar,unsigned char *dir);

//��λת������
float RPreMin2CounterPerSec(float sp);
float CounterPerSec2RPreMin(float sp);

unsigned char PlanTraj(signed long long int posTar,float speedMax,char ch);
unsigned char PosPID(long long int Curpos,long long int PosTar);
#define vMax  (70.0)
#define	a1	(60000)
#define	a2	(10.0)
#define PlanTime	(10)	//��λ ms  ÿ�θ����ٶ�Ŀ���ʱ��

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
	float acce;		//���ٶ�  ���ٶ�
	float CtrlPWM;
	float CtrlLastPWM;
	unsigned char ctrlDir;
	
	signed long long int PosTarget;	//����������ֵ
	int ctrlStep;		//���Ƶ���step
	int stepCounter;	//�ڿ��ƹ�����ʣ���step
	int reduceStep;		//��ʼ���ٵ�step
	int acceStep;		//�������step
	int ctrlProcess;	//���ƹ���  0 ֹͣ  1����  2����  3����
	float speedAcce;	//ֻ���յ�����ʱ����һ��
	float speedReduce;	//���ٽ׶εļ����ٶ�
	int stepArr[3];		//�����׶ε�ʱ��
	long long int distance[3];	//�ֱ��Ǽ��ٶ�  ���ٶ� ���ٶε�λ��

	//
}MotorCtrl_t;
//ö��



#endif
