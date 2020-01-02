#ifndef __MOTER_H
#define __MOTER_H

#include "include.h"

#define FULLPWM			1000
#define CLOCKWISE		0x21
#define ANTICLOCKWISE	0x22
#define	MOTORSTOP		0x20
#define	REDUCTION		62

char setPWMAitClockwise(unsigned int pwm);		//一路PWM   一路高电平
char setPWMClockwise(unsigned int pwm);		//一路PWM   一路高电平
char setFullPWM(void);
char setZeroPWM(void);

char CalcSpeedTar(float *speedTar,float speed);
char CtrlMotor(unsigned char mode);
char CalcSpeedPID(int *ctrl,float speedTar,float speedCur);
char SetMotorPWM(char dir,int pwm,int lastPWM,int *last);


#endif
