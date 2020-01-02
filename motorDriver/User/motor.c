#include "motor.h"
extern Motor_t M1;



char CtrlMotor(unsigned char mode)
{

	switch(mode)
	{
		case CTRL_MODE:
		{
			CalcSpeedTar(&M1.speedTar,M1.speed);
			CalcSpeedPID(&M1.CtrlPWM,M1.speedTar,M1.speed);
			SetMotorPWM(CLOCKWISE,M1.CtrlPWM,M1.CtrlLastPWM,&(M1.CtrlLastPWM));
		}break;
	}
	
}

//计算期望速度
/*
判断当前处于的梯形速度曲线区间  1.加速阶段  2.匀速阶段  3.减速阶段  0.停止
根据加速阶段  计算当前的目标位置
*/
char CalcSpeedTar(float *speedTar,float speed)
{
//	*speedTar = 50.5;	//先给定值
	
	return HAL_OK;
}

//跟随期望速度  增量式PI控制器   返回的是pwm增量
char CalcSpeedPID(int *ctrl,float speedTar,float speedCur)
{
	static unsigned char ii; 
	static signed int Error[16];   //误差 期望位置-当前位置
	static signed int VelocityError;
	ii++;
	Error[(ii&15)]=(speedTar - speedCur);  //必须有数据类型转换！  unsigned 转换为 signed  最高位直接被当做符号位  signed 转换为unsigned 最高位直接被当做数据位
																//两个unsigned做差 差为负数时会导致错误
	VelocityError=Error[(ii&15)] - Error[(ii-1&15)];
	
	*ctrl=(int)(Ctrl_P*(Error[(ii&15)])+Ctrl_D*VelocityError);
	
	if(Error[(ii&15)]<=0.1&&Error[(ii&15)]>=-0.1) *ctrl = 0;;  //电流ADC<+-10时 不给控制量  对应电流为+-0.03A
	return HAL_OK;
}	

//对PID输出进行分析  调用电机基础控制函数   由期望转动方向（由位置差给出）  与  pid计算完之后的pwm   决定调用什么函数
char SetMotorPWM(char dir,int pwm,int lastPWM,int *last)
{
	int Ctrl = 0;
	Ctrl = lastPWM + pwm;
	*last = Ctrl;
	if(Ctrl<0)	Ctrl = 0;
	switch(dir)
	{
		case CLOCKWISE:
		{
			
			setPWMClockwise(Ctrl);
		}break;
		case ANTICLOCKWISE:
		{
			setPWMAitClockwise(Ctrl);
		}break;
		case MOTORSTOP:
		{
			setFullPWM();
		}break;
		default :
		{
			setFullPWM();
		}break;
	}
	
	return HAL_OK;
}
/**************************************电机基础控制****************************************/
char setPWMAitClockwise(unsigned int pwm)		//一路PWM   一路高电平
{
	if(pwm <= 1000)
	{
		TIM1->CCR2 = 0;
		TIM1->CCR1 = pwm;
	}
	else
	{
		TIM1->CCR2 = 0;
		TIM1->CCR1 = 900;
		return 0;
	}
	return 1;
}

char setPWMClockwise(unsigned int pwm)		//一路PWM   一路高电平
{
	if(pwm <= 1000)
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = pwm;
	}
	else
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 900;
		return 0;
	}
	return 1;
}

char setFullPWM()		//制动模式  阻尼
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	return 0;
}

char setZeroPWM()	//滑行模式  掉电
{
	TIM1->CCR1 = FULLPWM;
	TIM1->CCR2 = FULLPWM;
	return 0;
}
