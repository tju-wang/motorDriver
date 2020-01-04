#include "motor.h"
#include "math.h"
extern Motor_t M1;
MotorCtrl_t M1Ctrl;


char CtrlMotor(unsigned char mode)
{
	static unsigned long int CtrlCounter = 0;
	CtrlCounter++;
	switch(mode)
	{
		case CTRL_MODE:
		{
			if(CtrlCounter%10==0)	{
				CalcSpeedTar(&M1Ctrl.speedTar,M1.speed);
			}
			CalcSpeedPID(&M1Ctrl.CtrlPWM,M1Ctrl.speedTar,M1.speed);
			SetMotorPWM(CLOCKWISE,M1Ctrl.CtrlPWM,M1Ctrl.CtrlLastPWM,&(M1Ctrl.CtrlLastPWM));
		}break;
	}
	
}

//计算期望速度  规划部分
/*
判断当前处于的梯形速度曲线区间  1.加速阶段  2.匀速阶段  3.减速阶段  0.停止

根据加速阶段  计算当前的目标位置  得出转动方向  目标速度
不断进行重新规划  规划周期大于执行周期
假定规划周期   10ms   执行周期1ms

构造控制结构体

输入：目标位置  执行总时间  当前已经执行的时间  最大加、减速度  最大速度  当前速度
得出：本次规划目标速度  转动方向  
*/
char CalcSpeedTar(float *speedTar,float speed)
{
//	*speedTar = 50.5;	//先给定值
	static int sin_t = 0;
	sin_t++;
	*speedTar = 35*(sin(0.005*PI*sin_t)+1);
	return HAL_OK;
}

//跟随期望速度  增量式PI控制器   返回的是pwm增量
char CalcSpeedPID(int *ctrl,float speedTar,float speedCur)
{
	static unsigned char ii; 
	static float Error[16];   //误差 期望位置-当前位置
	static float VelocityError;
	ii++;
	Error[(ii&15)]=(speedTar - speedCur);  //必须有数据类型转换！  unsigned 转换为 signed  最高位直接被当做符号位  signed 转换为unsigned 最高位直接被当做数据位
																//两个unsigned做差 差为负数时会导致错误
	VelocityError=Error[(ii&15)] - Error[(ii-1&15)];
	
	*ctrl=(int)(Ctrl_P*VelocityError + Ctrl_I*(Error[(ii&15)]));
	
	//if(Error[(ii&15)]<=0.1&&Error[(ii&15)]>=-0.1) *ctrl = 0;  //电流ADC<+-10时 不给控制量  对应电流为+-0.03A
	return HAL_OK;
}	

//对PID输出进行分析  调用电机基础控制函数   由期望转动方向（由位置差给出）  与  pid计算完之后的pwm   决定调用什么函数
char SetMotorPWM(char dir,int pwm,int lastPWM,int *last)
{
	int Ctrl = 0;
	Ctrl = lastPWM + pwm;
	if(Ctrl>1000)
	{
		Ctrl = 1000;
	}
	else if(Ctrl<0)
	{
		Ctrl = 0;
	}
	*last = Ctrl;
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
	if(pwm < PWM_MAX*10)
	{
		TIM1->CCR2 = 0;
		TIM1->CCR1 = pwm;
	}
	else
	{
		TIM1->CCR2 = 0;
		TIM1->CCR1 = PWM_MAX*10;
		return 0;
	}
	return 1;
}

char setPWMClockwise(unsigned int pwm)		//一路PWM   一路高电平
{
	if(pwm < PWM_MAX*10)
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = pwm;
	}
	else
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = PWM_MAX*10;
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
