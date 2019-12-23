#include "motor.h"






char setPWMAitClockwise(unsigned int pwm)		//一路PWM   一路高电平
{
	if(pwm <= FULLPWM)
	{
		TIM1->CCR2 = FULLPWM;
		TIM1->CCR1 = pwm;
		return 0;
	}
	return 1;

}

char setPWMClockwise(unsigned int pwm)		//一路PWM   一路高电平
{
	if(pwm <= FULLPWM)
	{
		TIM1->CCR1 = FULLPWM;
		TIM1->CCR2 = pwm;
		return 0;
	}
	return 1;
}

char setFullPWM()		//制动模式  阻尼
{
	TIM1->CCR1 = FULLPWM;
	TIM1->CCR2 = FULLPWM;
	return 0;
}

char setZeroPWM()	//滑行模式  掉电
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	return 0;
}
