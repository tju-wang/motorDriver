#include "motor.h"
#include "math.h"
#include "stdlib.h"
extern Motor_t M1;
MotorCtrl_t M1Ctrl;
enum Process M1Process;
extern long long int gPosTar;
char tempChar[10] = {0};
long long tempLL[10] = {0};
char CtrlMotor(unsigned char mode)	//1ms调用一次
{
	static unsigned long int CtrlCounter = 0;
	CtrlCounter++;
	
	switch(mode)
	{
		case CTRL_MODE:
		{
			if(M1Ctrl.stepCounter>0)
				M1Ctrl.stepCounter--;
			if(CtrlCounter%PlanTime==0)	{
				PlanTraj(gPosTar,70,2);
				CalcSpeedTar(&M1Ctrl.speedTar,&M1Ctrl.ctrlDir);
			}
			CalcSpeedPID(&M1Ctrl.CtrlPWM,M1Ctrl.speedTar,M1.speed);
			SetMotorPWM(M1Ctrl.ctrlDir,M1Ctrl.CtrlPWM,M1Ctrl.CtrlLastPWM,&(M1Ctrl.CtrlLastPWM));	//直接给定了方向  应该比较后选择
		}break;
		case POSCTRL_MODE:
		{
//			PosPID();
			CalcSpeedTar(&M1Ctrl.speedTar,&M1Ctrl.ctrlDir);
		}break;
		case WHEEL_MODE:	//com.c中  接受到轮子模式指令，直接设置pwm
		{
			
		}break;
		case DIE_MODE:
		{
			setPWMClockwise(0);
		}break;
		case SPEED_MODE:
		{
			CalcSpeedPID(&M1Ctrl.CtrlPWM,M1Ctrl.speedTar,M1.speed);
			SetMotorPWM(M1Ctrl.ctrlDir,M1Ctrl.CtrlPWM,M1Ctrl.CtrlLastPWM,&(M1Ctrl.CtrlLastPWM));	//直接给定了方向  应该比较后选择
		}break;
		
	}
	return HAL_OK;
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
char CalcSpeedTar(float *speedOut,unsigned char *dirOut)
{
	float speedTar;
	//long long int err = 0;
	//先判断模式  依据当前位置与最后位置的位置差
/*
	err = M1Ctrl.PosTarget - M1.EnCounter;
	if(err<0)	err = -err;
	if(err<127000)	//改变模式
	{
		M1.motorMode = POSCTRL_MODE;
	}
*/
//	static int sin_t = 0;
//	sin_t++;
//	*speedOut = 35*(sin(0.008*PI*sin_t)+1);
//	*dirOut = CLOCKWISE;
	//比较当前位置与目标位置大小  得出dir
	

	if(M1Ctrl.PosTarget > M1.EnCounter)	{
		*dirOut = CLOCKWISE;
	}
	else if(M1Ctrl.PosTarget < M1.EnCounter)	{
		*dirOut = ANTICLOCKWISE;
	}
	else	{
		*dirOut = MOTORSTOP;
	}
	//根据当前所处阶段（step判断）  求解当前目标速度  每一次都重新规划 reduceStep
	//判断当前所处状态
	if(M1Ctrl.stepCounter > M1Ctrl.acceStep)	{
		M1Ctrl.ctrlProcess = ProAcce;
	}
	else if(M1Ctrl.stepCounter > M1Ctrl.reduceStep && ((*dirOut==CLOCKWISE && M1.EnCounter<=(M1Ctrl.PosTarget-M1Ctrl.distance[2])) || (*dirOut==ANTICLOCKWISE && M1.EnCounter>=(M1Ctrl.PosTarget+M1Ctrl.distance[2]))))	{
		M1Ctrl.ctrlProcess = ProUniform;
	}
	else {	//减速阶段  负责减速停车与接近目标角度
		M1Ctrl.ctrlProcess = ProReduce;
	}
	//根据状态  规划目标角度
	switch(M1Ctrl.ctrlProcess)
	{
		case  ProStop:
		{
			speedTar = 0;
		}break;
		case ProAcce:
		{
			speedTar = M1Ctrl.speedTar + M1Ctrl.speedAcce;
			if(speedTar > M1Ctrl.speedMax)
				speedTar = M1Ctrl.speedMax;
		}break;
		case ProUniform:	//匀速阶段  按理说要重新规划一次  实时改变最快速度和 reduceStep
		{
			speedTar = M1Ctrl.speedMax;		
		}break;
		case ProReduce:		//减速到速度为0
		{
			speedTar = M1Ctrl.speedTar - M1Ctrl.speedReduce; 
			if(speedTar<0)
				speedTar = fabs(speedTar);
//			if(llabs((M1Ctrl.PosTarget-M1.EnCounter)) < POSERR)	//防抖
//			{
//				speedTar = 0;
//			}	
		}break;
		default:
		{}break;	
	}
	
	*speedOut = speedTar;
	return HAL_OK;
}

//规划  设定目标位置、最大速度，加速度与减速度  只有在指令输入时做一次规划  在减速阶段重新规划  进入位置控制模式
unsigned char PlanTraj(signed long long int posTar,float speedMax,char ch)
{
	static float spMax=0;
	//float vPlan,tAll;
	tempChar[0] = ch;
	if(ch==1)
	{
		M1Ctrl.PosTarget = posTar;
		M1Ctrl.speedMax = speedMax;			
		spMax = RPreMin2CounterPerSec(speedMax);
		M1Ctrl.acce = ACCLERATE;
		M1.motorMode = CTRL_MODE;
	}
	long long posErr =  llabs((M1Ctrl.PosTarget-M1.EnCounter));	//本次规划的位置差  有可能是负数
	tempLL[0] = posErr;
	
	if(ch==1)
	{
		//求出减速时间  t = v/a
		M1Ctrl.stepArr[2] = (spMax / M1Ctrl.acce) * 1000;	//减速时间  单位ms
		M1Ctrl.stepArr[0] = (spMax / M1Ctrl.acce) * 1000;
		
		M1Ctrl.distance[2] = ((double)M1Ctrl.stepArr[2]/1000.0f) * spMax * 0.5;	//减速三角形的面积
		M1Ctrl.distance[0] = ((double)M1Ctrl.stepArr[0]/1000.0f) * spMax * 0.5;	//加速三角形面积
		M1Ctrl.distance[1] = posErr - (M1Ctrl.distance[0]+M1Ctrl.distance[2]);
		tempLL[1] = M1Ctrl.distance[0];
		tempLL[2] = M1Ctrl.distance[1];
		tempLL[3] = M1Ctrl.distance[2];
		if(M1Ctrl.distance[1]<0)	{	//此时  没有匀速段  只有加减速段
			M1Ctrl.distance[1] = 0;
			M1Ctrl.distance[2] = posErr/2.0f;
			M1Ctrl.distance[0] = M1Ctrl.distance[2];
			if(M1Ctrl.distance[2]>0)
			{
				M1Ctrl.stepArr[2] = sqrt(2*M1Ctrl.distance[2]/M1Ctrl.acce)*1000;	//s = 1/2 * a * t^2
				M1Ctrl.stepArr[0] = sqrt(2*M1Ctrl.distance[0]/M1Ctrl.acce)*1000;
			}
			else
			{
				M1Ctrl.stepArr[2] = sqrt(-2*M1Ctrl.distance[2]/M1Ctrl.acce)*1000;
				M1Ctrl.stepArr[0] = sqrt(-2*M1Ctrl.distance[0]/M1Ctrl.acce)*1000;
			}			
		}
		
		//求出匀速阶段的运行时间
		M1Ctrl.stepArr[1] = (M1Ctrl.distance[1] / spMax)*1000;		//匀速阶段  单位ms
		
		//规划切换时机
		M1Ctrl.ctrlStep = M1Ctrl.stepArr[0] + M1Ctrl.stepArr[1] + M1Ctrl.stepArr[2];
		
		M1Ctrl.stepCounter = M1Ctrl.ctrlStep;
		M1Ctrl.reduceStep = M1Ctrl.stepArr[2];
		M1Ctrl.acceStep = M1Ctrl.ctrlStep - M1Ctrl.stepArr[0]; 
		
		M1Ctrl.speedAcce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
		M1Ctrl.speedReduce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
		
		//确定转动方向
		if(M1Ctrl.PosTarget > M1.EnCounter)	{
			M1Ctrl.ctrlDir = CLOCKWISE;
		}
		else if(M1Ctrl.PosTarget < M1.EnCounter)	{
			M1Ctrl.ctrlDir = ANTICLOCKWISE;
		}
		else	{
			M1Ctrl.ctrlDir = MOTORSTOP;
		}
		//规划结束  切换到控制模式
		M1.motorMode = CTRL_MODE;	
		M1Ctrl.ctrlProcess = 0;
	}
	else if(ch==2)
	{
		switch(M1Ctrl.ctrlProcess)
		{
			case ProAcce:	//加速阶段
			{
				//M1Ctrl.stepArr[2] = (spMax / M1Ctrl.acce) * 1000;	//减速时间  单位ms
				M1Ctrl.stepArr[0] = ((spMax- RPreMin2CounterPerSec(M1.speed))/ M1Ctrl.acce) * 1000;	//剩余加速所需的时间
				//M1Ctrl.distance[2] = (M1Ctrl.stepArr[2]/1000.0f) * spMax * 0.5;	//减速三角形的面积
				M1Ctrl.distance[0] = ((double)M1Ctrl.stepArr[0]/1000.0f) * (spMax+RPreMin2CounterPerSec(M1.speed)) * 0.5;//剩余的加速梯形面积
				M1Ctrl.distance[1] = posErr - (M1Ctrl.distance[0]+M1Ctrl.distance[2]);
				
				if(M1Ctrl.distance[1]<0)	{	//此时  没有匀速段  只有加减速段
					M1Ctrl.distance[1] = 0;
					M1Ctrl.distance[0] = (posErr-M1Ctrl.distance[2]);
					M1Ctrl.stepArr[0] = (M1Ctrl.distance[0] * 2 / ((spMax+RPreMin2CounterPerSec(M1.speed))))*1000;
				}
				//求出匀速阶段的运行时间
				M1Ctrl.stepArr[1] = (M1Ctrl.distance[1] / spMax)*1000;		//匀速阶段  单位ms
				//规划切换时机
				M1Ctrl.ctrlStep = M1Ctrl.stepArr[0] + M1Ctrl.stepArr[1] + M1Ctrl.stepArr[2];	
				//M1Ctrl.stepCounter = M1Ctrl.ctrlStep;
				M1Ctrl.acceStep = M1Ctrl.ctrlStep - M1Ctrl.stepArr[0]; 
				M1Ctrl.reduceStep = M1Ctrl.stepArr[2];
				M1Ctrl.speedAcce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
				M1Ctrl.speedReduce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
			}break;
			case ProUniform:
			{
				//M1Ctrl.stepArr[2] = (spMax / M1Ctrl.acce) * 1000;	//减速时间  单位ms
				M1Ctrl.stepArr[0] = 0;
				//M1Ctrl.distance[2] = (M1Ctrl.stepArr[2]/1000.0f) * spMax * 0.5;	//减速三角形的面积
				M1Ctrl.distance[0] = 0;//加速三角形面积
				M1Ctrl.distance[1] = posErr - (M1Ctrl.distance[0]+M1Ctrl.distance[2]);
				
				if(M1Ctrl.distance[1]<0)	{	//此时  应该进入减速阶段
					M1Ctrl.distance[1] = 0;
					M1Ctrl.distance[2] = posErr;
					if(M1Ctrl.distance[2]>0)
						M1Ctrl.stepArr[2] = sqrt(2*M1Ctrl.distance[2]/M1Ctrl.acce)*1000;
					//M1Ctrl.ctrlProcess = ProReduce;	//直接切到减速阶段
				}
				//求出匀速阶段的运行时间
				M1Ctrl.stepArr[1] = (M1Ctrl.distance[1] / spMax)*1000;		//匀速阶段  单位ms
				//规划切换时机
				M1Ctrl.ctrlStep = M1Ctrl.stepArr[1] + M1Ctrl.stepArr[2];
				M1Ctrl.reduceStep = M1Ctrl.stepArr[2];
				//M1Ctrl.speedAcce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
				M1Ctrl.speedReduce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
				
			}break;
			case ProReduce:
			{	
				M1Ctrl.distance[2] = posErr;	//减速三角形的面积	
				if(M1Ctrl.distance[2]>0)	{
					M1Ctrl.stepArr[2] = (2*M1Ctrl.distance[2] / RPreMin2CounterPerSec(M1.speed))*1000;	//减速时间  单位ms
					M1Ctrl.speedReduce = ((PlanTime)*0.001 *(M1.speed)/(M1Ctrl.stepArr[2]/1000.0f));	//？
				}
				else	{ 
					M1Ctrl.stepArr[2] = 1;	//不能为0
					M1Ctrl.speedReduce = (double)CounterPerSec2RPreMin(M1.speed/0.01);
				}
				M1Ctrl.stepArr[0] = 0;
				M1Ctrl.distance[0] = 0;//加速三角形面积
				M1Ctrl.distance[1] = 0;	//匀速阶段位移
				
				//求出匀速阶段的运行时间
				M1Ctrl.stepArr[1] = 0;		//匀速阶段  单位ms
				//规划切换时机				
				
			}break;
			
		}
		M1Ctrl.stepCounter = M1Ctrl.ctrlStep;
		//M1.motorMode = CTRL_MODE;	
	}

	return HAL_OK;
}

// 速度单位  r/min -> counter/s
float RPreMin2CounterPerSec(float sp)
{
	return (float)(sp*1024*4*REDUCTION/60.0f);
}
float CounterPerSec2RPreMin(float sp)
{
	return (float)(sp*60/(4*REDUCTION*1024.0f));
}
	
unsigned char PosPID(long long int Curpos,long long int PosTar)
{
	return HAL_OK;
}

//跟随期望速度  增量式PI控制器   返回的是pwm增量
char CalcSpeedPID(float *ctrl,float speedTar,float speedCur)
{
	static unsigned char ii; 
	static float Error[16];   //误差 期望位置-当前位置
	static float VelocityError;
	static float Error_I = 0;
	static float max_I = 50;
	ii++;
	Error[(ii&15)]=(speedTar - speedCur);  //必须有数据类型转换！  unsigned 转换为 signed  最高位直接被当做符号位  signed 转换为unsigned 最高位直接被当做数据位
																//两个unsigned做差 差为负数时会导致错误
	VelocityError=Error[(ii&15)] - Error[(ii-1&15)];
	Error_I += Error[(ii&15)];
	if(Error_I > max_I)
		Error_I = max_I;
	else if(Error_I < -max_I)
		Error_I = -max_I;
		
//	*ctrl=(float)(Ctrl_P*VelocityError + Ctrl_I*(Error[(ii&15)]));
	*ctrl=(float)(Ctrl_P*0.01*(Error[(ii&15)]) + Ctrl_D*0.01*VelocityError + Ctrl_I*0.01*Error_I);
	//if(Error[(ii&15)]<=0.1&&Error[(ii&15)]>=-0.1) *ctrl = 0;  //电流ADC<+-10时 不给控制量  对应电流为+-0.03A

	return HAL_OK;
}	

//对PID输出进行分析  调用电机基础控制函数   由期望转动方向（由位置差给出）  与  pid计算完之后的pwm   决定调用什么函数
char SetMotorPWM(char dir,float pwm,float lastPWM,float *last)
{
	float Ctrl = 0;
	Ctrl = (lastPWM + pwm);
	if(Ctrl>1000)
	{
		Ctrl = 1000;
	}
	else if(Ctrl<0)
	{
		Ctrl = 0;
	}
	//加posErr
	if(M1.motorMode==CTRL_MODE)
	{
		if(llabs((M1Ctrl.PosTarget-M1.EnCounter)) < POSERR)	//防抖
		{
			Ctrl = 0;
		}
	}
			
	*last = Ctrl;
	switch(dir)
	{
		case CLOCKWISE:
		{
			setPWMClockwise((int)Ctrl);
		}break;
		case ANTICLOCKWISE:
		{
			setPWMAitClockwise((int)Ctrl);
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
