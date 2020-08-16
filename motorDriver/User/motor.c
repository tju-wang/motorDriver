#include "motor.h"
#include "math.h"
#include "stdlib.h"
extern Motor_t M1;
MotorCtrl_t M1Ctrl;
enum Process M1Process;
extern long long int gPosTar;
char tempChar[10] = {0};
long long tempLL[10] = {0};
char CtrlMotor(unsigned char mode)	//1ms����һ��
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
			SetMotorPWM(M1Ctrl.ctrlDir,M1Ctrl.CtrlPWM,M1Ctrl.CtrlLastPWM,&(M1Ctrl.CtrlLastPWM));	//ֱ�Ӹ����˷���  Ӧ�ñȽϺ�ѡ��
		}break;
		case POSCTRL_MODE:
		{
//			PosPID();
			CalcSpeedTar(&M1Ctrl.speedTar,&M1Ctrl.ctrlDir);
		}break;
		case WHEEL_MODE:	//com.c��  ���ܵ�����ģʽָ�ֱ������pwm
		{
			
		}break;
		case DIE_MODE:
		{
			setPWMClockwise(0);
		}break;
		case SPEED_MODE:
		{
			CalcSpeedPID(&M1Ctrl.CtrlPWM,M1Ctrl.speedTar,M1.speed);
			SetMotorPWM(M1Ctrl.ctrlDir,M1Ctrl.CtrlPWM,M1Ctrl.CtrlLastPWM,&(M1Ctrl.CtrlLastPWM));	//ֱ�Ӹ����˷���  Ӧ�ñȽϺ�ѡ��
		}break;
		
	}
	return HAL_OK;
}

//���������ٶ�  �滮����
/*
�жϵ�ǰ���ڵ������ٶ���������  1.���ٽ׶�  2.���ٽ׶�  3.���ٽ׶�  0.ֹͣ

���ݼ��ٽ׶�  ���㵱ǰ��Ŀ��λ��  �ó�ת������  Ŀ���ٶ�
���Ͻ������¹滮  �滮���ڴ���ִ������
�ٶ��滮����   10ms   ִ������1ms

������ƽṹ��

���룺Ŀ��λ��  ִ����ʱ��  ��ǰ�Ѿ�ִ�е�ʱ��  ���ӡ����ٶ�  ����ٶ�  ��ǰ�ٶ�
�ó������ι滮Ŀ���ٶ�  ת������  
*/
char CalcSpeedTar(float *speedOut,unsigned char *dirOut)
{
	float speedTar;
	//long long int err = 0;
	//���ж�ģʽ  ���ݵ�ǰλ�������λ�õ�λ�ò�
/*
	err = M1Ctrl.PosTarget - M1.EnCounter;
	if(err<0)	err = -err;
	if(err<127000)	//�ı�ģʽ
	{
		M1.motorMode = POSCTRL_MODE;
	}
*/
//	static int sin_t = 0;
//	sin_t++;
//	*speedOut = 35*(sin(0.008*PI*sin_t)+1);
//	*dirOut = CLOCKWISE;
	//�Ƚϵ�ǰλ����Ŀ��λ�ô�С  �ó�dir
	

	if(M1Ctrl.PosTarget > M1.EnCounter)	{
		*dirOut = CLOCKWISE;
	}
	else if(M1Ctrl.PosTarget < M1.EnCounter)	{
		*dirOut = ANTICLOCKWISE;
	}
	else	{
		*dirOut = MOTORSTOP;
	}
	//���ݵ�ǰ�����׶Σ�step�жϣ�  ��⵱ǰĿ���ٶ�  ÿһ�ζ����¹滮 reduceStep
	//�жϵ�ǰ����״̬
	if(M1Ctrl.stepCounter > M1Ctrl.acceStep)	{
		M1Ctrl.ctrlProcess = ProAcce;
	}
	else if(M1Ctrl.stepCounter > M1Ctrl.reduceStep && ((*dirOut==CLOCKWISE && M1.EnCounter<=(M1Ctrl.PosTarget-M1Ctrl.distance[2])) || (*dirOut==ANTICLOCKWISE && M1.EnCounter>=(M1Ctrl.PosTarget+M1Ctrl.distance[2]))))	{
		M1Ctrl.ctrlProcess = ProUniform;
	}
	else {	//���ٽ׶�  �������ͣ����ӽ�Ŀ��Ƕ�
		M1Ctrl.ctrlProcess = ProReduce;
	}
	//����״̬  �滮Ŀ��Ƕ�
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
		case ProUniform:	//���ٽ׶�  ����˵Ҫ���¹滮һ��  ʵʱ�ı�����ٶȺ� reduceStep
		{
			speedTar = M1Ctrl.speedMax;		
		}break;
		case ProReduce:		//���ٵ��ٶ�Ϊ0
		{
			speedTar = M1Ctrl.speedTar - M1Ctrl.speedReduce; 
			if(speedTar<0)
				speedTar = fabs(speedTar);
//			if(llabs((M1Ctrl.PosTarget-M1.EnCounter)) < POSERR)	//����
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

//�滮  �趨Ŀ��λ�á�����ٶȣ����ٶ�����ٶ�  ֻ����ָ������ʱ��һ�ι滮  �ڼ��ٽ׶����¹滮  ����λ�ÿ���ģʽ
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
	long long posErr =  llabs((M1Ctrl.PosTarget-M1.EnCounter));	//���ι滮��λ�ò�  �п����Ǹ���
	tempLL[0] = posErr;
	
	if(ch==1)
	{
		//�������ʱ��  t = v/a
		M1Ctrl.stepArr[2] = (spMax / M1Ctrl.acce) * 1000;	//����ʱ��  ��λms
		M1Ctrl.stepArr[0] = (spMax / M1Ctrl.acce) * 1000;
		
		M1Ctrl.distance[2] = ((double)M1Ctrl.stepArr[2]/1000.0f) * spMax * 0.5;	//���������ε����
		M1Ctrl.distance[0] = ((double)M1Ctrl.stepArr[0]/1000.0f) * spMax * 0.5;	//�������������
		M1Ctrl.distance[1] = posErr - (M1Ctrl.distance[0]+M1Ctrl.distance[2]);
		tempLL[1] = M1Ctrl.distance[0];
		tempLL[2] = M1Ctrl.distance[1];
		tempLL[3] = M1Ctrl.distance[2];
		if(M1Ctrl.distance[1]<0)	{	//��ʱ  û�����ٶ�  ֻ�мӼ��ٶ�
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
		
		//������ٽ׶ε�����ʱ��
		M1Ctrl.stepArr[1] = (M1Ctrl.distance[1] / spMax)*1000;		//���ٽ׶�  ��λms
		
		//�滮�л�ʱ��
		M1Ctrl.ctrlStep = M1Ctrl.stepArr[0] + M1Ctrl.stepArr[1] + M1Ctrl.stepArr[2];
		
		M1Ctrl.stepCounter = M1Ctrl.ctrlStep;
		M1Ctrl.reduceStep = M1Ctrl.stepArr[2];
		M1Ctrl.acceStep = M1Ctrl.ctrlStep - M1Ctrl.stepArr[0]; 
		
		M1Ctrl.speedAcce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
		M1Ctrl.speedReduce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
		
		//ȷ��ת������
		if(M1Ctrl.PosTarget > M1.EnCounter)	{
			M1Ctrl.ctrlDir = CLOCKWISE;
		}
		else if(M1Ctrl.PosTarget < M1.EnCounter)	{
			M1Ctrl.ctrlDir = ANTICLOCKWISE;
		}
		else	{
			M1Ctrl.ctrlDir = MOTORSTOP;
		}
		//�滮����  �л�������ģʽ
		M1.motorMode = CTRL_MODE;	
		M1Ctrl.ctrlProcess = 0;
	}
	else if(ch==2)
	{
		switch(M1Ctrl.ctrlProcess)
		{
			case ProAcce:	//���ٽ׶�
			{
				//M1Ctrl.stepArr[2] = (spMax / M1Ctrl.acce) * 1000;	//����ʱ��  ��λms
				M1Ctrl.stepArr[0] = ((spMax- RPreMin2CounterPerSec(M1.speed))/ M1Ctrl.acce) * 1000;	//ʣ����������ʱ��
				//M1Ctrl.distance[2] = (M1Ctrl.stepArr[2]/1000.0f) * spMax * 0.5;	//���������ε����
				M1Ctrl.distance[0] = ((double)M1Ctrl.stepArr[0]/1000.0f) * (spMax+RPreMin2CounterPerSec(M1.speed)) * 0.5;//ʣ��ļ����������
				M1Ctrl.distance[1] = posErr - (M1Ctrl.distance[0]+M1Ctrl.distance[2]);
				
				if(M1Ctrl.distance[1]<0)	{	//��ʱ  û�����ٶ�  ֻ�мӼ��ٶ�
					M1Ctrl.distance[1] = 0;
					M1Ctrl.distance[0] = (posErr-M1Ctrl.distance[2]);
					M1Ctrl.stepArr[0] = (M1Ctrl.distance[0] * 2 / ((spMax+RPreMin2CounterPerSec(M1.speed))))*1000;
				}
				//������ٽ׶ε�����ʱ��
				M1Ctrl.stepArr[1] = (M1Ctrl.distance[1] / spMax)*1000;		//���ٽ׶�  ��λms
				//�滮�л�ʱ��
				M1Ctrl.ctrlStep = M1Ctrl.stepArr[0] + M1Ctrl.stepArr[1] + M1Ctrl.stepArr[2];	
				//M1Ctrl.stepCounter = M1Ctrl.ctrlStep;
				M1Ctrl.acceStep = M1Ctrl.ctrlStep - M1Ctrl.stepArr[0]; 
				M1Ctrl.reduceStep = M1Ctrl.stepArr[2];
				M1Ctrl.speedAcce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
				M1Ctrl.speedReduce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
			}break;
			case ProUniform:
			{
				//M1Ctrl.stepArr[2] = (spMax / M1Ctrl.acce) * 1000;	//����ʱ��  ��λms
				M1Ctrl.stepArr[0] = 0;
				//M1Ctrl.distance[2] = (M1Ctrl.stepArr[2]/1000.0f) * spMax * 0.5;	//���������ε����
				M1Ctrl.distance[0] = 0;//�������������
				M1Ctrl.distance[1] = posErr - (M1Ctrl.distance[0]+M1Ctrl.distance[2]);
				
				if(M1Ctrl.distance[1]<0)	{	//��ʱ  Ӧ�ý�����ٽ׶�
					M1Ctrl.distance[1] = 0;
					M1Ctrl.distance[2] = posErr;
					if(M1Ctrl.distance[2]>0)
						M1Ctrl.stepArr[2] = sqrt(2*M1Ctrl.distance[2]/M1Ctrl.acce)*1000;
					//M1Ctrl.ctrlProcess = ProReduce;	//ֱ���е����ٽ׶�
				}
				//������ٽ׶ε�����ʱ��
				M1Ctrl.stepArr[1] = (M1Ctrl.distance[1] / spMax)*1000;		//���ٽ׶�  ��λms
				//�滮�л�ʱ��
				M1Ctrl.ctrlStep = M1Ctrl.stepArr[1] + M1Ctrl.stepArr[2];
				M1Ctrl.reduceStep = M1Ctrl.stepArr[2];
				//M1Ctrl.speedAcce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
				M1Ctrl.speedReduce = CounterPerSec2RPreMin((PlanTime)*0.001 * M1Ctrl.acce);
				
			}break;
			case ProReduce:
			{	
				M1Ctrl.distance[2] = posErr;	//���������ε����	
				if(M1Ctrl.distance[2]>0)	{
					M1Ctrl.stepArr[2] = (2*M1Ctrl.distance[2] / RPreMin2CounterPerSec(M1.speed))*1000;	//����ʱ��  ��λms
					M1Ctrl.speedReduce = ((PlanTime)*0.001 *(M1.speed)/(M1Ctrl.stepArr[2]/1000.0f));	//��
				}
				else	{ 
					M1Ctrl.stepArr[2] = 1;	//����Ϊ0
					M1Ctrl.speedReduce = (double)CounterPerSec2RPreMin(M1.speed/0.01);
				}
				M1Ctrl.stepArr[0] = 0;
				M1Ctrl.distance[0] = 0;//�������������
				M1Ctrl.distance[1] = 0;	//���ٽ׶�λ��
				
				//������ٽ׶ε�����ʱ��
				M1Ctrl.stepArr[1] = 0;		//���ٽ׶�  ��λms
				//�滮�л�ʱ��				
				
			}break;
			
		}
		M1Ctrl.stepCounter = M1Ctrl.ctrlStep;
		//M1.motorMode = CTRL_MODE;	
	}

	return HAL_OK;
}

// �ٶȵ�λ  r/min -> counter/s
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

//���������ٶ�  ����ʽPI������   ���ص���pwm����
char CalcSpeedPID(float *ctrl,float speedTar,float speedCur)
{
	static unsigned char ii; 
	static float Error[16];   //��� ����λ��-��ǰλ��
	static float VelocityError;
	static float Error_I = 0;
	static float max_I = 50;
	ii++;
	Error[(ii&15)]=(speedTar - speedCur);  //��������������ת����  unsigned ת��Ϊ signed  ���λֱ�ӱ���������λ  signed ת��Ϊunsigned ���λֱ�ӱ���������λ
																//����unsigned���� ��Ϊ����ʱ�ᵼ�´���
	VelocityError=Error[(ii&15)] - Error[(ii-1&15)];
	Error_I += Error[(ii&15)];
	if(Error_I > max_I)
		Error_I = max_I;
	else if(Error_I < -max_I)
		Error_I = -max_I;
		
//	*ctrl=(float)(Ctrl_P*VelocityError + Ctrl_I*(Error[(ii&15)]));
	*ctrl=(float)(Ctrl_P*0.01*(Error[(ii&15)]) + Ctrl_D*0.01*VelocityError + Ctrl_I*0.01*Error_I);
	//if(Error[(ii&15)]<=0.1&&Error[(ii&15)]>=-0.1) *ctrl = 0;  //����ADC<+-10ʱ ����������  ��Ӧ����Ϊ+-0.03A

	return HAL_OK;
}	

//��PID������з���  ���õ���������ƺ���   ������ת��������λ�ò������  ��  pid������֮���pwm   ��������ʲô����
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
	//��posErr
	if(M1.motorMode==CTRL_MODE)
	{
		if(llabs((M1Ctrl.PosTarget-M1.EnCounter)) < POSERR)	//����
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
/**************************************�����������****************************************/
char setPWMAitClockwise(unsigned int pwm)		//һ·PWM   һ·�ߵ�ƽ
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

char setPWMClockwise(unsigned int pwm)		//һ·PWM   һ·�ߵ�ƽ
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

char setFullPWM()		//�ƶ�ģʽ  ����
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	return 0;
}

char setZeroPWM()	//����ģʽ  ����
{
	TIM1->CCR1 = FULLPWM;
	TIM1->CCR2 = FULLPWM;
	return 0;
}
