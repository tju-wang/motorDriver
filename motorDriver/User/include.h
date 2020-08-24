#ifndef __INCLUDE_H
#define __INCLUDE_H

//typedef (unsigned char) uchar;


/*************************** FLASH  Address **************************************************/
#define FLASH_SECTOR_SIZE          (0x2000U)	//sector ��СΪ128k
#define ADDR_FLASH_SECTOR6    ((uint32_t)0x08040000) /* Base @ of Bank1 sector6, 128 Kbyte */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR6   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (FLASH_USER_START_ADDR + FLASH_SECTOR_SIZE) 
#define 	FLASHSIZE	(24)

//ServoMode �������ģʽ�Ķ���
#define         CTRL_MODE               (0X10)          //����ģʽ
#define			POSCTRL_MODE			(0x16)			//λ�ÿ���ģʽ
#define         FLAT_MODE               (0x11)          //��ģʽ������״̬
#define         INFLATE_MODE            (0x12)          //Ӳģʽ���������ڿ��ƽǶȵȵ�
#define         DIE_MODE                (0x13)          //��ģʽ����״̬
#define         WHEEL_MODE              (0x14)          //����ģʽ 

#define 		CURRENT_MODE			(0x15)			//����ģʽ
#define			SPEED_MODE				(0x17)			//��ȷ�ٶ�ģʽ


#define         ProductNum					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*0)))		//32bit    	1                        
#define         SoftwareNum            		(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*1)))
#define			MechanicalNum				(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*2))) 
#define			HardwareNum					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*3))) 
	
#define			Ctrl_P						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*4)))		//			5				
#define			Ctrl_I						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*5)))	
#define			Ctrl_D						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*6)))	
#define			DebugFreq					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*7)))	
	
#define			PWM_MAX						((*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*8))))	
#define			PWM_MIN						((*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*9))))	//		10

#define			POSERR						((float)(*(unsigned char*)(uint32_t)(ADDR_FLASH_SECTOR6+(4*10))))			
#define			ACCLERATE					((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*11)))*1000)	//�滮�׶μ��ٶ�	

//#define			Sensor_Para					(0.02)
#define			CANorUSART					((*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*12))))	
#define			Free1						((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*13)))/1000)
//������  PID����
#define			Free2						((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*14)))/1000)	
#define			Free3						((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*15)))/1000)		
#define			Free4						((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*16)))/1000) 
	
#define			Free5						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*17))) 
#define			Free6						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*18))) 
#define			Free7						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*19))) 
#define			Free8						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*20))) 
#define			Free9						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*21)))
	
#define			Free10						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*22))) 
#define			Free11						(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*23)))

#include "stdio.h"
#include "gpio.h"
#include "tim.h"	
#include "tim_ctrl.h"
#include "main.h"
#include "usart.h"
#include "motor.h"
#include "stmflash.h"
#include "adc.h"
#include "protocol.h"
#include "can.h"

#endif

