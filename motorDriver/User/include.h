#ifndef __INCLUDE_H
#define __INCLUDE_H

//typedef (unsigned char) uchar;


/*************************** FLASH  Address **************************************************/
#define FLASH_SECTOR_SIZE          (0x2000U)	//sector 大小为128k
#define ADDR_FLASH_SECTOR6    ((uint32_t)0x08040000) /* Base @ of Bank1 sector6, 128 Kbyte */

#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR6   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     (FLASH_USER_START_ADDR + FLASH_SECTOR_SIZE) 
#define 	FLASHSIZE	(24)

//ServoMode 舵机各个模式的定义
#define         CTRL_MODE               (0X10)          //控制模式
#define         FLAT_MODE               (0x11)          //瘪模式，阻尼状态
#define         INFLATE_MODE            (0x12)          //硬模式，可以用于控制角度等等
#define         DIE_MODE                (0x13)          //死模式，空状态
#define         WHEEL_MODE              (0x14)          //轮子模式 

#define 		CURRENT_MODE			(0x15)			//力矩模式



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

#define			StopPwm2					((float)(*(unsigned char*)(uint32_t)(ADDR_FLASH_SECTOR6+(4*10)))/1000)			
#define			InertiaPara					((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*11)))/1000)	

//#define			Sensor_Para					(0.02)
#define			Sensor_Para					((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*12)))/1000)	
#define			Smooth_Para					((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*13)))/1000)
//传感器  PID运算
#define			sensor_P					((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*14)))/1000)	
#define			sensor_I					((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*15)))/1000)		
#define			sensor_D					((float)(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*16)))/1000) 
	
#define			SOFTWARE1					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*17))) 
#define			HARDWARE2					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*18))) 
#define			SOFTWARE2					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*19))) 
#define			HARDWARE3					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*20))) 
#define			SOFTWARE3					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*21)))
	
#define			HARDWARE					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*22))) 
#define			SOFTWARE					(*(unsigned int *)(uint32_t)(ADDR_FLASH_SECTOR6+(4*23)))

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


#endif

