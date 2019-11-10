#ifndef __OFFLINE_CHECK_H
#define __OFFLINE_CHECK_H

/* 包含头文件----------------------------------------------------------------*/

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

/* 本模块向外部提供的宏定义--------------------------------------------------*/	
#define LED_PERIOD  600
#define Check_PERIOD  100


/* 本模块向外部提供的接口函数原型声明----------------------------------------*/		
void Led_Task(void const * argument);
void Check_Task(void const * argument);
























#endif
