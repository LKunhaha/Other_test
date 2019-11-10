/*******************************************************************************
*                     版权所有 (C), 2017-,NCUROBOT
************************************************************************************************************************************************************
* 文 件 名   : Motor_USE_TIM.c
* 版 本 号   : 初稿
* 作    者   : NCURM
* 生成日期   : 2018年7月
* 最近修改   :
* 功能描述   : 电机库模块中使用TIM进行控制的电机
* 函数列表   :
*
*							Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo)

*							Friction_Wheel_Motor_Stop(void)
*					
	******************************************************************************
*/
#include "Motor_USE_TIM.h"
#include "pid.h"

uint16_t mc_count[2]={0,0};

void TIM5_PWM_Init(uint32_t speed1,uint32_t speed2)
{
	  TIM5->CCR1=speed1;
	  TIM5->CCR2=speed2;
	  TIM5->CCR3=speed2;
}

/**
	**************************************************************
	** Descriptions: 摩擦轮电机初始化函数
	** Input:  NULL	
	** Output: NULL
	**************************************************************
**/
void GUN_Init(void)
{
	__HAL_TIM_ENABLE(&htim5);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);   //PH10,右侧摩擦轮
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);   //PH11,左侧摩擦轮
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);   //PH12,左侧备用
	
	HAL_Delay(100);
  TIM5->CCR1 = 200;
	HAL_Delay(3000);
	TIM5->CCR1 = 100;
	HAL_Delay(2000);
	TIM5->CCR1= 105;
  HAL_Delay(1000);
//	HAL_Delay(1000);
	TIM5->CCR2 = 200;
	HAL_Delay(3000);
	TIM5->CCR2 = 100;
	HAL_Delay(2000);
	TIM5->CCR2= 105;
//	HAL_Delay(1000);
//	TIM5->CCR3 = 200;
//	HAL_Delay(3000);
//	TIM5->CCR3 = 100;
//	HAL_Delay(2000);
//	TIM5->CCR3= 105;
	
  HAL_Delay(100);
}

/**
	**************************************************************
	** Descriptions: 摩擦轮电机驱动函数
	** Input: 	
	**	两轮转速:
	**					wheelone
	**					wheeltwo
	** Output: NULL
	**************************************************************
**/

	
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo)
{
  TIM5_PWM_Init(wheelone,wheeltwo);
}

void Friction_Wheel_Motor_Stop(void)
{
	
	htim5.Instance->CR1 &= ~(0x01);  //关闭定时器
		
}

void Maichong_Count(uint8_t i)
{
	
	if(i == 0)
	mc_count[0]++;
	
	if(i == 1)
	mc_count[1]++;
	
	
}

