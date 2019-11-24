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
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
		TIM5_PWM_Init(2000,2000);
		HAL_Delay(3000);
		TIM5_PWM_Init(1000,1000);
		HAL_Delay(2000);
	  TIM5_PWM_Init(1000,1000);
		//
//		TIM5_PWM_Init(lowspeed,lowspeed);

		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
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

pid_t pid_rub_spd[2]  = {0,0};	  //抹茶速度环

	
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo)
{
	
		static int16_t Val[2]={0,0}; 

#if	Mocha_Moshi
					pid_calc(&pid_rub_spd[0], mc_get[0], wheelone);	
					Val[0] += (int16_t)pid_rub_spd[0].pos_out;

					if(Val[0]>200)  Val[0]=200;
					if(Val[0]<100)  Val[0]=100;

					pid_calc(&pid_rub_spd[1], mc_get[1], wheeltwo);	
					Val[1] += (int16_t)pid_rub_spd[1].pos_out;

					if(Val[1]>200)  Val[1]=200;
					if(Val[1]<100)  Val[1]=100;

					TIM5_PWM_Init(Val[0],Val[1]);
#else
          TIM5_PWM_Init(wheelone,wheeltwo);
#endif		
				
		
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