/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
  ******************************************************************************
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim3;      //时间统计函数时基 
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;


void MX_TIM2_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_ClockConfigTypeDef sClockSourceConfig;
	
	
	htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	
	
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
   Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
   Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
   Error_Handler();
  }

//  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
//  {
//   Error_Handler();
//  }

//  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
//  {
//   Error_Handler();
//  }	
}

void MX_TIM4_Init(void)
{
	TIM_Encoder_InitTypeDef sConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;
  TIM_ClockConfigTypeDef sClockSourceConfig;
	
	
	htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	
	
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
   Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
   Error_Handler();
  }

  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
   Error_Handler();
  }
	
}



/* TIM5 init function */
void MX_TIM5_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	
	  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 200;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim5);

}

/* TIM12 init function */
void MX_TIM12_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84 - 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 5 - 1;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
	
	HAL_TIM_Base_Start_IT(&htim12); //使能定时器12和定时器12更新中断：TIM_IT_UPDATE   

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
	  htim3.Instance=TIM3;                          //通用定时器3
    htim3.Init.Prescaler=84-1;                     //分频系数
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    htim3.Init.Period=50-1;                        //自动装载值
    htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&htim3);
    
    HAL_TIM_Base_Start_IT(&htim3); //使能定时器3和定时器3更新中断：TIM_IT_UPDATE   
}

/* TIM6 init function */
void MX_TIM6_Init(void)
{
	  htim6.Instance=TIM6;                          //通用定时器6
    htim6.Init.Prescaler=840-1;                     //分频系数
    htim6.Init.CounterMode=TIM_COUNTERMODE_UP;    //向上计数器
    htim6.Init.Period=100-1;                        //自动装载值
    htim6.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;//时钟分频因子
    HAL_TIM_Base_Init(&htim6);
    
    HAL_TIM_Base_Start_IT(&htim6); //使能定时器6和定时器6更新中断：TIM_IT_UPDATE  
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM12)
  {
  /* USER CODE BEGIN TIM12_MspInit 0 */

  /* USER CODE END TIM12_MspInit 0 */
    /* TIM12 clock enable */
    __HAL_RCC_TIM12_CLK_ENABLE();

    /* TIM12 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
  /* USER CODE BEGIN TIM12_MspInit 1 */

  /* USER CODE END TIM12_MspInit 1 */
  }else if(tim_baseHandle->Instance==TIM5)
	{
		__HAL_RCC_TIM5_CLK_ENABLE();

	}else if(tim_baseHandle->Instance==TIM3)
	{
		__HAL_RCC_TIM3_CLK_ENABLE();            //使能TIM3时钟
		HAL_NVIC_SetPriority(TIM3_IRQn,1,0);    //设置中断优先级，抢占优先级1，子优先级0
		HAL_NVIC_EnableIRQ(TIM3_IRQn);          //开启ITM3中断  
	}else if(tim_baseHandle->Instance==TIM6)
	{
		__HAL_RCC_TIM6_CLK_ENABLE();            //使能TIM6时钟
		HAL_NVIC_SetPriority(TIM6_DAC_IRQn,2,0);    //设置中断优先级，抢占优先级1，子优先级0
		HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);          //开启ITM6中断  
	}
	
	
}


void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspPostInit 0 */

  /* USER CODE END TIM5_MspPostInit 0 */
  
    /**TIM5 GPIO Configuration    
    PH11     ------> TIM5_CH2
    PH10     ------> TIM5_CH1 
    */
    GPIO_InitStruct.Pin = TIM5_CH2_Pin|TIM5_CH1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM5_MspPostInit 1 */

  /* USER CODE END TIM5_MspPostInit 1 */
  }else if(timHandle->Instance==TIM12)
	{
	    /**TIM5 GPIO Configuration    
    PH9     ------> TIM12_CH2
    PH6     ------> TIM12_CH1 
    */
    GPIO_InitStruct.Pin = TIM12_CH2_Pin|TIM12_CH1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
	}

}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

  if(tim_pwmHandle->Instance==TIM5)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */
    __HAL_RCC_TIM5_CLK_DISABLE();

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
  }else if(tim_pwmHandle->Instance==TIM12)
	{
  /* USER CODE BEGIN TIM12_MspDeInit 0 */

  /* USER CODE END TIM12_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM12_CLK_DISABLE();

    /* TIM12 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
  /* USER CODE BEGIN TIM12_MspDeInit 1 */

  /* USER CODE END TIM12_MspDeInit 1 */
	}
} 


void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(htim->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM5_MspDeInit 0 */
    __HAL_RCC_TIM2_CLK_ENABLE();
			
	  GPIO_InitStruct.Pin = TIM2_CH1_Pin|TIM2_CH2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
		 HAL_NVIC_SetPriority(TIM2_IRQn,6,0);    //设置中断优先级，抢占优先级1，子优先级0
		 HAL_NVIC_EnableIRQ(TIM2_IRQn);          //开启ITM2中断  
	}
	else if(htim->Instance == TIM4)
	{
	   __HAL_RCC_TIM4_CLK_ENABLE();
			
	  GPIO_InitStruct.Pin = TIM4_CH1_Pin|TIM4_CH2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE END TIM5_MspDeInit 0 */
    /* Peripheral clock disable */
		 HAL_NVIC_SetPriority(TIM4_IRQn,6,0);    //设置中断优先级，抢占优先级1，子优先级0
		 HAL_NVIC_EnableIRQ(TIM4_IRQn);          //开启ITM2中断  
	}
	
}






//void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
//{
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	if(htim->Instance==TIM2)
//  {
//  /* USER CODE BEGIN TIM5_MspDeInit 0 */
//    __HAL_RCC_TIM2_CLK_ENABLE();
//			
//	  GPIO_InitStruct.Pin = TIM2_CH1_Pin|TIM2_CH2_Pin;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//		
//		TIM2->CNT=0;
//  /* USER CODE END TIM5_MspDeInit 0 */
//    /* Peripheral clock disable */
//	}
////	else if(htim->Instance == TIM4)
////	{
////		__HAL_RCC_TIM4_CLK_ENABLE();
////			
////	  GPIO_InitStruct.Pin = TIM4_CH1_Pin|TIM4_CH2_Pin;
////    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
////    GPIO_InitStruct.Pull = GPIO_NOPULL;
////    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
////    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
////    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
////		
////		TIM4->CNT=0;
////	}
//}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
