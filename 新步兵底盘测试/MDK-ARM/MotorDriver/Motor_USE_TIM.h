#ifndef  __Motor_USE_TIM_H
#define  __Motor_USE_TIM_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "tim.h"

#define highspeed 200
#define lowspeed  170

extern uint16_t mc_count[2];
extern uint16_t mc_get[2];

void TIM5_PWM_Init(uint32_t speed1,uint32_t speed2);
void GUN_Init(void);
void Friction_Wheel_Motor(uint32_t wheelone,uint32_t wheeltwo);
void Friction_Wheel_Motor_Stop(void);
void Maichong_Count(uint8_t i);

#endif
