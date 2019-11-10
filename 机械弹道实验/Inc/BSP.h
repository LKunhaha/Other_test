#ifndef  __BSP_H
#define  __BSP_H

#include "stm32f4xx_hal.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"
#include "can.h"
#include "spi.h"
#include "adc.h"
#include "communication.h "
#include "Motor_USE_TIM.h"
#include "Motor_USE_CAN.h"
#include "minipc.h"
#include "Power_restriction.h"
#include "atom_imu.h"
#include "decode.h"
#include "SystemState.h"



extern volatile unsigned long long FreeRTOSRunTimeTicks;
void ConfigureTimerForRunTimeStats(void);
void BSP_Init(void);
void JY61_SLEEPorUNSLEEP(UART_HandleTypeDef *huart);
void JY61_Frame(void);


#endif
