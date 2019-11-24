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


extern xQueueHandle UART1_RX_QueHandle;//
extern xQueueHandle UART2_RX_QueHandle;//
extern xQueueHandle UART6_RX_QueHandle;//
extern xQueueHandle UART8_RX_QueHandle;//


extern volatile unsigned long long FreeRTOSRunTimeTicks;
void ConfigureTimerForRunTimeStats(void);
void BSP_Init(void);


#endif
