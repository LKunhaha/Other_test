/*************************************************************************************
*	@file			AX12-A.h
* @author	 	
*	@version 	V1.0
*	@date			2018/10/8
* @brief		数字总线舵机驱动程序
*************************************************************************************/
#ifndef __ax12_a_h
#define __ax12_a_h

/* Includes ------------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Private function prototypes -----------------------------------------------------*/

void AX_Init(uint8_t id);
void uart_t(uint8_t data);

void Set_AX6(uint8_t id,uint16_t angle,uint16_t speed);
void Open_Door(void);
void Close_Door(void);




#endif
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

