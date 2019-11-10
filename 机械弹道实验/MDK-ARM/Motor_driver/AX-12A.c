/*************************************************************************************
*	@file			AX12-A.c
* @author	 	NCURM
*	@version 	V1.0
*	@date			2018/10/8
* @brief		数字总线舵机驱动程序
*************************************************************************************/
/* Includes ------------------------------------------------------------------------*/
#include "AX-12A.h"
/* External variables --------------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
/* Internal variables --------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------------*/

void AX_Init(uint8_t id)
{

    //uint8_t id[8] = {0x0e,0x12,0x10,0x03};
    
//  for(uint8_t i = 1;i < 15;i++)
//    {
      uart_t(0xff);
      uart_t(0xff);
      uart_t(0xfe);
      uart_t(0x09);
      uart_t(0x83);
      uart_t(0x06);
      uart_t(0x04);
      uart_t(id);
      uart_t(0x00);
      uart_t(0x00);  
      uart_t(0xff);
      uart_t(0x03);
      uart_t(~((0x94 + 0x03 + 0xff + (id)) & 0xff));
      /*设置波特率*/
      uart_t(0xff);
      uart_t(0xff);
      uart_t(0xfe);
      uart_t(0x06);
      uart_t(0x83);
      uart_t(0x04);
      uart_t(0x01);
      uart_t(id);
      uart_t(0x10);//115200
      uart_t(~((0x9c + (id)) & 0xff));
//    }
}
void uart_t(uint8_t data)
{
  
  HAL_UART_Transmit(&huart2,&data,1,10);
  

}


void Set_AX6(uint8_t id,uint16_t angle,uint16_t speed)
{
  uint8_t low = angle;
  uint8_t high = angle >> 8;
  uint8_t low2 = speed;
  uint8_t high2 = speed >> 8;

  uart_t(0xff);
  uart_t(0xff);
  uart_t(0xfe);
  uart_t(0x0b);
  uart_t(0x83);
  uart_t(0x1c);
  uart_t(0x06);
  uart_t(id);
  uart_t(0x20);
  uart_t(0x20);  
  uart_t(low);
  uart_t(high);
  uart_t(low2);
  uart_t(high2);
  uart_t(~((0xee + (id) + (high) + (low) + (high2) + (low2)) & 0xff));

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

