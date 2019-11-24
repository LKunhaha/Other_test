#include "BSP.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

xQueueHandle UART1_RX_QueHandle;//串口1接收队列
xQueueHandle UART2_RX_QueHandle;//串口2接收队列
xQueueHandle UART6_RX_QueHandle;//串口6接收队列
xQueueHandle UART8_RX_QueHandle;//串口8接收队列

void Power_Init(void)
{
#if BoardNew

HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2, GPIO_PIN_SET);   //power1
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_3, GPIO_PIN_SET);   //power2
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_4, GPIO_PIN_SET);   //power3
HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //power4

#endif
	HAL_Delay(50);
}


void ConfigureTimerForRunTimeStats(void)  //时间统计
{
	FreeRTOSRunTimeTicks = 0;
	MX_TIM3_Init(); //周期50us，频率20K
}
void BSP_Init(void)
{
	
	/*引脚和引脚时钟*/
  MX_GPIO_Init();
	HAL_Delay(1000);
	Power_Init();
	/*dma*/
  MX_DMA_Init();
	/*can*/
	MX_CAN1_Init();
	MX_CAN2_Init();	
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	/*定时器*/
  MX_TIM5_Init();
	MX_TIM6_Init();
	SystemState_Inite();
  /*ADC*/
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_ADC3_Init();
	
	/*串口*/
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    MX_USART2_UART_Init();
	/*SPI*/
    MX_SPI5_Init();
		
    Bsp_UART_Receive_IT(&huart1,USART1_RX_DATA,SizeofRemote); //这一步的目的是创建一段接受内存
    Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofReferee);	
  /*开启ADC的DMA接收，注意缓存不能小于2，不能设置为_IO型即易变量*/
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)uhADC1ConvertedValue, 10); 
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)uhADC2ConvertedValue, 10); 
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)uhADC3ConvertedValue, 10); 
    __HAL_DMA_DISABLE_IT(&hdma_adc1,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(&hdma_adc2,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
    __HAL_DMA_DISABLE_IT(&hdma_adc3,DMA_IT_TC | DMA_IT_HT | DMA_IT_TE | DMA_IT_FE | DMA_IT_DME);
	/*陀螺仪*/
	  MPU6500_Init();
	/*使能can中断*/
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0); 
    HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0);
	
	  HAL_Delay(1000);
    huart6.gState = HAL_UART_STATE_READY;

}
