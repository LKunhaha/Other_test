#include "BSP.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

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

/**
	**************************************************************
	** Descriptions:	JY61休眠/解休眠
	** Input:	huart  发送指令的串口，波特率要求为115200
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_SLEEPorUNSLEEP(UART_HandleTypeDef *huart)
{
	uint8_t buff[3] = {0xff,0xaa,0x60};
	//休眠,解休眠
	HAL_UART_Transmit(huart,buff,3,10);
}

/**
	**************************************************************
	** Descriptions: JY61帧对齐函数
	** Input: 	
  **						
	**					
	**					
	** Output: NULL
	**************************************************************
**/
void JY61_Frame(void)
{
	static uint8_t JY61_Frame_flag = 0;
	static	uint8_t JY61_Frame_Num = 0;
	
while( UART8_RX_DATA[0] != 0x55 ||  JY61_Frame_flag == 1)
{
	
	if(UART8_RX_DATA[0] != 0x55 && JY61_Frame_flag == 0)
	{
				
				HAL_UART_DMAPause(&huart8);
				*UART8_RX_DATA = 0;
				JY61_Frame_flag = 1;
				
	}
	if(JY61_Frame_flag == 1)//休眠一次，必须解休眠
	{
			JY61_Frame_Num++;
			
					if(JY61_Frame_Num == 25)
					 {
						 
//								JY61_SLEEPorUNSLEEP(&huart8);
//								JY61_Frame_flag = 0;
//								JY61_Frame_Num = 0;
							
								HAL_UART_Receive_DMA(&huart8,UART8_RX_DATA,SizeofJY901);	//陀螺仪接收

				   } else if(JY61_Frame_Num == 50)
							 {
								   HAL_UART_DMAResume(&huart8);
							 } else if(JY61_Frame_Num > 100  )
									 {
										 JY61_Frame_flag = 0;
							       JY61_Frame_Num = 0;
									 }

	 }
}
	
}



void JY901_Init(void)
{
	uint8_t JY901[6][5] = {
													{0xff,0xaa,0x24,0x01,0x00},//六轴算法
													{0xff,0xaa,0x02,0x00,0x00},//开启自动校准
													{0xff,0xaa,0x02,0x0c,0x00},//回传内容:0x0c是输出速度和角度//0x08是只输出角度
													{0xff,0xaa,0x03,0x0b,0x00},//回传速率:200hz
													{0xff,0xaa,0x00,0x00,0x00},//保存当前配置
													{0xff,0xaa,0x04,0x06,0x00}//设置串口波特率:115200
												};
		
	HAL_UART_Transmit_DMA(&huart8,JY901[2],5);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart8,JY901[3],5);
	HAL_Delay(100);
	HAL_UART_Transmit_DMA(&huart8,JY901[4],5);	
	HAL_Delay(100);
	if(HAL_UART_Transmit_DMA(&huart8,JY901[2],5) == HAL_OK )	
	{
		printf("JY901 Init \n\r");
	}
		if(HAL_UART_Transmit_DMA(&huart8,JY901[4],5) == HAL_OK)	
	{
		printf("JY901 Init save\n\r");
	}
}




void BSP_Init(void)
{
	MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM6_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  MX_SPI5_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
	
	/*开发板电源输出*/
	Power_Init();
	
	/*CAN过滤器*/
	CanFilter_Init(&hcan1);
	CanFilter_Init(&hcan2);
	
	/*开启CAN和ADC*/
	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	HAL_ADC_Start(&hadc1);
	
	/*摩擦轮*/
	GUN_Init();
	
	/*板载imu*/
	MPU6500_Init();
	
	/*断线检测*/
	SystemState_Inite();
	
	/*使能串口的DMA接收，开启串口空闲中断*/
	Bsp_UART_Receive_IT(&huart1,USART1_RX_DATA,SizeofRemote); //这一步的目的是创建一段接受内存
	Bsp_UART_Receive_IT(&huart8,UART8_RX_DATA,SizeofJY901);
	Bsp_UART_Receive_IT(&huart6,USART6_RX_DATA,SizeofMinipc);
	
	/*开启ADC的DMA接收，注意缓存不能小于2，不能设置为_IO型即易变量*/
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)uhADCxConvertedValue, 10);

	/*使能can中断*/
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}
