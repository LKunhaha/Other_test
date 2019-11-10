#include "other_tasks.h"
#include "gun_control.h"
#include "gimbal_control.h"
/* 内部变量------------------------------------------------------------------*/
uint32_t test_i = 0;
/* 内部函数原型声明----------------------------------------------------------*/
extern Mode_Set bopan;
/* 任务主体部分 -------------------------------------------------------------*/
void testTask(void const * argument)
{
	int16_t angle[4];	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{ 				
		RefreshTaskOutLineTime(testTask_ON);
    test_i++;
//		  int16_t  *ptr = angle; //初始化指针
//			angle[0] = 0;
//			angle[1] = 0;
//			angle[2] = 0;
//			angle[3] = 0;
//			/*用虚拟示波器，发送数据*/
//			vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[0]));
//		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[1]));
//		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[2]));
//		  vcan_sendware((uint8_t *)ptr,4 * sizeof(angle[3]));
	
		
		  osDelayUntil(&xLastWakeTime,7);
	}
}
