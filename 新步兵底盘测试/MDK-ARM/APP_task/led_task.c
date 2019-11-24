/* 包含头文件----------------------------------------------------------------*/
#include "led_task.h"
#include "SystemState.h"
#include "gpio.h"
#include "Motor_USE_CAN.h"
#include "communication.h "
#include "usart.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define LED_PERIOD  600
#define Check_PERIOD  100
/* 外部变量声明--------------------------------------------------------------*/

SystemStateDef Remote;
SystemStateDef Chassis_motor;
SystemStateDef Gimbal_Motor;
SystemStateDef JY61;
/* 外部函数原型声明-----------------------------------------------------------

-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/
/* 内部函数原型声明----------------------------------------------------------*/

/* 任务主体部分 -------------------------------------------------------------*/



void Led_Task(void const * argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		RefreshTaskOutLineTime(LedTask_ON);
		

			  if((SystemState.OutLine_Flag&0x10))
				{
					  printf("BANGA over");  //貌似是云台与底盘通信出错标志位
				      Chassis_motor.Mode=5;
					 osDelayUntil(&xLastWakeTime,200);
				} 
				else if((SystemState.OutLine_Flag&0x01))
				{
					  Chassis_motor.Mode=1;
					  printf("motor1 over");
						osDelayUntil(&xLastWakeTime,200);
				}else if((SystemState.OutLine_Flag&0x02))
				{
					  Chassis_motor.Mode=2;
					  printf("motor2 over");
						osDelayUntil(&xLastWakeTime,200);
				} else if((SystemState.OutLine_Flag&0x04))
				{
					  Chassis_motor.Mode=3;
					  printf("motor3 over");
						osDelayUntil(&xLastWakeTime,200);
				} else  if((SystemState.OutLine_Flag&0x08))
				{
					  Chassis_motor.Mode=4;
					  printf("motor4 over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
						Chassis_motor.Mode=0;
				}
                
				

				
				if((SystemState.OutLine_Flag&0x20))//裁判系统通信失败
				{
					  printf("Referee over");
						osDelayUntil(&xLastWakeTime,200);
				} else  
				{
//						Chassis_motor.Mode=0;
				}

				if((SystemState_can.OutLine_Flag&0x01))//遥控器掉线
				{
					  printf("remote over");
						osDelayUntil(&xLastWakeTime,200);				  
				}
				if((SystemState_can.OutLine_Flag&0x02))
				{
					  printf("motor_Y over");
						osDelayUntil(&xLastWakeTime,200);
				}
				if((SystemState_can.OutLine_Flag&0x04))
				{
					  printf("motor_P over");
						osDelayUntil(&xLastWakeTime,200);
				} 
				if((SystemState_can.OutLine_Flag&0x08))
				{
					  printf("motor_B over");
						osDelayUntil(&xLastWakeTime,200);
				} 
				
				if((SystemState_can.OutLine_Flag&0x10))
				{
					  printf("JY61 over");
					  
				} 
			
			osDelayUntil(&xLastWakeTime,LED_PERIOD);
 }
}



void Check_Task(void const * argument)
{
	osDelay(100);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
	if((SystemState.task_OutLine_Flag&0x01))
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
				
				
				if((SystemState.task_OutLine_Flag&0x02))
				{
					printf("ChassisContrlTask GG \n\t");
					Chassis_Motor_Disable(&hcan1);
					osDelayUntil(&xLastWakeTime,100);
				} 
				
				
				if((SystemState.task_OutLine_Flag&0x04))
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState.task_OutLine_Flag&0x08))
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 

				
	    	if((SystemState_can.task_OutLine_Flag&0x01))
				{
					printf("testTask GG \n\t");
					osDelayUntil(&xLastWakeTime,100);
				}
	  
				if((SystemState_can.task_OutLine_Flag&0x02))
				{
						printf("RemoteDataTask GG \n\t");
			      Remote_Disable();
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState_can.task_OutLine_Flag&0x04))
				{
						printf("GimbalContrlTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState_can.task_OutLine_Flag&0x08))
				{
						printf("GunTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
				
				if((SystemState_can.task_OutLine_Flag&0x10))
				{
						printf("LedTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 


				if((SystemState_can.task_OutLine_Flag&0x20))
				{
						printf("vOutLineCheckTask GG \n\t");
						osDelayUntil(&xLastWakeTime,100);
				} 
			
				osDelayUntil(&xLastWakeTime,Check_PERIOD);
	
	}
}


