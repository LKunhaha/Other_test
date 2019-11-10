/* 包含头文件----------------------------------------------------------------*/
#include "chassis_control.h"
#include "SystemState.h"

/* 内部变量------------------------------------------------------------------*/
pid_t pid_3508_pos;     		 //底盘电机位置环
pid_t pid_3508_spd[4];			 //底盘电机速度环
pid_t pid_3508_current[4];	 //底盘电机电流环节	
pid_t pid_chassis_follow = {0};//底盘跟随位置环
pid_t pid_chassis_follow_spd = {0};//底盘跟随速度环

static float Current_set[4] = {0};  //传递给功率限制的缓存

//测试变量
int16_t angle[2];



/* 内部函数原型声明----------------------------------------------------------*/
void Chassis_pid_init(void)
{
	
	 PID_struct_init(&pid_3508_pos, POSITION_PID, 10000, 1000, 1.5f,	0.0f,	20.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
	 pid_3508_pos.deadband = 150;
	
	 PID_struct_init(&pid_chassis_follow, POSITION_PID,10000,1000, 2.0f, 0.01f , 1.0f);
	 pid_chassis_follow.deadband = 500;
	 PID_struct_init(&pid_chassis_follow_spd, POSITION_PID, 5000, 1000, 0.7f, 0.0f , 0.0f);
	
	 for(int i=0; i<4; i++)
	 { 
		 PID_struct_init(&pid_3508_spd[i], POSITION_PID, 10000, 2000, 1.5f,	0.1f,	0.1f);  //4 motos angular rate closeloop.
   }
	
		PID_struct_init(&pid_3508_current[0], POSITION_PID, 6000, 500, 0.6f, 0.01f, 0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[1], POSITION_PID, 6000, 500, 0.6f, 0.01f,	0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[2], POSITION_PID, 6000, 500, 0.6f, 0.01f, 0.01f);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[3], POSITION_PID, 6000, 500, 0.6f, 0.01f,	0.01f);  //4 motos angular rate closeloop.
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Chassis_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	底盘控制任务
	*	@retval	
****************************************************************************************/
void Chassis_Contrl_Task(void const * argument)
{
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
	Chassis_pid_init();
	
	for(;;)
	{
			osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
	}
}


