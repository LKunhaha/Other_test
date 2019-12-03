/* 包含头文件----------------------------------------------------------------*/
#include "chassis_task.h"
#include "SystemState.h"
#include "user_lib.h"
/* 内部宏定义----------------------------------------------------------------*/
#define radian_ratio 0.00076694f   //360/8191*0.01745
/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId Chassis_QueueHandle;

/* 内部常量定义--------------------------------------------------------------
void Chassis_Motor( CAN_HandleTypeDef * hcan,
									  int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
float pid_calc(pid_t* pid, float get, float set);
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
													float dstVmmps_Y,float dstVmmps_W);
----------------------------------------------------------------------------
*/
/* 外部变量声明--------------------------------------------------------------*/
moto3508_type  moto_3508_set = {.flag = 0}; 
extern float power;		//功率  	_测试变量
/* 调用的外部函数原型声明----------------------------------------------------------*/

/* 内部变量------------------------------------------------------------------*/
pid_t pid_3508_pos;     		 //底盘电机位置环
pid_t pid_3508_spd[4];			 //底盘电机速度环
pid_t pid_3508_current[4];	 //底盘电机电流环节	
pid_t pid_chassis_follow = {0};//底盘跟随位置环
pid_t pid_chassis_follow_spd = {0};//底盘跟随速度环

Mode_Set Chassis;
Mode_Set Shoot_mouse;
static float Current_set[4] = {0};  //传递给功率限制的缓存

//测试变量
int16_t angle[2];

#define CHASSIS_PERIOD 5

/* 内部函数原型声明----------------------------------------------------------*/
void Chassis_pid_init(void)
{
	
	 PID_struct_init(&pid_3508_pos, POSITION_PID, 10000, 1000,
									1.5f,	0.0f,	20.0f);  // motos angular rate closeloop.pid:1.5,0.0,20.0
	 pid_3508_pos.deadband=150;
	
	 PID_struct_init(&pid_chassis_follow, POSITION_PID,5000,100,
	                2.0f, 0.0f , 5.0f  );
//	 pid_chassis_follow.deadband=100;
	 PID_struct_init(&pid_chassis_follow_spd, POSITION_PID,5000,100,
	                1.5f, 0.00f , 1.5f  );
	
	
		for(int i=0; i<4; i++)
		{ 
			PID_struct_init(&pid_3508_spd[i], POSITION_PID, 10000, 2000,
										1.5f,	0.1f,	0.1f	);  //4 motos angular rate closeloop.    
		}
	
		PID_struct_init(&pid_3508_current[0], POSITION_PID, 6000, 500,
								0.6f,	0.01f,	0.01f	);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[1], POSITION_PID, 6000, 500,
								0.6f,	0.01f,	0.01f	);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[2], POSITION_PID, 6000, 500,
								0.6f,	0.01f,	0.01f	);  //4 motos angular rate closeloop.
		PID_struct_init(&pid_3508_current[3], POSITION_PID, 6000, 500,
								0.6f,	0.01f,	0.01f	);  //4 motos angular rate closeloop.
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
	static float  wheel[4] = {0,0,0,0};
	osDelay(1000);
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
  Chassis_pid_init();

	for(;;)
	{
	   IMU_Get_Data();

     RefreshTaskOutLineTime(ChassisContrlTask_ON);
          
		 motor_move_setvmmps(wheel,moto_3508_set.dstVmmps_X,moto_3508_set.dstVmmps_Y,moto_3508_set.dstVmmps_W);
		 for(int i = 0; i < 4; i++)
		 {		
			  pid_calc(&pid_3508_spd[i], moto_chassis_get[i].speed_rpm, wheel[i]);
		 }
//		/**********功率限制*********/

//		 Current_set[0] = pid_3508_spd[0].pos_out;
//		 Current_set[1] = pid_3508_spd[1].pos_out;
//		 Current_set[2] = pid_3508_spd[2].pos_out;
//		 Current_set[3] = pid_3508_spd[3].pos_out;			

//		 Super_Capacitance(Current_set);

//			
//		 pid_3508_spd[0].pos_out = Current_set[0];			
//		 pid_3508_spd[1].pos_out = Current_set[1];
//		 pid_3508_spd[2].pos_out = Current_set[2];
//		 pid_3508_spd[3].pos_out = Current_set[3];

		 if(Remote.Mode == 1)
		 {
				Chassis_Motor_Disable(&hcan1);
		 }else
		 {
				Chassis_Motor(&hcan1,pid_3508_spd[0].pos_out, pid_3508_spd[1].pos_out, pid_3508_spd[2].pos_out, pid_3508_spd[3].pos_out);						
		 }
				
		 osDelayUntil(&xLastWakeTime, CHASSIS_PERIOD);
  }
     
}




/***************************************************************************************
**
	*	@brief	hard_brak()
	*	@param
	*	@supplement	紧急停止函数
	*	@retval	
****************************************************************************************/
void hard_brak()
{
		moto_3508_set.dstVmmps_X=0;
		moto_3508_set.dstVmmps_Y=0;
		moto_3508_set.dstVmmps_W=0;
		moto_3508_set.dstVmmps_W=1;
	
}


