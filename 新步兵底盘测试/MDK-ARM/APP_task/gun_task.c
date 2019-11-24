/* 包含头文件----------------------------------------------------------------*/
#include "gun_task.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/

/* 任务相关信息定义----------------------------------------------------------*/
//extern osMessageQId JSYS_QueueHandle;
/* 内部常量定义--------------------------------------------------------------*/
#define GUN_PERIOD  10
#define Mocha_PERIOD  1
#define BLOCK_TIME 1000
#define REVERSE_TIME 2000
/* 外部变量声明--------------------------------------------------------------*/
Heat_Gun_t  ptr_heat_gun_t;
extern uint8_t shot_frequency;
volatile uint8_t finish_flag = 0;
//Power_Heat * power_heat;
/* 外部函数原型声明-----------------------------------------------------------

-----------------------------------------------------------------------------
-*/
/* 内部变量------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000,
									0.2f,	0.0000f,	2.0f);  
		//pid_pos[i].deadband=500;
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000,
									1.5f,	0.1f,	0.0f	);  
		pid_pit_spd.deadband=10;//2.5f,	0.03f,	1.0f	

//    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_5, GPIO_PIN_SET);   //电源引脚 _待续
}

void Mocha_Pid_Init()
{
	
		PID_struct_init(&pid_rub_spd[0], POSITION_PID, 10, 1,
									0.05f,	0.0f,	0.0f	);  
		PID_struct_init(&pid_rub_spd[1], POSITION_PID, 10, 1,
									0.05f,	0.0f,	0.0f	);  
	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gun_Task(void const * argument)
	*	@param
	*	@supplement	枪口热量限制任务
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 

	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
	
	uint8_t motor_stop_flag=0;
	static int32_t set_angle = 0;
	int32_t set_speed = 0;
	static uint8_t set_cnt = 0;
  static uint8_t block_flag;
  /*设定发弹*/

	for(;;)
	{
		
		RefreshTaskOutLineTime(GunTask_ON);

 /*判断拨盘是否转到位置*/		
		if(moto_dial_get.round_cnt <=-5*set_cnt)
			{
				moto_dial_get.round_cnt=0;
				moto_dial_get.offset_angle=moto_dial_get.angle;
				moto_dial_get.total_angle=0;	
				moto_dial_get.run_time=GetSystemTimer();
			}
			else
			{
					if( my_abs(moto_dial_get.run_time-moto_dial_get.cmd_time)>BLOCK_TIME )//堵转判定
					{
					
						block_flag=1;
						
					}
		  }
	
			if( moto_dial_get.reverse_time-moto_dial_get.run_time < REVERSE_TIME && block_flag)//反转设定
			{
				ptr_heat_gun_t.sht_flg=10;//反转
				moto_dial_get.round_cnt=0;
				moto_dial_get.offset_angle=moto_dial_get.angle;
				moto_dial_get.total_angle=0;	
				
			}else     block_flag=0;
			
 /*判断发射模式*/
    switch(ptr_heat_gun_t.sht_flg)
    {
			case 0://停止
			{
				set_angle=0;
				set_speed=0;
				set_cnt=0;
				moto_dial_get.cmd_time=GetSystemTimer();
			}break;
      case 1://按键单发模式
      {
				moto_dial_get.cmd_time=GetSystemTimer();
				set_cnt=1;
				set_angle=-42125*set_cnt;
				
        /*pid位置环*/
        pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
				set_speed=pid_dial_pos.pos_out;

      }break;
      case 2://固定角度模式
      {
				moto_dial_get.cmd_time=GetSystemTimer();
				set_cnt=3;
				set_angle=-42125*set_cnt;
			
        /*pid位置环*/
        pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
				set_speed=pid_dial_pos.pos_out;
      }break;
      case 3://连发模式
      { 
				moto_dial_get.cmd_time=GetSystemTimer();
        set_speed=-5000;
        set_cnt=1;
				
      }break;
			case 10://反转
			{
				
				set_speed=1000;
				
				moto_dial_get.reverse_time=GetSystemTimer();
        /*pid位置环*/
			}break;
			default :break;
    }
     /*速度环*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*驱动拨弹电机*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
		 minipc_rx.state_flag=0;
		 set_speed=0;
     osDelayUntil(&xLastWakeTime,GUN_PERIOD);
	}
}


void Mocha_Task(void const *argument)
{
	
	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		IMU_Get_Data();
		
   switch(ptr_heat_gun_t.sht_flg)
    {
			case 0://停止
			{
	       Friction_Wheel_Motor(1000,1000);
			}break;
			
			case 1://单发模式
      {
				Friction_Wheel_Motor(1700,1700);
      }break;
			
			case 2://固定角度模式
      {
				Friction_Wheel_Motor(1500,1500);
      }break;
      case 3://连发模式
      {
				 Friction_Wheel_Motor(1300,1300);
      }break;
			case 10://反转
			{
//			 Friction_Wheel_Motor(1010,1010);
			}break;
			default :break;
			
		}
	
	 osDelayUntil(&xLastWakeTime,Mocha_PERIOD);
		
	}
}

