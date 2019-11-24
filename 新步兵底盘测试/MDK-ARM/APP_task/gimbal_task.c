/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_task.h"
#include "Power_restriction.h"
#include "SystemState.h"
/* 内部宏定义----------------------------------------------------------------*/

/* 内部自定义数据类型--------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;
/* 任务相关信息定义----------------------------------------------------------*/

/* 内部常量定义--------------------------------------------------------------*/
#define GIMBAL_PERIOD 5
/* 外部变量声明--------------------------------------------------------------*/
Pos_Set  yaw_set;
Pos_Set  pit_set;
int8_t gimbal_disable_flg;

/* 调用的外部函数原型声明------------------------------------------------------------
void Cloud_Platform_Motor(CAN_HandleTypeDef * hcan,int16_t yaw,int16_t	pitch);
float pid_calc(pid_t* pid, float get, float set);
------------------------------------------------------------------------------
*/
/* 内部变量------------------------------------------------------------------*/
pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_jy901 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};

pid_t pid_yaw_saber = {0};  //外接陀螺仪 /*目前只用于位置环*/
pid_t pid_yaw_saber_spd = {0};
pid_t pid_pit_saber = {0};
pid_t pid_pit_saber_spd = {0};
/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)
{
		/*pitch axis motor pid parameter*/
	PID_struct_init(&pid_pit, POSITION_PID, 5000, 1000,
                  4.0f, 0.02f, 5.0f); 
  PID_struct_init(&pid_pit_jy901_spd, POSITION_PID, 5000, 1000,
                  2.0f, 0.0f, 0.0f );
	
  /* yaw axis motor pid parameter */
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 1000,
                  10.0f, 0.1f, 5.0f);
  PID_struct_init(&pid_yaw_jy901, POSITION_PID, 5000, 100,
                  3.0f, 0.02f, 10.0f); //	
  PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 5000, 100,
                  2.0f, 0.0f, 0.5f ); 

	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Contrl_Task(void const * argument)
{
	yaw_set.expect = 0; 
	pit_set.expect = 0;
	yaw_set.mode   = 0;
	gimbal_disable_flg=0;
	Pitch_Current_Value=0;
	Yaw_Current_Value=0;
	gimbal_pid_init();
	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();	
			
	for(;;)		
    {
	
	   RefreshTaskOutLineTime(GimbalContrlTask_ON);

			
			switch(1)
			{
				case 1: {
					           
										 pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
										 pid_calc(&pid_yaw_jy901_spd,(imu_data.gz), pid_yaw.pos_out);
				        }break;                                                                           //编码器模式
				case 2: {
										 pid_calc(&pid_yaw_jy901,(ptr_jy901_t_yaw.final_angle),yaw_set.expect);
										 pid_calc(&pid_yaw_jy901_spd,-(imu_data.gz), pid_yaw_jy901.pos_out);	
				        }break;//陀螺仪模式
				default:
				break;
			}                                                                                                 
		 
			//pit轴
					pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
					pid_calc(&pid_pit_jy901_spd,(imu_data.gx), pid_pit.pos_out);
			
//		  	Pitch_Current_Value=(-pid_pit_jy901_spd.pos_out); 
//		    Yaw_Current_Value= (-pid_yaw_jy901_spd.pos_out); 
				if(gimbal_disable_flg==1)
				{
					Cloud_Platform_Motor_Disable(&hcan1);
				}
				else Cloud_Platform_Motor(&hcan1,Yaw_Current_Value,Pitch_Current_Value);

			osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
			
   }
 
}
