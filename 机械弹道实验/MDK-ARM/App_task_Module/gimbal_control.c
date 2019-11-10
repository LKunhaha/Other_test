/* 包含头文件----------------------------------------------------------------*/
#include "gimbal_control.h"
#include "Power_restriction.h"
#include "SystemState.h"
#include "user_lib.h"

/* 内部变量------------------------------------------------------------------*/
static  int16_t Yaw_Current_Value = 0;
static  int16_t Pitch_Current_Value = 0;

Pos_Set  yaw_set={0};
Pos_Set  yaw_tly_set={0};
Pos_Set  pit_set={0};
Mode_Set Gimbal={0};
Mode_Set Minipc={0};
Mode_Set Shoot={0};

pid_t pid_yaw       = {0};  //yaw轴位置环
pid_t pid_yaw_jy901 = {0};  //外接陀螺仪 /*目前只用于位置环*/ 
pid_t pid_pit       = {0};	//pit轴位置环
pid_t pid_pit_dashen = {0};       //大神符参数
pid_t pid_pit_dashen_spd = {0};
pid_t pid_yaw_spd   = {0};	//yaw轴速度环
pid_t pid_pit_spd   = {0};	//pit轴速度环
pid_t pid_yaw_jy901_spd = {0};
pid_t pid_pit_jy901 = {0};
pid_t pid_pit_jy901_spd = {0};

pid_t pid_yaw_saber = {0};  //外接陀螺仪 /*目前只用于位置环*/
pid_t pid_yaw_saber_spd = {0};
pid_t pid_pit_saber = {0};
pid_t pid_pit_saber_spd = {0};

//zimiao
pid_t pid_yaw_zimiao = {0};        //
pid_t pid_yaw_zimiao_spd = {0};
pid_t pid_pit_zimiao = {0};        //
pid_t pid_pit_zimiao_spd = {0};
/* 内部函数原型声明----------------------------------------------------------*/
/**                                                           //待续
	**************************************************************
	** Descriptions: 云台pid初始化
	** Input:  NULL
	** Output: NULL
	**************************************************************
**/

void gimbal_pid_init(void)    //下次调一下PID
{
	
   /* yaw axis motor pid parameter */
   //大神符模式
   PID_struct_init(&pid_pit_dashen, POSITION_PID, 5000, 1000, 6.0f, 0.05f, 0.5f);  
   PID_struct_init(&pid_pit_dashen_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f);
  
	 PID_struct_init(&pid_yaw, POSITION_PID, 5000, 500, 6.0f, 0.05f, 5.0f); 
	 PID_struct_init(&pid_yaw_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f);
		
   //自瞄模式
   PID_struct_init(&pid_pit_zimiao, POSITION_PID, 5000, 1000, 4.0f, 0.03f, 2.0f);  //4.0 0.03 2.0
   PID_struct_init(&pid_pit_zimiao_spd, POSITION_PID, 5000, 1000, 5.0f, 0.0f, 0.0f);  //1.5 0.0 0.0
  
	 PID_struct_init(&pid_yaw_zimiao, POSITION_PID, 5000, 300, 15.0f, 0.01f, 25.0f); // 6.0 0.05 5.0 //15.0 0.01 25.0
	 PID_struct_init(&pid_yaw_zimiao_spd, POSITION_PID, 5000, 1000, 1.5f, 0.0f, 0.0f);  
  
   //陀螺仪模式
   PID_struct_init(&pid_pit, POSITION_PID, 5000, 100, 4.5f, 0.01f, 10.0f); 
   PID_struct_init(&pid_pit_jy901_spd, POSITION_PID, 5000, 1000, 2.0f, 0.0f, 0.0f );
  
   PID_struct_init(&pid_yaw_jy901, POSITION_PID, 5000, 500, 2.5f, 0.01f, 0.0f); 
   PID_struct_init(&pid_yaw_jy901_spd, POSITION_PID, 5000, 500, 2.0f, 0.0f, 1.0f );	
	
}
/* 任务主体部分 -------------------------------------------------------------*/

/***************************************************************************************
**
	*	@brief	Gimbal_Contrl_Task(void const * argument)
	*	@param
	*	@supplement	云台电机控制
	*	@retval	
****************************************************************************************/
void Gimbal_Task(void const * argument)
{
	yaw_set.mode = 0;
	Gimbal.flag = 0;
	Gimbal.mode = 3;          //初始设定为编码器模式
	Pitch_Current_Value = 0;
	Yaw_Current_Value = 0;
	gimbal_pid_init();
	
	osDelay(200);//延时200ms
	portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
/*************************     模式说明     ********************************
		minipc.mode              ：0  关闭视觉
	                             1  自瞄               将gimbal.mode置为2
															 2  能量机关           将gimbal.mode置为3
					
		gimbal.mode	             ：0  云台失效
															 1  陀螺仪模式(对应底盘的跟随模式,非扭腰模式下)
															 2  自瞄
															 3  编码器模式(YAW轴超限时、能量机关模式 也置为此模式) 
					
		gimbal.flag	             ：0  yaw轴不超限
															 1  暂未使用
															 2  yaw超限标志
															 3  暂未使用
	****************************************************************************/			
	for(;;)		
  {
		  IMU_Get_Data();
		  CAN_Send_Gimbal(&hcan2,yaw_get.angle,yaw_get.total_angle,Minipc.mode,Gimbal.mode,Gimbal.flag,Remote.Mode);//主控发送
			
	    RefreshTaskOutLineTime(GimbalContrlTask_ON);
      
			if(Minipc.mode == 2)   //能量机关
			{
            Gimbal.mode = 3;
            yaw_set.expect = yaw_set.expect_pc;
            pit_set.expect = pit_set.expect_pc;
        
            yaw_tly_set.expect_remote = ptr_jy901_t_yaw.final_angle;       //保存当前yaw角度值(以陀螺仪的方式,实时更新)
            pit_set.expect_remote = pit_get.total_angle;                    //保存当前pitch轴对的角度值(以电机角度的方式,实时更新)
			}else if(Minipc.mode == 1 && Gimbal.flag == 0) //自瞄&&不超限
			{
            Gimbal.mode = 2;
            yaw_set.expect = yaw_set.expect_pc;
            pit_set.expect = pit_set.expect_pc;
        
            yaw_tly_set.expect_remote = ptr_jy901_t_yaw.final_angle;     //保存当前yaw角度值(以陀螺仪的方式,实时更新)，防止取消自瞄后，产生较大的角度波动
            pit_set.expect_remote = pit_get.total_angle;                 //保存当前pitch轴对的角度值(以电机角度的方式,实时更新)，同上
			}else                                                //遥控模式
			{                                                    //无视觉模式  或  自瞄&&超限
            yaw_tly_set.expect = yaw_tly_set.expect_remote;            
            pit_set.expect = pit_set.expect_remote;
			}
			
			
			if((yaw_get.angle < 3900) )                 //云台超限处理 
			{
            yaw_set.expect = 3900 - yaw_get.offset_angle;
            Gimbal.mode = 3;                      //超限后以编码器模式调整回来
            Gimbal.flag = 2;                      //超限标志位
			}else if((yaw_get.angle > 7500) )    
			{
            yaw_set.expect = 7500 - yaw_get.offset_angle;
            Gimbal.mode = 3;                      //超限后以编码器模式调整回来
            Gimbal.flag = 2;                      //超限标志位       
			}
			else if (Gimbal.flag == 2)                  //Yaw轴超限后，只有当yaw轴不超限才会进入此判断，并清除超限标志位
			{   
//            Gimbal.mode = 1;                    //置为陀螺仪模式
				    Gimbal.flag = 0;                      //清除超限标志
//            yaw_set.expect = 0;
            yaw_tly_set.expect = ptr_jy901_t_yaw.final_angle;  //yaw轴给定值为当前的yaw轴角度值(陀螺仪方式)		                                                 
			}
     
	
			if ( pit_get.angle < 5700)
			{
				    pit_set.expect_remote = 5800 - pit_get.offset_angle;
				    pit_set.expect = 5800 - pit_get.offset_angle;
			}
			else if (pit_get.angle > 7500)
			{
				    pit_set.expect_remote = 7450 - pit_get.offset_angle;
				    pit_set.expect = 7450 - pit_get.offset_angle;
			}
	

			switch(Gimbal.mode)
			{	
				case 1: 
				{     //陀螺仪模式
							pid_calc(&pid_yaw_jy901,(ptr_jy901_t_yaw.final_angle), yaw_tly_set.expect);  //ptr_jy901_t_yaw.final_angle为相对于初始陀螺仪的角度值，类似于云台电机的总角度算法
							pid_calc(&pid_yaw_jy901_spd,(imu_data.gx)/30, pid_yaw_jy901.pos_out);
					    Yaw_Current_Value = (-pid_yaw_jy901_spd.pos_out);
					
          		//pit轴
	            pid_calc(&pid_pit, pit_get.total_angle, pit_set.expect);
							pid_calc(&pid_pit_jy901_spd,(imu_data.gz)/16.4, pid_pit.pos_out);
              Pitch_Current_Value = (-pid_pit_jy901_spd.pos_out); 
					
				      yaw_set.expect = yaw_get.total_angle;
				}break;
				
        case 2:
				{
					    //自瞄模式
							pid_calc(&pid_yaw_zimiao,yaw_get.total_angle, yaw_set.expect);
							pid_calc(&pid_yaw_zimiao_spd,(imu_data.gx)/30, pid_yaw_zimiao.pos_out);
					    Yaw_Current_Value = (-pid_yaw_zimiao_spd.pos_out);
          
          		//pit轴
							pid_calc(&pid_pit_zimiao, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_zimiao_spd,(imu_data.gz)/16.4, pid_pit_zimiao.pos_out);
              Pitch_Current_Value = (-pid_pit_zimiao_spd.pos_out); 
				 }break;
				
				case 3: 
				{     //编码器模式  
							pid_calc(&pid_yaw, yaw_get.total_angle, yaw_set.expect);
						  pid_calc(&pid_yaw_spd,(imu_data.gx)/30, pid_yaw.pos_out);
							Yaw_Current_Value = (-pid_yaw_spd.pos_out);
				
							pid_calc(&pid_pit_dashen, pit_get.total_angle, pit_set.expect);
						  pid_calc(&pid_pit_dashen_spd,(imu_data.gz)/16.4, pid_pit_dashen.pos_out);   
							Pitch_Current_Value = (-pid_pit_dashen_spd.pos_out); 
					
					    yaw_tly_set.expect = ptr_jy901_t_yaw.final_angle;
				 }break; 
//        case 4 ://回中
//               {
//           
//               }break;
				 default: break;
				
			}                                                                             
//		 		
			Cloud_Platform_Motor(&hcan1,-Yaw_Current_Value,Pitch_Current_Value);
			pit_set.expect_remote_last = pit_set.expect_remote;
			
		  osDelayUntil(&xLastWakeTime, GIMBAL_PERIOD);
  }
 
}
