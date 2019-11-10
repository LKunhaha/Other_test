/* 包含头文件----------------------------------------------------------------*/
#include "data_processing.h"
#include "SystemState.h"
#include "user_lib.h"
#include "main.h"
#include "AX-12A.h"

/* 内部变量------------------------------------------------------------------*/
pid_t pid_minipc_yaw = {0};
pid_t pid_minipc_pit = {0};
Mode_Set Shoot_mouse = {0};

uint8_t Reset_Mocha = 0;
/* 内部函数原型声明-----------------------------------------------------------*/


void Minipc_Pid_Init()
{
		PID_struct_init(&pid_minipc_yaw, POSITION_PID, 50, 1, 1.0f,	0.00f, 0.0f);  
		PID_struct_init(&pid_minipc_pit, POSITION_PID, 6000, 5000, 0.5f,	0.00f, 0.0f);   
}


/* 任务主体部分 -------------------------------------------------------------*/
/***************************************************************************************
**
	*	@brief	Data_Pro_task(void const * argument)
	*	@param
	*	@supplement	遥控数据接收及处理任务
	*	@retval	
****************************************************************************************/
void Remote_Data_Task(void const * argument)
{
	  uint32_t NotifyValue;

		portTickType xLastWakeTime;
		xLastWakeTime = xTaskGetTickCount();
	
	for(;;)
	{
		NotifyValue = ulTaskNotifyTake(pdTRUE,portMAX_DELAY);  //未有任务通知则进入堵塞状态去等待任务通知
		
    if(NotifyValue == 1)
		{
			RefreshTaskOutLineTime(RemoteDataTask_ON);
			
			Remote_Ctrl();   //接收机原始数据处理
			
			Send_MiniPC_Data(Minipc.color,Minipc.mode,yaw_get.offset_angle,pit_get.offset_angle,0);
			CAN_Send_Remote(&hcan2,RC_Ctl.key.v,RC_Ctl.rc.ch0,RC_Ctl.rc.ch1,RC_Ctl.rc.s1,RC_Ctl.rc.s2);
			
			RemoteControlProcess();   //遥控器数据处理
		}
		
		osDelayUntil(&xLastWakeTime, REMOTE_PERIOD);
	}
}

/***************************************************************************************
**
	*	@brief	RemoteControlProcess()
	*	@param
	*	@supplement	与遥控器进行对接，对遥控器的数据进行处理，实现对底盘、云台、发射机构的控制
	*	@retval	
****************************************************************************************/
void RemoteControlProcess()  
{
		switch(RC_Ctl.rc.s2)
		{
			case 1: //上
			{
				 Gimbal.mode = 1;            //陀螺仪模式
				 Set_AX6(2,819,0x3ff);       //逆时针角度   240° 关
//						RemoteControlProcess();
			}break; 
			case 2: //下
			{
				 Set_AX6(2,819,0x3ff);       //逆时针角度   240° 关
			   Gimbal.mode = 3;            //编码器模式  拨盘调试置为编码器模式
//						MouseKeyControlProcess();
			}break; 
			case 3: //中
			{
				 Set_AX6(2,512,0x3ff);       //逆时针角度   150°
//						hard_brak();
			}break; 
			default :break;
		}					
        
    if(Gimbal.mode == 1 )            //陀螺仪模式
		{
				pit_set.expect_remote = pit_set.expect_remote +(0x400-RC_Ctl.rc.ch3)/20;	  //pit_set.expect_remote不受模式影响
				yaw_tly_set.expect_remote = yaw_tly_set.expect_remote +(0x400-RC_Ctl.rc.ch2)/20;	
			
		}else if(Gimbal.mode == 3)       // 编码器模式  ||  超限
		{
				pit_set.expect_remote = pit_set.expect_remote +(0x400-RC_Ctl.rc.ch3)/20;	
				yaw_set.expect = yaw_set.expect +(0x400-RC_Ctl.rc.ch2)/20;
		}

	  if(RC_Ctl.rc.s1 == 1)            //上
		{    
			
				 bopan.mode = 1;             //单发              
//						 HAL_GPIO_WritePin(GPIOG,LASTER_Pin,GPIO_PIN_RESET);//激光关闭

		}
		else if(RC_Ctl.rc.s1 == 2)       //下
		{
	//		Gimbal.mode = 3;              //编码器模式
//                Shoot.mode=3;
//                if(shoot.out == 115) 
//              bopan.mode=3;    
		
				bopan.mode = 3;               //连发
//           Set_AX6(2,512,0x3ff);    //逆时针角度   150°
		}else                             //中
		{
			  bopan.mode = 0;                 //停止
//							HAL_GPIO_WritePin(GPIOG,LASTER_Pin,GPIO_PIN_SET);//激光开启
//							bopan.mode = 0;
//							Gimbal.mode = 1;     //陀螺仪模式
		    Minipc.mode = 0;
		}		
		
					
}

/***************************************************************************************
**
	*	@brief	MouseKeyControlProcess()
	*	@param
	*	@supplement	对键鼠的数据进行处理
	*	@retval	
****************************************************************************************/
void MouseKeyControlProcess()
{
		
                
                   
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

	
}





/***************************************************************************************
**
	*	@brief	MiniPC_Data_task(void const * argument)
	*	@param
	*	@supplement	视觉数据处理任务
	*	@retval	
****************************************************************************************/
void MiniPC_Data_task(void const * argument)
{
	static uint8_t frist_flag = 0;
  
  static int16_t  frist_angle;//pit的角度偏差
	minipc_rx.state_flag = 0;
	minipc_rx.angle_pit  = 0;
	minipc_rx.angle_yaw  = 0;
  uint32_t NotifyValue;
	Minipc_Pid_Init();
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
		frist_angle  =  pit_get.total_angle-pit_set.expect_pc;
	for(;;)
	{
	   NotifyValue=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    if(NotifyValue==1)
		{
			Get_MiniPC_Data();  //获取视觉传来的信息
			
			switch(Minipc.mode)
			{
					case 1 ://自瞄
					{
 
						if(my_abs(minipc_rx.angle_yaw)>640 || my_abs(minipc_rx.angle_pit)>360)  //pc数据异常
						{
							
						}else
						{
//								      pid_calc(&pid_minipc_yaw,minipc_rx.angle_yaw,0);
//                      pid_calc(&pid_minipc_pit,minipc_rx.angle_pit,0);

//                      yaw_zm_set.expect_pc = (pid_minipc_yaw.pos_out)+tly.final_angle;
//											pit_zm_set.expect_pc = (-pid_minipc_pit.pos_out)+pit_get.total_angle;

//                      yaw_zm_set.expect_pc += (-minipc_rx.angle_yaw);
//											pit_zm_set.expect_pc += (minipc_rx.angle_pit);                   
							
						  	minipc_rx.angle_yaw = 0;
						  	minipc_rx.angle_pit = 0;
//                      
//                      if(yaw_zm_set.expect_pc>500)yaw_zm_set.expect_pc=500;
//                      else if(yaw_zm_set.expect_pc<-500)yaw_zm_set.expect_pc=-500;
//                      
//                      if(pit_zm_set.expect_pc>500)pit_zm_set.expect_pc=500;
//                      else if(yaw_zm_set.expect_pc<-500)yaw_zm_set.expect_pc=-500;
							
							/*  调准心测试使用
							Shoot.mode = 3; //先启动抹茶轮 设置不同的速度
							if( shoot.out == 112 ) bopan.mode  = 1;
							*/
						}
					}break ;
					
					case 2 ://打符						
					{
						
						if(my_abs(minipc_rx.angle_yaw)>500 || my_abs(minipc_rx.angle_yaw)>500)  //pc数据异常
						{
						}else
						{
								Shoot.mode = 1; //先启动抹茶轮
								if( frist_flag == 0)
								{
										if(shoot.out == 128 )  frist_flag=1;
								
										yaw_set.expect_pc = minipc_rx.angle_yaw;
										pit_set.expect_pc = minipc_rx.angle_pit;
								}
							
								if(minipc_rx.state_flag)//检测到不同装甲块时立即调准位置
								{
										yaw_set.expect_pc = minipc_rx.angle_yaw;
										pit_set.expect_pc = minipc_rx.angle_pit;
								}
								else if( frist_flag )//误差不能设置的太大
								{
										yaw_set.expect_pc = minipc_rx.angle_yaw ;               
										pit_set.expect_pc = minipc_rx.angle_pit ;
										bopan.mode = 1;
								}else     
										bopan.mode = 0;
								
						 }break;
					 }
					 default : break;
				}
			 
		}

	  osDelayUntil(&xLastWakeTime, MINIPC_PERIOD);
  }
}
