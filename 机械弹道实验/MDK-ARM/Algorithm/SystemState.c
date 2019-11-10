#include "SystemState.h"
#include "tim.h"
#include "gimbal_control.h"
/*   by  donglin   */


SystemStateDef SystemState = {0};
float g_TimePer[100] = {0};
float g_Time_DeviceOutLine[DeviceTotal_No] = {0};//外设最近一次通信时间数组
float g_Time_TASKOutLine[TASKTotal_No] = {0};//外设最近一次通信时间数组
static float Time;
static float Shangdan_time;

//断线检测检测
void OutLine_Check()
{
	short num = 0;//临时变量累加用
	float time = GetSystemTimer();//当前系统时间

	for(num = 0; num < DeviceTotal_No; num++)
	{
		if(time - g_Time_DeviceOutLine[num] > OutLine_Time)
		{
			MyFlagSet(SystemState.OutLine_Flag,(num));//设置断线标志
		}
		else
		{
			MyFlagClear(SystemState.OutLine_Flag,(num));//清除断线标志
		}
	}
}

//断线检测检测
void TASK_Check()
{
	short num = 0;//临时变量累加用
	float time = GetSystemTimer();//当前系统时间

	for(num = 0; num < TASKTotal_No; num++)
	{
		if(time - g_Time_TASKOutLine[num] > OutLine_Time)
		{
			MyFlagSet(SystemState.task_OutLine_Flag,(num));//设置断线标志
		}
		else
		{
			MyFlagClear(SystemState.task_OutLine_Flag,(num));//清除断线标志
		}
	}
}


//=====================================================
//							  内部函数
//
//====================================================





int SystemState_Inite()
{
	int state;
	SystemState.Enable = 0;
	SystemState.State = 0;
	SystemState.Task = 0;
	SystemState.Time = 0;
	SystemState.htim = &htim6;//计时器请设定 每 10us 记一个数  重载值为 100-1 (1ms)  例如 Timer3 主频168M 预分频 (840-1) 重载值 (100-1)
//	state=HAL_TIM_Base_Start_IT(SystemState.htim);//启动时间计数器
  return state;
}

//中断刷新中调用 更新系统时间 ms
 void RefreshSysTime(void)
{
		SystemState.Time += 1;
}


//获得系统时间
inline float GetSystemTimer()
{
	return SystemState.htim->Instance->CNT / 100.0 + SystemState.Time;   //经计算，得到的最终时间单位为ms
}


//刷新外设通信时间时间数组
void RefreshDeviceOutLineTime(DeviceX_NoDEF DevX_No)
{
	
	g_Time_DeviceOutLine[DevX_No] = GetSystemTimer();
	
}



//刷新任务时间数组
void RefreshTaskOutLineTime(TASK_NoDEF Task_No)
{
	
	g_Time_TASKOutLine[Task_No] = GetSystemTimer();
	
}

void xianwei_check()     //限位检测（放在掉线检测任务里）
{
		float time = GetSystemTimer();//当前系统时间
  
	  if(xianwei_flg)   //限位标志
		{
        Shoot.shoot_flg = 1;                  // 压限位开关  为高电平 
        Shangdan_time = GetSystemTimer();
		}else
		{
			  Shoot.shoot_flg = 0;                  //未压限位开关  或  过限位开关  
		}
		
		if(Shoot.last_shoot_flg != Shoot.shoot_flg) 
		{
				Shoot.flag = 1;           //弹丸从   未压限位开关 至 压限位开关   或   压限位开关 至 过限位开关
				Shoot.cnt += 1;            //发射一颗弹丸加2(两次电平变化)
				
		}else  Shoot.flag = 0;         //弹丸位置无变化
					        
    if(time - Shangdan_time > shangdan_time  )//上弹检测（未压限位开关持续时间大于10秒就进入上弹模式）
		{
			  Shoot.flag = 2;
		}
     
    
    Shoot.last_shoot_flg = Shoot.shoot_flg;
}
