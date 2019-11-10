/* 包含头文件----------------------------------------------------------------*/
#include "gun_control.h"
#include "math.h"
#include "SystemState.h"
#include "user_lib.h"
#include "gimbal_control.h"

/* 内部变量------------------------------------------------------------------*/

pid_t pid_dial_pos  = {0};  //拨盘电机位置环
pid_t pid_dial_spd  = {0};	//拨盘电机速度环

Heat_Gun_t  ptr_heat_gun_t;
Mode_Set bopan = {0};	
Mode_Set Shoot_heat = {0};
ramp_function_source_t shoot;
extern uint8_t shot_frequency;
uint8_t finish_flag = 0;
static uint8_t Delay_flg;
uint8_t Guodu_or_Stop_mode_flag = 0;   //过渡模式或停止模式的标志位(用于判断是否第一次进入这两种模式)
uint8_t Reverse_mode_flag = 0;         //反转模式的标志位(用于判断是否第一次进入这种模式)
int Lianfa_speed;
/* 内部函数原型声明----------------------------------------------------------*/
void Gun_Pid_Init()
{
		PID_struct_init(&pid_dial_pos, POSITION_PID, 6000, 5000, 0.2f, 0.0000f,	2.0f);  
		PID_struct_init(&pid_dial_spd, POSITION_PID, 6000, 5000, 1.5f,	0.1f,	0.0f	);  
}

/* 任务主体部分 -------------------------------------------------------------*/

/*********调试初始参数设定***************/
#define Test_mode  bopan_danfa_mode          //拨盘模式初设定，可设：bopan_danfa_mode、bopan_Lianfa_mode
#define Lianfa_speed  3000                   //连发模式下的的速度大小，设定值可设为：0—6000
#define	shoot_value   120                    //摩擦轮转速设定，设定值可设为：110—125
/****************************************/


/***************************************************************************************
**
	*	@brief	void Gun_Task(void const * argument)
	*	@param
	*	@supplement	发射任务
	*	@retval	
****************************************************************************************/
void Gun_Task(void const * argument)
{ 
	osDelay(2000);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	Gun_Pid_Init();
	
  static int32_t set_angle = 0;
	int32_t set_speed = 0;
	static uint8_t set_cnt = 0;
  /*设定发弹*/
  bopan.flag=1;
	
	
	static float Guodu_Start_time;
	static float Reverse_Start_time;
	
	
	
	/*************************      模式说明      ******************************
	未压限位开关    :  弹丸还在弹道中，但未到达限位开关
	压限位开关      :  弹丸到达限位开关，限位开关接通
	过限位开关      :  弹丸通过了限位开关，即将经过摩擦轮发射出去
	
	Limit_switch_Control : 是否采用限位开关控制
	
	Shoot_mouse.mode：0  
										1  Shoot.mode = Mocha_danfa_mode  , bopan.mode = bopan_danfa_mode(摩擦轮到达指定转速后设定模式)
										2  Shoot.mode = Mocha_Duofa_mode  , bopan.mode = bopan_Duofa_mode(摩擦轮到达指定转速后设定模式)
										3  Shoot.mode = Mocha_Lianfa_mode , bopan.mode = bopan_Lianfa_mode(摩擦轮到达指定转速后设定模式)
										4  Shoot.mode = Mocha_Duofa_mode  , bopan.mode = bopan_Lianfa_mode(摩擦轮到达指定转速后设定模式)
										
	bopan.mode      :  bopan_Stop           0       
										 bopan_danfa_mode     1           Shoot.mode == 1             ***基本模式
										 bopan_Duofa_mode     2           Shoot.mode == 2             ***基本模式
										 bopan_Lianfa_mode    3           Shoot.mode == 3             ***基本模式
										 bopan_shangdan_mode  4           上弹，指将弹丸送入限位开关处( 压限位开关  表示到达指定位置)
										 bopan_guodu_mode     5           过渡模式，停转一小会
										 bopan_fanzhuan_mode  10
										 2
  bopan.flag      : 0   反转标志位
	                  1
	
	Shoot.flag      : 0   限位开关电平未变化
(限位开关电平变化)  1   限位开关电平发生变化
										2   上弹标志
										
  Shoot.shoot_flg : 0   未压限位开关  或  过限位开关
(限位开关电平)      1   压限位开关 
	
	Delay_flg       :
	****************************************************************************/
	for(;;)
	{
		
			RefreshTaskOutLineTime(GunTask_ON);

		  moto_dial_get.cmd_time = GetSystemTimer();    //获取任务实时时间
		
	/*********调试初始参数设定***************/
	 bopan.mode = Test_mode;      //拨盘模式初设定，可设：bopan_danfa_mode、bopan_Lianfa_mode
	/****************************************/
		
			if(Limit_switch_Control)                      //是否采用限位开关控制
			{    
					 if(Shoot.flag == 2 )  
					 {
							bopan.mode = bopan_shangdan_mode;
							Delay_flg = 0;
					 }else if(Shoot.shoot_flg == 1 && bopan.mode == bopan_shangdan_mode)      // 压限位开关 && 上弹模式
							bopan.mode = bopan_Stop;
					 else if(Shoot.flag == 1 && bopan.mode == bopan_danfa_mode && Shoot.cnt == Shoot.danfa_cnt + 2) //预置时已经加了一
							Delay_flg = 1;        //限位开关电平变化 && 单发模式 && 弹丸刚通过限位开关 ，进入过渡模式
					 else if(Shoot.flag == 1 && bopan.mode == bopan_Duofa_mode && Shoot.cnt == Shoot.last_cnt + 6 ) 
							Delay_flg = 1;        //限位开关电平变化 && 多发模式 && 3颗弹丸刚通过限位开关 ，先暂停一段时间 
					 else
					 {        
							Delay_flg = 0;
					 }                       
			}
			
		 if(Delay_flg || ( moto_dial_get.cmd_time - Guodu_Start_time < DELAY_TIME ))  //进入过渡模式后需暂停一段时间    
		 {
				 bopan.mode = bopan_guodu_mode;
			   moto_dial_get.run_time = GetSystemTimer();                 //获取拨盘开始转动的时间
		 }
	 
		 bopan_check(set_cnt);//卡单检测	

    /*判断发射模式*/
   switch(bopan.mode)
   {
			case bopan_Stop://停止
			{
				  if(Guodu_or_Stop_mode_flag == 0)  //表明首次进入停止模式
					{
						  Moto_dial_Angle_Reset();
						  set_cnt = 0;
						  set_angle = moto_dial_get.total_angle;
						
						  Guodu_or_Stop_mode_flag = 1;
					}else
					{
					    set_cnt = 0;
					}
					
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
			}break;
			
      case bopan_danfa_mode://按键单发模式
      {
					set_cnt = 1;    
					set_angle = -36860 * set_cnt - 300;  //一个弹丸槽角度  8191*36/8    2006电机减速比为36    500是为了提高转矩而加的增幅
				
					/*pid位置环*/
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
				
          Guodu_or_Stop_mode_flag = 0;
				  Reverse_mode_flag = 0;
      }break;
			
      case bopan_Duofa_mode://固定角度模式   三连发
      {
					set_cnt = 3;
					set_angle = -36860 * set_cnt - 500;
				
					/*pid位置环*/
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
					set_speed = pid_dial_pos.pos_out;
					
					Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
			
      case bopan_Lianfa_mode://连发模式
      { 
					set_speed = -Lianfa_speed;
					set_cnt = 1;
					
				  Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
			
			case bopan_shangdan_mode://上弹
      { 
					moto_dial_get.cmd_time = GetSystemTimer();
					set_speed = -500;
					set_cnt = 0;
				
				  Guodu_or_Stop_mode_flag = 0;
					Reverse_mode_flag = 0;
      }break;
      
      case bopan_guodu_mode://过渡
      { 
				  if(Guodu_or_Stop_mode_flag == 0)  //表明首次进入过渡模式
					{
						  Moto_dial_Angle_Reset();
						  set_cnt = 0;
						  set_angle = moto_dial_get.total_angle;
						
						  Guodu_or_Stop_mode_flag = 1;
						  Guodu_Start_time = GetSystemTimer();  //获取过渡模式开始时的时间
					}else
					{
							Delay_flg = 0;
							set_cnt = 0;
					}
				
					pid_calc(&pid_dial_pos, moto_dial_get.total_angle,set_angle);	
			    set_speed = pid_dial_pos.pos_out;
					
      }break;
      
			case bopan_fanzhuan_mode://反转
			{
				if(Reverse_mode_flag == 0)        //表明首次进入反转模式
				{
					  set_cnt = 0;
						Reverse_Start_time = GetSystemTimer();	 //获取反转模式开始的时间
						Reverse_mode_flag = 1;
				}else
				{
						if( moto_dial_get.cmd_time - Reverse_Start_time > REVERSE_TIME )//反转设定    
						{
							Moto_dial_Angle_Reset();
							bopan.flag = 1;
							set_cnt = 0;
						}else 
						{			
							set_speed = 1000;
						}
				}
			}break;
			default :break;
    }
     /*速度环*/
     pid_calc(&pid_dial_spd,moto_dial_get.speed_rpm ,set_speed);
     /*驱动拨弹电机*/
		 Allocate_Motor(&hcan1,pid_dial_spd.pos_out);
 
		if(bopan.last_mode != Mocha_Duofa_mode && bopan.mode == Mocha_Duofa_mode)  
			 Shoot.last_cnt = Shoot.cnt;		 //模式切换为单发或多发时，保存此时弹丸计数值(每颗弹丸计数值为2)，
		else if(bopan.last_mode != Mocha_danfa_mode && bopan.mode == Mocha_danfa_mode)  
			 Shoot.danfa_cnt = Shoot.cnt;
		
		bopan.last_mode = bopan.mode; 

    osDelayUntil(&xLastWakeTime,GUN_PERIOD);

	 }
}


/***************************************************************************************
**
	*	@brief	void Mocha_Task(void const *argument)
	*	@param
	*	@supplement	   
	*	@retval	
****************************************************************************************/
void Mocha_Task(void const *argument)  //调试拨盘中关闭了此任务，使得摩擦轮速度不变
{
	osDelay(100);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	ramp_init(&shoot,0.01,150,100);//摩擦轮斜坡           0.01为斜率  150和100为最大、最小值(最大值也为目标值)
	
	for(;;)  //目前用来调整摩擦轮的最终转速
	{
		shoot.max_value = shoot_value;																	
		ramp_calc(&shoot,100);  //100为增长系数
		Friction_Wheel_Motor(shoot.out,shoot.out);
	
	 osDelayUntil(&xLastWakeTime,Mocha_PERIOD);
		
	}
}



/***************************************************************************************
**
	*	@brief	void Shoot_Heat_Task(void const *argument)
	*	@param
	*	@supplement	   
	*	@retval	
****************************************************************************************/
void Shoot_Heat_Task(void const *argument)
{
  
  osDelay(500);
	portTickType xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  
  
  for(;;)
  {
    
    Shoot_heat.val = Robot.heat.shoot_17_cooling_limit - Robot.heat.shoot_17_heat;//剩余热量
    
    if(Shoot_heat.val<Shoot_heat.heat_forecast)
    {
      Shoot_heat.flag = 0;
    }else Shoot_heat.flag = 1;
    
    osDelayUntil(&xLastWakeTime,10);
  }
   
}



/***************************************************************************************
**
	*	@brief	void bopan_check( uint8_t  Set_cnt )
	*	@param
	*	@supplement	  拨盘堵转检测
	*	@retval	
****************************************************************************************/
void bopan_check( uint8_t  Set_cnt )
{
	
	   	if(bopan.flag  == 1)
			{
		     /*判断拨盘是否转到位置*/			
					if(my_abs(moto_dial_get.total_angle) >= 36859 * Set_cnt)      //转动一个弹丸，转子需转36/8 = 4.5 圈
					{
							if((bopan.mode == bopan_danfa_mode) && ( Set_cnt != 0) && (!Limit_switch_Control))
							{ 
								 Delay_flg = 1;
							}
							
							if((bopan.mode != bopan_guodu_mode) && (bopan.mode != bopan_Stop))   //这两种模式下因为Set_cnt为0，所以不对其进行频繁清零
							{
								 Moto_dial_Angle_Reset();
						  }
							
							moto_dial_get.run_time = GetSystemTimer();                 //获取拨盘开始转动的时间
							bopan.flag = 1;
					}
					else   //刚进入单发、多发、连发，或上弹模式  才会进入此判断
					{
							if( my_abs(moto_dial_get.cmd_time - moto_dial_get.run_time) > BLOCK_TIME )//堵转判定    
							{
									bopan.flag = 0;	
							}
					}
			}
								
			if(bopan.flag == 0)   
				bopan.mode = bopan_fanzhuan_mode;//反转
	
}


/***************************************************************************************
**
	*	@brief	void Moto_dial_Angle_Reset(void)
	*	@param
	*	@supplement	  拨盘角度清零
	*	@retval	
****************************************************************************************/
void Moto_dial_Angle_Reset(void)
{
		moto_dial_get.round_cnt = 0;            
		moto_dial_get.offset_angle = moto_dial_get.angle;
		moto_dial_get.total_angle = 0;
}
