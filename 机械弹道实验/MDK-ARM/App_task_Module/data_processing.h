#ifndef  __DATA_PROCESSING_H
#define  __DATA_PROCESSING_H
#ifdef __cplusplus
 extern "C" {
#endif
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "pid.h"
#include "string.h"
#include "minipc.h"
#include "protocol.h"
#include "gun_control.h"
#include "gimbal_control.h"
#include "chassis_control.h"
#include "communication.h"
#include "Power_restriction.h"	 
 
/* 本模块向外部提供的宏定义--------------------------------------------------*/	 
#define  NORMAL_SPEED_MAX 	1000
#define  NORMAL_SPEED_MIN  -1000
#define  ACC_SPEED    30 
#define  DEC_SPEED    300 
#define  press_times  20
#define REMOTE_PERIOD 1       //遥控器的接收机数据发送周期为7ms
#define MINIPC_PERIOD 10
	 
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\
	 

//按键偏移量宏定义
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

/* 本模块向外部提供的接口常量声明--------------------------------------------*/	
extern float power; 				 //底盘功率 _测试
extern int8_t chassis_gimble_Mode_flg;
extern float chassis_Current; 
extern float	 chassis_Volt; 
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/		 

void Remote_Data_Task(void const * argument); 
void MiniPC_Data_task(void const * argument);
void RemoteControlProcess(void);
void MouseKeyControlProcess(void);
void hard_brak(void);
void Remote_Ctrl(void);
void Minipc_Pid_Init(void);

/* 全局配置区----------------------------------------------------------------*/
	
#ifdef __cplusplus
}
#endif
#endif /*__ usart_task_H */

