#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H
/* 包含头文件----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "pid.h"
#include "communication.h "
#include "Motor_USE_CAN.h"
#include "chassis_control.h"
#include "atom_imu.h"
#include "decode.h"
#include "gimbal_control.h"

/* 本模块向外部提供的宏定义--------------------------------------------------*/	 
#define GIMBAL_PERIOD 5


/* 本模块向外部提供的数据类型定义--------------------------------------------*/
typedef struct{
		//int16_t expect;
		int32_t expect;
		uint8_t	step;
		uint8_t mode;
		int32_t expect_pc;
		int32_t expect_remote;
		int32_t expect_remote_last;
    int16_t err;
} Pos_Set;

typedef struct {
	
	uint8_t mode;
	uint8_t flag;
  uint8_t shoot_flg;
  uint8_t stop_flg;
  uint8_t last_shoot_flg;
	uint16_t last_cnt;
	uint16_t cnt;
	uint16_t danfa_cnt;
	uint8_t last_mode;
  uint8_t last_flag;
	float val;
  uint16_t heat_forecast;
  uint8_t  color;
  uint8_t   shexiang;
  }Mode_Set;

/* 本模块向外部提供的宏定义--------------------------------------------------*/

/* 本模块向外部提供的接口常量声明--------------------------------------------*/

extern Pos_Set  yaw_tly_set;
extern Pos_Set  pit_set;
extern Pos_Set  yaw_set;
extern int8_t gimbal_disable_flg;
extern Mode_Set Gimbal;
extern Mode_Set Minipc;
extern Mode_Set Shoot;
extern Mode_Set bopan;	
extern Mode_Set Shoot_heat;
extern Mode_Set Shoot_mouse;
/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Gimbal_Task(void const * argument);

/* 全局配置区----------------------------------------------------------------*/

#endif
