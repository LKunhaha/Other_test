#ifndef __GUN_CONTROL_H
#define __GUN_CONTROL_H
/* 包含头文件----------------------------------------------------------------*/
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "Motor_USE_CAN.h"
#include "Motor_USE_TIM.h"
#include "pid.h"
#include "minipc.h"

/* 本模块向外部提供的数据类型定义--------------------------------------------*/
typedef struct Heat_Gun_t
{
	int16_t  shted_bullet;
	int16_t limt_bullet;
	int16_t last_limt_bullet;
	uint16_t limt_heat;
	uint16_t rel_heat;
	uint16_t last_rel_heat;
	float    remain_power;
	uint8_t  limt_spd;
	uint8_t  roboLevel;
	uint8_t  sht_flg;
	uint8_t  stop_flg;
	uint8_t  heat_down_flg;
}Heat_Gun_t;
volatile typedef struct 
{
	volatile uint16_t rel_heat;
	volatile float remain_power;
}Power_Heat;

/* 本模块向外部提供的宏定义--------------------------------------------------*/
#define GUN_PERIOD  10
#define Mocha_PERIOD  200
#define BLOCK_TIME 2000
#define REVERSE_TIME 2000
#define DELAY_TIME  1000

#define Limit_switch_Control 0      //0为无限位开关控制，1为有限位开关控制

#define Mocha_Stop         0
#define Mocha_danfa_mode   1
#define Mocha_Duofa_mode   2
#define Mocha_Lianfa_mode  3

#define bopan_Stop          0
#define bopan_danfa_mode    1
#define bopan_Duofa_mode    2
#define bopan_Lianfa_mode   3
#define bopan_shangdan_mode 4
#define bopan_guodu_mode    5
#define bopan_fanzhuan_mode 10
/* 本模块向外部提供的接口常量声明--------------------------------------------*/
extern volatile Power_Heat * ptr_power_heat;
extern Heat_Gun_t  ptr_heat_gun_t;
extern volatile float remain_power;

/* 本模块向外部提供的接口函数原型声明----------------------------------------*/
void Gun_Task(void const * argument);
void Mocha_Task(void const * argument);
void Shoot_Heat_Task(void const *argument);
void bopan_check( uint8_t  Set_cnt );
void Moto_dial_Angle_Reset(void);
/* 全局配置区----------------------------------------------------------------*/

#endif
