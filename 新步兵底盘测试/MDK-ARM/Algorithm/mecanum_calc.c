/*******************************************************************************
                      版权所有 (C), 2017-,NCUROBOT
 *******************************************************************************
  文 件 名   : communication.c
  版 本 号   : 初稿
  作    者   : NCUERM
  生成日期   : 2018年7月
  最近修改   :
  功能描述   : 麦轮结算
  函数列表   :void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,
																			float dstVmmps_Y,float dstVmmps_W)

*******************************************************************************/

/* 包含头文件 ----------------------------------------------------------------*/
#include "mecanum_calc.h"
/* 内部自定义数据类型 --------------------------------------------------------*/
/* 内部宏定义 ----------------------------------------------------------------*/
#define MyAbs(x) ((x > 0) ? (x) : (-x))
#define Front_ratio  1.0f
#define Brak_ratio   1.0f

/* 任务相关信息定义-----------------------------------------------------------*/

/* 内部常量定义---------------------------------------------------------------*/

/* 外部变量声明 --------------------------------------------------------------*/

/* 外部函数原型声明 ----------------------------------------------------------*/

/* 内部变量 ------------------------------------------------------------------*/

/* 函数原型声明 ----------------------------------------------------------*/


void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W)
{
     //云台靠后，加入比例调准旋转中心
			wheel[0] = -(-dstVmmps_X + dstVmmps_Y + dstVmmps_W*Front_ratio);
			wheel[1] = -(-(dstVmmps_X + dstVmmps_Y - dstVmmps_W*Front_ratio));
			wheel[2] = -(-(-dstVmmps_X + dstVmmps_Y - dstVmmps_W*Brak_ratio));
			wheel[3] = -(dstVmmps_X + dstVmmps_Y + dstVmmps_W*Brak_ratio);	
}


void Rotate_Component_clac( fp32 X_speed , fp32 Y_speed , float X_speed_out , float Y_speed_out ,fp32 yaw_angle )
{

//  if(yaw_angle<0)
//  {
//         if(X_speed != 0)
//         {
//                X_speed_out = X_speed;
//                Y_speed_out = X_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//         }else if(Y_speed != 0)
//          {
//                Y_speed_out = Y_speed;
//                X_speed_out = -Y_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//          }
//  }else if(yaw_angle>0)
//  {
//         if(X_speed != 0)
//         {
//                X_speed_out = X_speed;
//                Y_speed_out = -X_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//         }else if(Y_speed != 0)
//          {
//                Y_speed_out = Y_speed;
//                X_speed_out = Y_speed*arm_sin_f32(yaw_angle*radian_ratio)/arm_cos_f32(yaw_angle*radian_ratio);
//          }
//  }else 
//  {
//         X_speed_out = X_speed;
//         Y_speed_out = Y_speed;
//  }
  
}












