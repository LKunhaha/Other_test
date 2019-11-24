#ifndef __mecanum_calc_H
#define __mecanum_calc_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f4xx_hal.h"
#include "stdlib.h"
#include "stm32f4xx_hal.h"
#include "arm_math.h"
#include "user_lib.h"
   

#define Middle_angle  5850   

typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

typedef int8_t 		s8;
typedef int16_t 	s16;
typedef int32_t		s32; 
typedef struct
{
		float vx,vy,vw;
} mecanum_t;

typedef struct
{
    float           hvx;     // forward/back
    float           hvy;     // left/right
    float           vw;      // rotate
    int16_t         wheel_speed[4];
} chassis_t;


/* Extern  ------------------------------------------------------------------*/
void motor_move_setvmmps(float  wheel[4],float dstVmmps_X,float dstVmmps_Y,float dstVmmps_W);
void Rotate_Component_clac( fp32 X_speed , fp32 Y_speed , float X_speed_out , float Y_speed_out ,float yaw_angle );

extern chassis_t chassis;
extern mecanum_t mecanum;

#ifdef __cplusplus
}
#endif
#endif 
