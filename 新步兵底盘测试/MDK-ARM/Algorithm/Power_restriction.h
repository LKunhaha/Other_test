#ifndef __POWER_RESTRICTION_H
#define __POWER_RESTRICTION_H
/* ????? ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "String.h"
#include "gpio.h"
#include "usart.h"
#include "protocol.h"

/* ???????????? --------------------------------------------------*/
//?????
#define Switch1_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_SET);	//????
#define Switch2_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_SET);//????
#define Switch3_On HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_SET);	//??????????

#define	Switch1_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch1_Pin,GPIO_PIN_RESET);
#define	Switch2_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch2_Pin,GPIO_PIN_RESET);
#define	Switch3_Off HAL_GPIO_WritePin(Switch_GPIO_Port,Switch3_Pin,GPIO_PIN_RESET);

/* ??????????????? --------------------------------------------*/
typedef struct{
	
 //????
 float  CurrentCalculat;			 			//??????
 float  Current_Offset;						  //?????(???????)
 float  Current_Referee;						//??????????
 int16_t Current_Offset_num; 
 //????
 float Capacitance_Volt;						//????
 //????
 float	Chassis_Current;						//??????

}Current_GET;   //????

typedef struct{
		
 float  Volt_Referee;								//??????????
 float  Power_Referee;							//?????????
 float  Power_Referee_Last;         
 float  Power_Calculat;							//????????
 float  Power_Chassis_Calculat;			//????????????
 float  PowerRemain_Referee;				//???????????
 float  PowerRemain_Referee_last;		
 float  PowerRemain_Calculat;				//??????????????
 float  PowerRemain_Calculat_Last;
 float  PowerRemain_Calculat_Next;
 float  PowerLimit;									//?????
 
}Limit;				 //????

typedef struct{

	uint32_t time_now;
	uint32_t time_last;
	int32_t  time;
	int32_t  total_time_ms;
	double  total_time_s;
	
}MyTimeTick;
/* ??????????????? --------------------------------------------*/
extern  uint32_t  uhADC1ConvertedValue[10];  //ADC1????
extern  uint32_t  uhADC2ConvertedValue[10];  //ADC2????
extern  uint32_t  uhADC3ConvertedValue[10];  //ADC3????

extern  Current_GET  current_get;
extern  Limit  limit;
extern  MyTimeTick  time_for_limit;
/* ????????????????? ----------------------------------------*/
void power_limit(float  Current_get[4]);
void Super_Capacitance(float  Current_get[4]);

float Limit_filter(float oldData,float newData,float val);
float LPF_1st(float oldData, float newData, float lpf_factor);

#define Only_Soft_Limit 1
#endif

