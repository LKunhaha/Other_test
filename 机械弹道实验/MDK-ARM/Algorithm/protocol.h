#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "USART.h"
#include "String.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define UP_REG_ID    0xA0  //up layer regional id
#define DN_REG_ID    0xA5  //down layer regional id
#define HEADER_LEN   sizeof(frame_header_t)
#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes

#define PROTOCAL_FRAME_MAX_SIZE  200

/** 
  * @brief  frame header structure definition
  */
//ÄÚ²¿±äÁ¿ÉùÃ÷
typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
//Íâ²¿±äÁ¿ÉùÃ÷

typedef __packed struct
{
	uint8_t SOF;          //Êı¾İÆğÊ¼×Ö½Ú£¬¹Ì¶¨Îª0xA5          
	uint16_t DataLength;  //Êı¾İ³¤¶È
	uint8_t Seq;          //°üĞòºÅ
	uint8_t CRC8;         //Ö¡Í·CRCĞ£Ñé
}tFrameHeader;//Ö¡Í·

typedef enum                 //Ã¶¾ÙÀàĞÍ£¬ÃüÁîid_±ä¸ü
{
	game_state = 0x0001,      
	game_result = 0x0002,            
	game_robot_survivors = 0x0003,                
	event_data = 0x0101,							
	supply_projectile_action = 0x0102,								
	game_robot_state = 0x0201,								
	power_heat_data = 0x0202,					
	game_robot_pos = 0x0203,						
	buff_musk =0x0204,  
  robot_energy=0x0205,
  robot_hurt=0x0206,
  shoot_data=0x0207,
  student_interactive_header_data = 0x0301,
  custom_data = 0x0301,
  interactive_data = 0x0301,
	Wrong = 0x1301       
}tCmdID; 

typedef __packed struct
{
  uint8_t game_type : 4;   //±ÈÈüÀàĞÍ
  uint8_t game_progress : 4;//±ÈÈü½×¶Î
  uint16_t stage_remain_time;//µ±Ç°½×¶ÎÊ£ÓàÊ±¼ä
}ext_game_state_t; //±ÈÈü»úÆ÷ÈË×´Ì¬£¨0x0001£©

typedef __packed struct
{

	uint8_t winner ;
	
}ext_game_result_t;   //±ÈÈü½á¹ûÊı¾İ£¨0x0002£©

typedef __packed struct
{
	uint16_t robot_legion;
	
}ext_game_robot_survivors_t;   //»úÆ÷ÈË´æ»îÊı¾İ£¨0x0003£©

typedef __packed struct
{

	uint32_t event_type;
	
} ext_event_data_t;  //³¡µØÊÂ¼şÊı¾İ£¨0x0101£©


typedef __packed struct
{
	
	uint8_t supply_projectile_id; 
  uint8_t supply_robot_id; 
  uint8_t supply_projectile_step; 
  uint8_t supply_projectile_num;
  
}ext_supply_projectile_action_t;							//³¡µØ²¹¸øÕ¾¶¯×÷Ê¶±ğÊı¾İ£¨0x0102£©

//typedef __packed struct
//{
//	
//	uint8_t supply_projectile_id;
//  uint8_t supply_robot_id; 
//  uint8_t supply_num;

//} ext_supply_projectile_booking_t;					//ÇëÇó²¹¸øÕ¾²¹µ°(0x0103)

typedef __packed struct
{
	
 uint8_t robot_id;
 uint8_t robot_level;
 uint16_t remain_HP;
 uint16_t max_HP;
 uint16_t shooter_heat0_cooling_rate;
 uint16_t shooter_heat0_cooling_limit;
 uint16_t shooter_heat1_cooling_rate;
 uint16_t shooter_heat1_cooling_limit;
 uint8_t mains_power_gimbal_output : 1;
 uint8_t mains_power_chassis_output : 1;
 uint8_t mains_power_shooter_output : 1;

} ext_game_robot_state_t;				//±ÈÈü»úÆ÷ÈË×´Ì¬(x00201)

typedef __packed struct
{
	
 uint16_t chassis_volt; 
 uint16_t chassis_current; 
 float chassis_power; 
 uint16_t chassis_power_buffer; 
 uint16_t shooter_heat0; 
 uint16_t shooter_heat1; 
	
} ext_power_heat_data_t;						//ÊµÊ±¹¦ÂÊÈÈÁ¿Êı¾İ(0x0202)

typedef __packed struct
{
	float x;
 float y;
 float z;
 float yaw;
} ext_game_robot_pos_t;                     //»úÆ÷ÈËÎ»ÖÃ(0x0203)

typedef __packed struct
{
 uint8_t power_rune_buff;
}ext_buff_musk_t;                      //»úÆ÷ÈËÔöÒæ(x00204)

typedef __packed struct
{
  uint8_t energy_point;
  uint8_t attack_time;
} aerial_robot_energy_t;              //¿ÕÖĞ»úÆ÷ÈËÄÜÁ¿×´Ì¬(0x0205)

typedef __packed struct
{
 uint8_t armor_id : 4;
 uint8_t hurt_type : 4;
} ext_robot_hurt_t;                  //ÉËº¦×´Ì¬(0x0206)

typedef __packed struct
{
 uint8_t bullet_type;
 uint8_t bullet_freq;
 float bullet_speed;
} ext_shoot_data_t;                    //ÊµÊ±Éä»÷ĞÅÏ¢(0x0207)

typedef __packed struct
{
 uint16_t data_cmd_id;
 uint16_t send_ID;
 uint16_t receiver_ID;
}ext_student_interactive_header_data_t;    //½»»¥Êı¾İ½ÓÊÕĞÅÏ¢(0x0301)


typedef __packed struct
{
float data1;
float data2;
float data3;
uint8_t masks;
} client_custom_data_t;                    //¿Í»§¶Ë×Ô¶¨ÒåÊı¾İ(0x0301)

typedef __packed struct
{
   uint8_t data[10];
} robot_interactive_data_t;                 //½»»¥Êı¾İ


typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	__packed union 
	{
		ext_game_state_t    			game_state;  				  //
		ext_game_result_t  		    game_result;          //
		ext_game_robot_survivors_t     		game_robot_survivors;          //
		ext_event_data_t			            event_data;				//
		ext_supply_projectile_action_t		supply_projectile_action;						//
		ext_game_robot_state_t		game_robot_state;			//
		ext_power_heat_data_t			power_heat_data;				//
		ext_game_robot_pos_t       game_robot_pos; 		//    
    ext_buff_musk_t            buff_musk;
    aerial_robot_energy_t      robot_energy;
    ext_robot_hurt_t           robot_hurt;
    ext_shoot_data_t           shoot_data;
    ext_student_interactive_header_data_t student_interactive_header_data;
    client_custom_data_t       custom_data;
    robot_interactive_data_t   interactive_data;
	}Data;
	uint16_t        CRC16;         //Ö®Ç°ËùÓĞÊı¾İCRCĞ£Ñé   ×¢Òâ´ËÊı¾İºÍÖ®Ç°µÄÊı¾İ¿ÉÄÜ²»Á¬Ğø£¬ËùÒÔ²»ÒªÖ±½ÓÊ¹ÓÃ£¬ÈôĞèÒªÖ±½ÓÊ¹ÓÃ£¬±ØĞëÔÚ´Ë¸³Öµ
}tFrame;  //Êı¾İÖ¡


//typedef __packed struct
//{
//	tFrameHeader    FrameHeader;
//	tCmdID          CmdID;
//  tSelfDefine     SelfDefine;
//	uint16_t        CRC16;         //Ö®Ç°ËùÓĞÊı¾İCRCĞ£Ñé   ×¢Òâ´ËÊı¾İºÍÖ®Ç°µÄÊı¾İ¿ÉÄÜ²»Á¬Ğø£¬ËùÒÔ²»ÒªÖ±½ÓÊ¹ÓÃ£¬ÈôĞèÒªÖ±½ÓÊ¹ÓÃ£¬±ØĞëÔÚ´Ë¸³Öµ
//}tFrame;  //Êı¾İÖ¡


typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_state_t    			game_state;    
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}game_state_tFrame;  //
typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_result_t  		    game_result;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}game_resultFrame; //©
typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_robot_survivors_t     		game_robot_survivors;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}game_robot_survivorsFrame;    //

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_event_data_t	  event_data;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}event_dataFrame;   //   

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_supply_projectile_action_t		supply_projectile_action;	   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}supply_projectile_actionFrame;			//

//typedef __packed struct
//{
//	tFrameHeader    FrameHeader;
//	tCmdID          CmdID;
//	ext_supply_projectile_booking_t		supply_projectile_booking;   
//	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
//}supply_projectile_bookingFrame;								//±ÈÈü½á¹ûÊı¾İ(0x006)

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_robot_state_t		game_robot_state;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}game_robot_stateFrame;					//buff×´Ì¬ÈÎÒâbuff×´Ì¬¸Ä±äºó·¢ËÍÒ»´Î(0x007);	

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_power_heat_data_t			power_heat_data;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}power_heat_dataFrame;					 	//»úÆ÷ÈËÎ»ÖÃĞÅÏ¢ºÍÇ¹¿Ú³¯ÏòÎ»ÖÃ(0x008)

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_game_robot_pos_t       game_robot_pos;    
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}game_robot_posFrame;               //

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_buff_musk_t            buff_musk;    
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}buff_muskFrame;               //

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	aerial_robot_energy_t      robot_energy;    
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}robot_energyFrame;     

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_robot_hurt_t           robot_hurt;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}robot_hurtFrame;   

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_shoot_data_t           shoot_data;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}shoot_dataFrame; 

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	ext_student_interactive_header_data_t student_interactive_header_data;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}student_interactive_header_dataFrame; 

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	client_custom_data_t       custom_data;   
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}custom_dataFrame; 

typedef __packed struct
{
	tFrameHeader    FrameHeader;
	tCmdID          CmdID;
	robot_interactive_data_t   interactive_data;  
	uint16_t        CRC16;         //Êı¾İCRCĞ£Ñé
}interactive_dataFrame; 


/* ²ÃÅĞÏµÍ³Êı¾İ»º³åÇø--------------------------------------------*/	
typedef struct
{
  uint16_t shoot_17_cooling_rate; //17Ã¿ÃëÀäÈ´Öµ
  uint16_t shoot_42_cooling_rate; 
  uint16_t shoot_17_cooling_limit; //17ÀäÈ´ÉÏÏŞ
  uint16_t shoot_42_cooling_limit;
  uint16_t shoot_17_heat;         //17Ç¹¿ÚÈÈÁ¿
  uint16_t shoot_42_heat;
  uint8_t  shoot_17_freq;         //17ÉäÆµ
  uint8_t  shoot_42_freq;
  float    shoot_17_speed;        //17ÉäËÙ
  float    shoot_42_speed; 
}_HEAT;  
   
typedef struct
{
  float x;
  float y;
  float z;
  float yaw; 
}_POS;     

typedef struct
{
 uint16_t Chassis_Volt;//µ×ÅÌÊä³öµçÑ¹
 uint16_t Chassis_Current;//µ×ÅÌÊä³öµçÁ÷
 float    chassis_Power;//µ×ÅÌ¹¦ÂÊ
 uint16_t Chassis_Power_buffer;//¹¦ÂÊ»º³å
}_POWER;  

   typedef  struct   //
{
  uint8_t id;              //idºÅ
  uint8_t level;           //µÈ¼¶
  uint16_t remainHp;       //Ê£ÓàÑªÁ¿
  uint16_t maxHp;          //×î´óÑªÁ¿
  uint8_t  Hp_ratio;       //ÑªÁ¿±È£¨Ê£Óà*10/×î´ó£©
  uint8_t  buff;           //ÔöÒæ
  _HEAT    heat;           //Ç¹¿ÚÈÈÁ¿
  _POS     postion;        //Î»ÖÃ
  _POWER   Chassis_Power;  //µ×ÅÌ¹¦ÂÊ
 
}ROBOT;


//ÄÚ²¿º¯Êı
uint8_t verify_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint8_t verify_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);
uint8_t get_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength, uint8_t ucCRC8);
uint16_t get_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t  append_crc8_check_sum(uint8_t* pchMessage, uint16_t dwLength);
uint16_t append_crc16_check_sum(uint8_t* pchMessage, uint32_t dwLength);

//½Ó¿Úº¯Êı
void Referee_Data_Task(void const * argument);
void Send_FrameData(tCmdID cmdid, uint8_t * pchMessage,uint8_t dwLength);
void sendata(void);


extern ROBOT Robot;
#endif
