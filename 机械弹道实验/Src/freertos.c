/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "data_processing.h"
#include "gimbal_control.h"
#include "gun_control.h"
#include "offline_check.h"
#include "other_tasks.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId testTaskHandle; 				
osThreadId RemoteDataTaskHandle;  
osThreadId RefereeDataTaskHandle;
osThreadId GimbalTaskHandle;		
osThreadId GunTaskHandle; 				
osThreadId MiniPCDataTaskHandle;
osThreadId LedTaskHandle;
osThreadId vOutLineCheckTaskHandle;
osThreadId CheckTaskHandle;
osThreadId MochaTaskHandle;
osThreadId shootheatTaskHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
extern void testTask(void const * argument);
extern void Remote_Data_Task(void const * argument);
extern void Referee_Data_Task(void const * argument);
extern void Gimbal_Task(void const * argument);
extern void Gun_Task(void const * argument);
extern void MiniPC_Data_task(void const * argument);
extern void Led_Task(void const * argument);
extern void vOutLineCheck_Task(void const *argument);
extern void Check_Task(void const *argument);
extern void Mocha_Task(void const *argument);
extern void Shoot_Heat_Task(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);           //暂未使用
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(Task1, testTask, osPriorityNormal, 0, 256);                         //测试任务，内容自定
	testTaskHandle = osThreadCreate(osThread(Task1), NULL);

//	osThreadDef(RemoteDataTask, Remote_Data_Task, osPriorityHigh, 0, 256);          //遥控器数据处理
//	RemoteDataTaskHandle = osThreadCreate(osThread(RemoteDataTask), NULL);

//	osThreadDef(MiniPCDataTask, MiniPC_Data_task, osPriorityAboveNormal, 0, 128);   //视觉数据处理
//	MiniPCDataTaskHandle = osThreadCreate(osThread(MiniPCDataTask), NULL);

//	osThreadDef(GimbalTask, Gimbal_Task, osPriorityNormal, 0, 256);                 //云台控制
//	GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);
	
	
	/**********发射控制**************/
	osThreadDef(GunTask, Gun_Task, osPriorityNormal, 0, 128);                       //枪口发射的拨盘控制
	GunTaskHandle = osThreadCreate(osThread(GunTask), NULL);
	
	osThreadDef(MochaTask, Mocha_Task, osPriorityNormal, 0, 128);                   //摩擦轮速度调整
	MochaTaskHandle = osThreadCreate(osThread(MochaTask), NULL);

//	osThreadDef(ShootHeattask, Shoot_Heat_Task, osPriorityNormal, 0, 128);          //枪口热量控制
//	shootheatTaskHandle = osThreadCreate(osThread(ShootHeattask), NULL);


  /***********断线检测**************/
//	osThreadDef(LedTask, Led_Task, osPriorityAboveNormal, 0, 256);                  //根据模块断线标志进行亮灯处理
//	LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);
//	
//	osThreadDef(CheckTask, Check_Task, osPriorityNormal, 0, 128);                   //根据任务断线标志进行处理
//	CheckTaskHandle = osThreadCreate(osThread(CheckTask), NULL);
//	
//	osThreadDef(vOutLineCheckTask, vOutLineCheck_Task, osPriorityNormal, 0, 128);   //其他任务断线检测、模块（电机、遥控器等）掉线检测及设置断线标志
//	vOutLineCheckTaskHandle = osThreadCreate(osThread(vOutLineCheckTask), NULL);
	
	
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    

  /* USER CODE BEGIN StartDefaultTask */
	
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
