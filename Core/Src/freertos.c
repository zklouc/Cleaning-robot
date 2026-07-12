/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Global_Define.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN1_Propel_TasK */
osThreadId_t CAN1_Propel_TasKHandle;
const osThreadAttr_t CAN1_Propel_TasK_attributes = {
  .name = "CAN1_Propel_TasK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for ROV_Feedback_Task */
osThreadId_t ROV_Feedback_TaskHandle;
const osThreadAttr_t ROV_Feedback_Task_attributes = {
  .name = "ROV_Feedback_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for ROV_Receive_Task */
osThreadId_t ROV_Receive_TaskHandle;
const osThreadAttr_t ROV_Receive_Task_attributes = {
  .name = "ROV_Receive_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for Init_Task */
osThreadId_t Init_TaskHandle;
const osThreadAttr_t Init_Task_attributes = {
  .name = "Init_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for Cabin_Env_Data_Task */
osThreadId_t Cabin_Env_Data_TaskHandle;
const osThreadAttr_t Cabin_Env_Data_Task_attributes = {
  .name = "Cabin_Env_Data_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for PWM_Task */
osThreadId_t PWM_TaskHandle;
const osThreadAttr_t PWM_Task_attributes = {
  .name = "PWM_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for Press_Data_Task */
osThreadId_t Press_Data_TaskHandle;
const osThreadAttr_t Press_Data_Task_attributes = {
  .name = "Press_Data_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for Compass_Data_Task */
osThreadId_t Compass_Data_TaskHandle;
const osThreadAttr_t Compass_Data_Task_attributes = {
  .name = "Compass_Data_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime3,
};
/* Definitions for RS485_Propel_Tx_Task */
osThreadId_t RS485_Propel_Tx_TaskHandle;
const osThreadAttr_t RS485_Propel_Tx_Task_attributes = {
  .name = "RS485_Propel_Tx_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for RS485_Propel_Rx_Task */
osThreadId_t RS485_Propel_Rx_TaskHandle;
const osThreadAttr_t RS485_Propel_Rx_Task_attributes = {
  .name = "RS485_Propel_Rx_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime4,
};
/* Definitions for Manual_V_Task */
osThreadId_t Manual_V_TaskHandle;
const osThreadAttr_t Manual_V_Task_attributes = {
  .name = "Manual_V_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for Manual_H_Task */
osThreadId_t Manual_H_TaskHandle;
const osThreadAttr_t Manual_H_Task_attributes = {
  .name = "Manual_H_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for Auto_V_Task */
osThreadId_t Auto_V_TaskHandle;
const osThreadAttr_t Auto_V_Task_attributes = {
  .name = "Auto_V_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for Auto_H_Task */
osThreadId_t Auto_H_TaskHandle;
const osThreadAttr_t Auto_H_Task_attributes = {
  .name = "Auto_H_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for Enc_Data_Task */
osThreadId_t Enc_Data_TaskHandle;
const osThreadAttr_t Enc_Data_Task_attributes = {
  .name = "Enc_Data_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime2,
};
/* Definitions for Timer_Standby */
osTimerId_t Timer_StandbyHandle;
const osTimerAttr_t Timer_Standby_attributes = {
  .name = "Timer_Standby"
};
/* Definitions for Vertical_Mutex */
osMutexId_t Vertical_MutexHandle;
const osMutexAttr_t Vertical_Mutex_attributes = {
  .name = "Vertical_Mutex"
};
/* Definitions for Horizontal_Mutex */
osMutexId_t Horizontal_MutexHandle;
const osMutexAttr_t Horizontal_Mutex_attributes = {
  .name = "Horizontal_Mutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void CAN1_SendData(void *argument);
extern void ROV_FeedbackData(void *argument);
extern void ROV_ReceiveData(void *argument);
void Global_Init(void *argument);
void ControlCabin_GetData(void *argument);
void PWM_Ctrl(void *argument);
extern void Press_DataPro(void *argument);
extern void Compass_GetData(void *argument);
extern void RS485_Propel_Set(void *argument);
extern void RS485_Propel_Receive(void *argument);
extern void Manual_V_Ctrl(void *argument);
extern void Manual_H_Ctrl(void *argument);
extern void Auto_V_Ctrl(void *argument);
extern void Auto_H_Ctrl(void *argument);
extern void Encoder_Data_Get(void *argument);
void Standby_Callback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of Vertical_Mutex */
  Vertical_MutexHandle = osMutexNew(&Vertical_Mutex_attributes);

  /* creation of Horizontal_Mutex */
  Horizontal_MutexHandle = osMutexNew(&Horizontal_Mutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Timer_Standby */
  Timer_StandbyHandle = osTimerNew(Standby_Callback, osTimerPeriodic, NULL, &Timer_Standby_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CAN1_Propel_TasK */
  CAN1_Propel_TasKHandle = osThreadNew(CAN1_SendData, NULL, &CAN1_Propel_TasK_attributes);

  /* creation of ROV_Feedback_Task */
  ROV_Feedback_TaskHandle = osThreadNew(ROV_FeedbackData, NULL, &ROV_Feedback_Task_attributes);

  /* creation of ROV_Receive_Task */
  ROV_Receive_TaskHandle = osThreadNew(ROV_ReceiveData, NULL, &ROV_Receive_Task_attributes);

  /* creation of Init_Task */
  Init_TaskHandle = osThreadNew(Global_Init, NULL, &Init_Task_attributes);

  /* creation of Cabin_Env_Data_Task */
  Cabin_Env_Data_TaskHandle = osThreadNew(ControlCabin_GetData, NULL, &Cabin_Env_Data_Task_attributes);

  /* creation of PWM_Task */
  PWM_TaskHandle = osThreadNew(PWM_Ctrl, NULL, &PWM_Task_attributes);

  /* creation of Press_Data_Task */
  Press_Data_TaskHandle = osThreadNew(Press_DataPro, NULL, &Press_Data_Task_attributes);

  /* creation of Compass_Data_Task */
  Compass_Data_TaskHandle = osThreadNew(Compass_GetData, NULL, &Compass_Data_Task_attributes);

//  /* creation of RS485_Propel_Tx_Task */
//  RS485_Propel_Tx_TaskHandle = osThreadNew(RS485_Propel_Set, NULL, &RS485_Propel_Tx_Task_attributes);

//  /* creation of RS485_Propel_Rx_Task */
//  RS485_Propel_Rx_TaskHandle = osThreadNew(RS485_Propel_Receive, NULL, &RS485_Propel_Rx_Task_attributes);

  /* creation of Manual_V_Task */
  Manual_V_TaskHandle = osThreadNew(Manual_V_Ctrl, NULL, &Manual_V_Task_attributes);

/* creation of Manual_H_Task */
  Manual_H_TaskHandle = osThreadNew(Manual_H_Ctrl, NULL, &Manual_H_Task_attributes);

//  /* creation of Auto_V_Task */
//  Auto_V_TaskHandle = osThreadNew(Auto_V_Ctrl, NULL, &Auto_V_Task_attributes);

//  /* creation of Auto_H_Task */
//  Auto_H_TaskHandle = osThreadNew(Auto_H_Ctrl, NULL, &Auto_H_Task_attributes);

  /* creation of Enc_Data_Task */
//  Enc_Data_TaskHandle = osThreadNew(Encoder_Data_Get, NULL, &Enc_Data_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		/* 正常运行的代码永远不会执行到这里 */
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Global_Init */
/**
* @brief Function implementing the Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Global_Init */
void Global_Init(void *argument)
{
  /* USER CODE BEGIN Global_Init */
	
	/* 关闭调度器--禁止任务切换 */
  vTaskSuspendAll();
	
	/* PID初始化 */
	AutoParameter_Init();
	
	/* CAN初始化 */
	CAN_Config();
	
	/* 六路串口初始化 */
	User_UART_Init();
	
	/* LED初始化 */
  LED_Init();
	
	/* 电子罗盘初始化 */
	Compass_Init();
	
	/* 旋转编码器初始化 */
//	Enc_Init();
	
	/* 温湿度传感器初始化 */
	SHT3X_Init(0x44);
	hSHT30.error = SHT3X_SoftReset();
	HAL_Delay(10);
	hSHT30.error |= SHT3X_ReadStatus(&hSHT30.status.u16);
	hSHT30.error = SHT3X_StartPeriodicMeasurment(REPEATAB_HIGH, FREQUENCY_4HZ);
	
	/* 推进器初始化 */
	CAN_Prop_Init();
	
	/* 待机定时器初始化 */
	osTimerStart(Timer_StandbyHandle, 3000);
	
	/* 重新打开调度器 */
  xTaskResumeAll();
	
	/* 深度传感器初始化 */
	ROV_Press_Init();
	
	/* 初始化完成--自删除 */
	vTaskDelete(NULL);
	
  /* USER CODE END Global_Init */
}

/* USER CODE BEGIN Header_ControlCabin_GetData */
/**
* @brief Function implementing the ControlCabin_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlCabin_GetData */
void ControlCabin_GetData(void *argument)
{
  /* USER CODE BEGIN ControlCabin_GetData */
	
	/* 记录起始节拍 */
	static TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
  /* Infinite loop */
  for(;;)
  {
    /* 控制仓温湿度 */
		hSHT30.error = SHT3X_ReadMeasurementBuffer(&hSHT30.temperature, &hSHT30.humidity);
		
		/* 控制仓气压 */
		AD_GetValue();
		
		/* 任务总时长固定100ms */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
  }
  /* USER CODE END ControlCabin_GetData */
}

/* USER CODE BEGIN Header_PWM_Ctrl */
/**
* @brief Function implementing the PWM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PWM_Ctrl */
void PWM_Ctrl(void *argument)
{
  /* USER CODE BEGIN PWM_Ctrl */
	
	/* 记录起始节拍 */
	static TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
	
  /* Infinite loop */
  for(;;)
  {
		/* LED控制 */
		LED_Ctrl();
		
    /* 任务总时长固定50ms */
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
  }
  /* USER CODE END PWM_Ctrl */
}

/* Standby_Callback function */
void Standby_Callback(void *argument)
{
  /* USER CODE BEGIN Standby_Callback */
	
	/* 收到数据:清理标志,下次再检 */
	if (Comm_Flag) Comm_Flag = false;
	/* 3s未收到数据:进入待机状态,清除所有控制量	*/
	else
	{
		/* 摇杆控制量 */
		for (uint8_t i = R_FB; i <= L_LR; i++)
		{
			JoyStick[i] = 1500;
		}
		
		/* 上浮下潜控制量 */
		Prop_Infor.Vertical_Value = 1500;
		
		/* 灯亮度控制量 */
		for (uint8_t i = 0; i < LED_Device_Num; i++)
		{
			LED.CQ_Current[i] = 0;
		}
		
		/* 推进器控制模式 */
//		Motor_Kind[Vertical].Enable_Flag = 0;
//		Motor_Kind[Horizontal].Enable_Flag = 0;
//		Motor_Kind[Vertical].Gear = 0;
//		Motor_Kind[Horizontal].Gear = 0;
//		Prop_Infor.Roll_Flag = false;
	}
		
  /* USER CODE END Standby_Callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

