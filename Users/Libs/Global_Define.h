/**
  ******************************************************************************
  * @file    Global_Define.h
  * @author  Z H(OUC Fab U+/ROV Team)
  * @brief   全局Define
  * @version V1.0
	* @date    2025-8-8
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef GLOBAL_DEFINE_H
#define GLOBAL_DEFINE_H

/* 头文件定义区 ----------------------------------------------*/
/* C语言 Includes --------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
/* FreeRTOS Includes -----------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "task.h"
/* STM32 Includes --------------------------*/
#include "dma.h"
#include "can.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "usart.h"
#include "stm32f4xx.h"
/* Users Includes --------------------------*/
/* Libs ---------------------*/
#include "crc16_lib.h"
/* Modules ------------------*/
#include "Module_Prop.h"
#include "Module_CAN_Prop.h"
#include "Module_485_Prop.h"
#include "Module_LED.h"
#include "Module_PID.h"
#include "Module_UART.h"
#include "Module_Encoder.h"
#include "Error_Handle.h"
#include "Module_Compass.h"
#include "ROV_T&H_Sensor.h"
#include "MainCommunicate.h"
#include "ROV_AirPressure_Sensor.h"
#include "ROV_Pressure_Sensor.h"



/* 宏定义区 --------------------------------------------------*/

/* 枚举定义区 ------------------------------------------------*/

/* 函数运行状态 ----------------------------------------------*/

/* 结构体定义区 ----------------------------------------------*/

/* 函数声明区 ------------------------------------------------*/

/* 变量外部引用区 --------------------------------------------*/
extern osTimerId_t Timer_StandbyHandle;
extern osMutexId_t Vertical_MutexHandle;
extern osMutexId_t Horizontal_MutexHandle;
extern osThreadId_t Manual_V_TaskHandle;
extern osThreadId_t Manual_H_TaskHandle;
extern osThreadId_t Auto_V_TaskHandle;
extern osThreadId_t Auto_H_TaskHandle;

#endif
