/**
  ******************************************************************************
  * @file    Module_LED.h
  * @author  Z H(OUC Fab U+/ROV Team)
  * @brief   照明管理头文件
  * @version V1.1.1
	* @date    2025-8-18
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef Module_LED_H
#define Module_LED_H

#include "Global_Define.h"

/* 频率4kHz(占空比0~100代表熄灭~全亮) */

#define LED_Device_Num		 	4
#define LED_Value_Min				0
#define LED_Value_Max			450		//实际最大为500,这里为保护灯珠将最大占空比设为90%,换算的控制量即为450

typedef struct
{
	bool     State[LED_Device_Num];							 //通讯状态
	short    CQ_Current[LED_Device_Num]; 		//LED当前控制量
	short    CQ_Last[LED_Device_Num];    		//LED当前控制量
	uint8_t  inc_btn;
	uint8_t  dec_btn;
}LED_TypeDef;

extern LED_TypeDef LED;

void LED_Init(void);
void LED_Ctrl(void);

#endif
