/**
  ******************************************************************************
  * @file    ROV_Pressure_Sensor.h
  * @author  Li D(OUC Fab U+/ROV Team)
  * @brief   ROV 压力传感器头文件
	* @Hardware自研压力传感器--X101
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
	
#ifndef ROV_Pressure_Sensor_H
#define ROV_Pressure_Sensor_H

/***头文件定义区***/
#include "Global_Define.h"
/***宏定义区***/

#define PRESS_START_H		0xEB
#define PRESS_START_L		0x90
#define PRESS_END_H			0x0D
#define PRESS_END_L			0x0A
#define PRESS_LENGTH		16u

#define PRESS_CONV_K		1.0336f   //压力-深度转换系数

#define lpt_k           0.5       //低通滤波系数
/***枚举定义区***/


/***结构体定义去***/
typedef struct
{
	uint16_t		Init_press;					//初始压力
	uint16_t		Press;							//mbar
	uint16_t		Press_Temp;						
	uint16_t		Press_last;         //上一时刻的压力值
	int16_t			Temp;								//0.01°
	uint16_t		Depth;							//cm
	
	float		    Conv_depth[10];			//换算深度 cm
	float				Conv_speed[10];			//换算速度 mm/s
	
	//Func_State  Press_State;      //压力传感器运行状态  
}Rov_PressData;

/*函数声明区*/
void ROV_Press_Init(void);

/***变量外部引用区***/
extern Rov_PressData ROV_Press;

#endif
