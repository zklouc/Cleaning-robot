#ifndef MAINCOMMUNICATE_H
#define MAINCOMMUNICATE_H

#include "Module_UART.h"

#define HOST_START_H        		0XEB
#define HOST_START_L        		0X90
#define HOST_END_H        			0X0D
#define HOST_END_L        			0X0A

/* 摇杆ID号 */
typedef enum
{
	R_FB = 0,		//右侧前后摇杆
	R_LR,				//右侧左右摇杆
	L_FB,				//左侧前后摇杆
	L_LR				//左侧左右摇杆
} JoyStick_ID;


typedef struct
{
	uint16_t Current;
	uint16_t Voltage;
	uint16_t Humidity;
	uint16_t Temperature;
	uint16_t AirPressure;
} BatteryCabinValue_TypeDef;




typedef struct
{
	uint8_t 			Focus;
	uint8_t 		Defocus;
} Focalize_TypeDef;

extern volatile bool Comm_Flag;
extern BatteryCabinValue_TypeDef BatteryCabin;

void ROV_ReceiveData(void *argument);
void ROV_FeedbackData(void *argument);

#endif
