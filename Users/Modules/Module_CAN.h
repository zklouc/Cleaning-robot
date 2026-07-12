/**
  ******************************************************************************
  * @file    Module_CAN.h
  * @author  Z H(OUC Fab U+/ROV Team)
  * @brief   CAN外设管理头文件
  * @version V1.4.0
	* @date    2025-9-6
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef MODULE_CAN_H
#define MODULE_CAN_H

#include <can.h>
#include "Module_CAN_Prop.h"

/* CAN发送扩展帧起始地址 */
#define				Prop_Tx_BaseID			 0x0501EF70

/* CAN接收扩展帧起始地址 */
#define				Prop_Rx_BaseID			 0x050170E0

typedef struct
{
	uint16_t    Prop_Tx_Ctl;
	uint8_t     Prop_Tx_Config;
	uint8_t     Prop_Tx_Mode;
	uint8_t     Prop_Tx_Direction;
	uint8_t     Prop_Tx_Curren_Range;
	uint8_t     Prop_Tx_Start_Time;
	uint16_t    Prop_Rx_RPM;
//	uint8_t     Prop_Rx_Mode;
	uint8_t     Prop_Rx_Temperature;
	uint8_t     Prop_Rx_Current;
	uint16_t    Prop_Rx_Volte;
	uint8_t     Prop_Rx_Warning;
	uint32_t    Prop_EXTID;
	uint8_t     Prop_life;
}CAN_Prop_Typedef;

void CAN_Config(void);
uint8_t Can_Prop_Send(CAN_HandleTypeDef *hcan, uint8_t *msg, uint8_t len, uint32_t CAN_EXT_ID);
uint8_t Can_LED_Send(CAN_HandleTypeDef *hcan, uint8_t *msg, uint8_t len, uint32_t CAN_EXT_ID);
uint8_t Can_FOC_Send(CAN_HandleTypeDef *hcan, uint8_t *msg, uint8_t len, uint16_t CAN_STD_ID);


#endif
