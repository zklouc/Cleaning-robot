#ifndef _MODULE_ENCODER_H
#define _MODULE_ENCODER_H

#include "Global_Define.h"

typedef struct 
{
    uint16_t RawValue1;   // 原始单圈值 (0~4095)
	  uint16_t RawValue2; 
    uint16_t Speed1;   // 原始单圈值 (0~4095)
    uint16_t Speed2;   // 原始单圈值 (0~4095)
	  uint8_t  ErrCnt;   // 通信错误计数（用于监测总线健康）
} ROV_Encoder_Data;

// ===================== 状态定义 =====================
typedef enum {
    STATE_SEND_CMD1 = 0,  // 发送读角度指令
    STATE_SEND_CMD2,      // 发送读速度指令
    STATE_SEND_CMD3,      // 发送读角度指令
    STATE_SEND_CMD4,      // 发送读速度指令
} EncoderState_t;



extern ROV_Encoder_Data ROV_Encoder;
extern EncoderState_t currentState;
extern uint8_t Encoder_Reset_Flag;
	
void ROV_Encoder_SetBuad(void);
void ROV_Encoder_SetSampleTime(void);
void ROV_Encoder_SendResetCmd(void);
void ROV_Send_CMD(void);
void ROV_Encoders_Process(uint8_t *UART5_RxBuf);
#endif

