#ifndef MODULE_CAN_PROP_H
#define MODULE_CAN_PROP_H

#include "Global_Define.h"

/* 垂向推进器分布(俯视) */
/********************
***(1)*********(7)***
*********************
*********************
***(2)*********(6)***
*********************/

/* CAN发送扩展帧起始地址 */
#define				CAN_Prop_Tx_BaseID			 0x0501EF70

/* CAN接收扩展帧起始地址 */
#define				CAN_Prop_Rx_BaseID			 0x050170E0


/* 垂向推进器控制量 */
#define 			CAN_CQ_MIN		-5000
#define 			CAN_CQ_MID		0
#define				CAN_CQ_MAX		5000


/*推进器增量限幅*/
#define Prop_Add_Size  30    //数值越小缓起缓停越明显
#define Prop_Dec_Size  20 

/*转化系数*/
#define Convet_Factor(x,y)  ((float)((x-y)/255.0))
	
/*转弯转化系数*/
#define Convet1_Factor(x,y)  ((float)((x-y)/600.0))
	
/* 转弯控制量转化系数 */
#define	 Turn_Factor		2.4f
/**
 * @brief 推进器CAN通信控制结构体
 */
typedef struct
{
    uint16_t    Prop_Tx_Ctl;          // 推进器控制指令（转速/占空比）
    uint8_t     Prop_Tx_Config;       // 报文类型（0控制/1配置）
    uint8_t     Prop_Tx_Mode;         // 控制模式（0开环/1闭环）
    uint8_t     Prop_Tx_Direction;    // 电机转向（0正向/1反向）
    uint8_t     Prop_Tx_Curren_Range; // 过流阈值（A/10）
    uint8_t     Prop_Tx_Start_Time;   // 软启动时间（预留）
    uint16_t    Prop_Rx_RPM;          // 电机转速反馈（RPM）
//  uint8_t     Prop_Rx_Mode;         // 控制模式反馈（未启用）
    uint8_t     Prop_Rx_Temperature;  // 内部气压/温度（80-110Kpa）
    uint8_t     Prop_Rx_Current;     // 母线电流反馈（A/10）
    uint16_t    Prop_Rx_Voltage;        // 母线电压反馈（V）
    uint8_t     Prop_Rx_Warning;      // 报警信息（故障码）
    uint32_t    Prop_EXTID;           // CAN扩展ID
    uint8_t     Prop_life;            // 生命信号（循环发送）
}CAN_Prop_CtrlTypeDef;


extern CAN_Prop_CtrlTypeDef CAN_Prop[4];
extern uint8_t Vertical_Prop_ID[4];

void CAN_Config(void);
uint8_t Can_Prop_Send(CAN_HandleTypeDef *hcan, uint8_t *msg, uint8_t len, uint32_t CAN_EXT_ID);

void CAN_Prop_Init(void);

#endif
