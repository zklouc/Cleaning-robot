/**
  ******************************************************************************
  * @file    Module_PID.h
  * @author  Ju CE(OUC Fab U+/ROV Team)
  * @brief   PID
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

#ifndef MODULE_PID_H
#define MODULE_PID_H

#include "Global_Define.h"


/* 以下各ID即为上文推进器分布ID */
typedef enum
{
	V_Prop_RF = 0,		//垂向右前--00
	V_Prop_RM,				//垂向右中--01
	V_Prop_RB,				//垂向右后--02
	V_Prop_LB,				//垂向左后--03
	V_Prop_LM,				//垂向左中--04
	V_Prop_LF,				//垂向左前--05
	H_Prop_RF,				//水平右前--06
	H_Prop_RB,				//水平右后--07
	H_Prop_LB,				//水平左后--08
	H_Prop_LF,				//水平左前--09
	LD_Prop_R,				//履带右侧--10
	LD_Prop_L,				//履带左侧--11
	Brush_Roll,				//滚刷旋转--12
	Brush_Lift,				//滚刷升降--13
	Bucket_Flex				//推铲伸缩--14
} Motor_XID_TypeDef;

typedef enum
{
  MODE_PPID = 0,
	MODE_SGPID,
	MODE_SCPID,
	MODE_CCPID,
	MODE_IPID,
}AutoCtrl_ModeTypeDef;

typedef enum
{
  OUT_ORG = 0,
  OUT_XGATE,
	OUT_YGATE,
	OUT_SQRT,
	OUT_DC,
}AutoCtrl_OModeTypeDef;

typedef enum
{
	Auto_Depth,						//自动定深
	Auto_Heading,					//自动定向
	Auto_Pitch,						//自动定俯仰
	Auto_Roll,						//自动定横滚
	Auto_Num							//自动模式种数
} Auto_Type;

typedef struct
{
	float											set_value;			//设定值---控制器计算最终设定值
	float											act_value;			//实际值
	float											max_range;			//最大范围
	float											err[3];					//误差值序列 [2]--当前误差
	float											esum;						//累计误差
	float                     Isum;           //积分累积
	float											kp;							//比例系数
	float											ki;							//积分系数
	float											kd;							//微分系数
	float											pidout;					//控制器输出
	float											out_dc;         
	float											out_db;  
  float                     Inter_MAX;      //积分饱和最大值
	float                     Inter_Min;      //积分饱和最小值
	uint8_t										out_mode;       //输出模式
	
	uint8_t                   Host_Enable;    //上位机使能
	uint8_t                   Key_Enable;     //键值使能
	
	uint8_t										enable;					//PID使能标志位
	uint8_t										symbol;					//PID控制标志位
	AutoCtrl_ModeTypeDef			mode;						//PID控制方式
} AutoCtrl_StateTypeDef;

extern AutoCtrl_StateTypeDef hAuto[Auto_Num];

void AutoParameter_Init(void);
static void PID_OutOrg(AutoCtrl_StateTypeDef *hPid);
void AutoCtrl(AutoCtrl_StateTypeDef *PID_Type);

void ROV_AutoCtrl_PPID(AutoCtrl_StateTypeDef *hPid);
void Position_Type_PID(AutoCtrl_StateTypeDef* PID_Type);
void Increment_Type_PID(AutoCtrl_StateTypeDef* PID_Type);
void Turtle_AutoCtrl_SGPID(AutoCtrl_StateTypeDef *hPid);
static void Turtle_Auto_CCPID(void);

#endif
