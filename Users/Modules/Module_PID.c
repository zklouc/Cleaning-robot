#include "Module_PID.h"

#define CYCLE						0.05f		//控制周期50ms
#define Static_CQ			 	1400		//静态误差基准值
/* 自动控制对象 */
AutoCtrl_StateTypeDef hAuto[Auto_Num];
/* PID初始参数 */
float pid_kp[Auto_Num] = {75.0,  1.0, 20.0,  0.0, };
float pid_ki[Auto_Num] = {0.75,  0.0,	 1.0,  0.0, };//0.75
float pid_kd[Auto_Num] = {80.0,	 1.0,	 0.4,  0.0, };//80
float pid_db[Auto_Num] = { 0.0,  0.0,  0.0,  0.0, };
/* 控制器输出量参数 */
static float out_max = 500.0f;      //控制器输出最大值
static float out_KCP = 1500.0f;      //控制器输出补偿值
static float out_gate = 2.0f;       //阈值
uint8_t out_sqrt_gate = 1;       //数值开平方阈值
/* 控制模式 */
AutoCtrl_ModeTypeDef  Ctl_Mode[Auto_Num] = {MODE_PPID, MODE_PPID, MODE_PPID, MODE_PPID};
/* 控制器输出模式 */
AutoCtrl_OModeTypeDef OUT_Mode[Auto_Num] = {OUT_ORG, OUT_ORG, OUT_ORG, OUT_ORG};


/**
	* @brief PID参数初始化
  *
  * @param 无
	*
  * @retval 无
**/
void AutoParameter_Init(void)
{
	for (Auto_Type i = Auto_Depth; i <= Auto_Roll; ++i)
	{
		hAuto[i].act_value = 0;
		hAuto[i].set_value = 0;
		hAuto[i].enable		 = 0;
		hAuto[i].kp				 = pid_kp[i];
		hAuto[i].ki				 = pid_ki[i];
		hAuto[i].kd				 = pid_kd[i];
		hAuto[i].out_db		 = pid_db[i];
		hAuto[i].mode			 = Ctl_Mode[i];  
    hAuto[i].out_mode  = OUT_Mode[i]; 	
		hAuto[i].symbol		 = i;
		
		if (hAuto[i].ki != 0.0f)
    {
			float IntLim = out_max / (hAuto[i].ki * CYCLE);
			hAuto[i].Inter_MAX =  IntLim;
			hAuto[i].Inter_Min = -IntLim;
    }
    else
    {
			hAuto[i].Inter_MAX = 0.0f;
			hAuto[i].Inter_Min = 0.0f;
    }
	}
}


/**
	* @brief 最短路径法计算角度误差
  *
	* @param target 目标角度:(0.0 ~ 360.0)
	* @param current 当前角度:(0.0 ~ 360.0)
	*
	* @retval 归一化后的误差:(-180.0 ~ 180.0)
**/
float Calculate_Angle_Error(float target, float current)
{
    float error = target - current;

    /* 将误差限制在 -180 到 +180 之间 */
    while (error > 180.0f)
    {
        error -= 360.0f;
    }
    while (error < -180.0f)
    {
        error += 360.0f;
    }
    return error;
}


/**
	* @brief 控制器输出处理
  *
  * @param *hPid 		控制器类别
	*
  * @retval 无
**/
static void PID_OutOrg(AutoCtrl_StateTypeDef *hPid)
{
	float u_buf;
	u_buf = hPid->pidout;
	if (u_buf > out_max)
		u_buf = out_max;
	else if(u_buf < out_gate && u_buf > -out_gate)
		u_buf = 0;
	else if (u_buf < -out_max)
		u_buf = -out_max;
	
	hPid->pidout = u_buf;
}


/**
	* @brief 自动控制
  *
  * @param *PID_Type 		自动控制状态数据的地址
	*
  * @retval 无
**/
void AutoCtrl(AutoCtrl_StateTypeDef *PID_Type)
{
	/* 判断使能 */
	if (PID_Type->enable == 0) return;
	
	/* 参数更新 */
	switch(PID_Type->symbol)
	{
		case Auto_Depth:
			PID_Type->set_value = 0.2f + (float)(Prop_Ctrol.Vertical_Value_ADD - 1000) * 0.02f;
//			PID_Type->act_value = (float)WaterData.Conv_Depth[0] * 0.01f;
			break;
		case Auto_Heading:
			PID_Type->act_value = Attitude_Data.Heading;
			break;
		case Auto_Pitch:
			PID_Type->act_value = Attitude_Data.Pitch * 10;
			break;
		case Auto_Roll:
			PID_Type->act_value = Attitude_Data.Roll * 10;
			break;
		default: break;
	}
	
	/* 误差获取 */
	PID_Type->err[2] = PID_Type->set_value - PID_Type->act_value;
	
	/* 误差临界处理 */
	switch (PID_Type->symbol)
	{
		case Auto_Depth:
			/* 暂无 */
			break;
		case Auto_Heading:
			if (PID_Type->err[2] > 180)
				PID_Type->err[2] = PID_Type->err[2] - 360;
			else if (PID_Type->err[2] < -180)
				PID_Type->err[2] = PID_Type->err[2] + 360;
			break;
		case Auto_Pitch:
			if (PID_Type->err[2] > 1800)
				PID_Type->err[2] = PID_Type->err[2] - 3600;
			else if (PID_Type->err[2] < -1800)
				PID_Type->err[2] = PID_Type->err[2] + 3600;
			break;
		case Auto_Roll:
			if (PID_Type->err[2] > 1800)
				PID_Type->err[2] = PID_Type->err[2] - 3600;
			else if (PID_Type->err[2] < -1800)
				PID_Type->err[2] = PID_Type->err[2] + 3600;
			break;
		default: break;
	}
	
	/* 控制器选择 */
	switch (PID_Type->mode)
	{
		case MODE_PPID:   Position_Type_PID(PID_Type);           break;    //位置式PID     
		case MODE_SGPID:  Turtle_AutoCtrl_SGPID(PID_Type);       break;    //分段式PID
		case MODE_SCPID:  ROV_AutoCtrl_PPID(PID_Type);
			                PID_Type->pidout += PID_Type->out_dc;  break;
		case MODE_CCPID:  Turtle_Auto_CCPID();                   break;    //串级PID(定深上)
		case MODE_IPID:   Increment_Type_PID(PID_Type);          break;    //增量PID
		default:                                                 break;
	}
	
	/* 控制器输出非线性处理 */
  switch (PID_Type->out_mode)
	{
		case OUT_ORG:        PID_OutOrg(PID_Type); break;
		case OUT_XGATE:                            break;
		case OUT_YGATE:                            break;
		case OUT_SQRT:
			if (PID_Type->pidout > out_sqrt_gate)
				PID_Type->pidout = sqrt(PID_Type->pidout);
			PID_OutOrg(PID_Type);
			break;
		case OUT_DC:
			if (PID_Type->symbol == Auto_Depth)
				PID_Type->pidout += PID_Type->out_dc;
			PID_OutOrg(PID_Type);
			break;
		default:	break;
	}
	
	/* 控制量转化电机输出量 */
	switch(PID_Type->symbol)
	{
		int16_t auto_cq = 0;
		case Auto_Depth:
			auto_cq = (int16_t)(out_KCP - (short)PID_Type->pidout);
			for (uint8_t device_id = V_Prop_RF; device_id <= V_Prop_LF; device_id++)
			{
				Prop_Ctrol.Auto_H_CQ[device_id] = Static_CQ - (auto_cq - RS485_CQ_MID);
			}
			break;
		case Auto_Heading:
			break;
		case Auto_Pitch:
			break;
		case Auto_Roll:
			break;
		default: break;
	}
}






/***********************************************************************************************/
/*                                   自动控制算法函数                                          */                       
/***********************************************************************************************/
/***
           位置式PID(原始公式)--Kp参数没有进行分离
                              ***/
void ROV_AutoCtrl_PPID(AutoCtrl_StateTypeDef *hPid)
{
	uint8_t i;
	
	hPid->esum += hPid->err[2];   
	hPid->pidout = hPid->kp*hPid->err[2] + hPid->ki*CYCLE*hPid->esum\
								 + hPid->kd/CYCLE*(hPid->err[2] - hPid->err[1]);
	for (i=0;i<2;i++)
		hPid->err[i] = hPid->err[i+1];
	hPid->err[2] = 0;
	//pidout[k] = Kp*(e[k] + T / Ti*esum[k] + Td / T*(e[k] - e[k - 1]));
}
/***
           位置式PID(参数整合)
                              ***/
void Position_Type_PID(AutoCtrl_StateTypeDef* PID_Type)
{	
	  /* 静态变量保持上一拍滤波值 */
    static float D_filt = 0.0f;
    const  float alpha  = 0.2f;
		/****误差累加****/
		PID_Type->esum += PID_Type->err[2];                 //累积积分
		/****积分累积****/
	  PID_Type->Isum = PID_Type->ki*CYCLE*PID_Type->esum;  //积分环节
		/****积分抗饱和处理****/
		if(PID_Type->Isum > PID_Type->Inter_MAX)
			PID_Type->Isum = PID_Type->Inter_MAX;
    if(PID_Type->Isum < PID_Type->Inter_Min)		
			PID_Type->Isum = PID_Type->Inter_Min;
		/* 微分&一阶低通 */
    float D_raw = PID_Type->kd * (PID_Type->err[2] - PID_Type->err[1]);
    D_filt      = alpha * D_raw + (1.0f - alpha) * D_filt;   // IIR 低通
		
		/****位置式PID控制器输出计算****/
		PID_Type->pidout = PID_Type->kp * PID_Type->err[2] +\
		                   PID_Type->Isum +\
		                   D_filt; //Kd*(E(k)-E(K-1))
		
		/****误差传递赋值****/
	  for (uint8_t i=0;i<2;i++)
			PID_Type->err[i] = PID_Type->err[i+1];
		PID_Type->err[2] = 0;
}
/***
           增量式PID
                             ***/

void Increment_Type_PID(AutoCtrl_StateTypeDef* PID_Type)
{
  /*** ▲U(k) = U(k)-U(k-1)
	           = Kp(e(k)-e(k-1))+Ki*e(k)+Kd*(e(k)-2e(k-1)+e(k-2)) ***/ 
	
	/*误差已经在主体函数中处理完成*/
	
	/****增量式PID控制器输出计算****/                                                               
	PID_Type->pidout += PID_Type->kp * (PID_Type->err[2]-PID_Type->err[1]) +\
	                    PID_Type->ki * PID_Type->err[2]+\
	                    PID_Type->kd * (PID_Type->err[2]-2*PID_Type->err[1]+PID_Type->err[0]);
	
	/****误差传递赋值****/
	  for (uint8_t i=0;i<2;i++)
			PID_Type->err[i] = PID_Type->err[i+1];
		PID_Type->err[2] = 0;
}

/***
           分段式PID
                             ***/

/*分段式PID参数*/
uint8_t sg_maxpow = 3;			//分段PID分段数 = SG_MAXPOW-1
uint8_t sg_step		= 2;			//分段PID步长
float   sg_kp_bs  = 0.8;		//KP基数
float   sg_kd_bs  = 0.9;		//KD基数
float   sg_ki_bs	= 1;			//KI基数
/*分段PID控制*/
float pid_buf[3];

static float rov_pow(float buf, uint8_t cnt)
{
	uint8_t i;
	for (i=0;i<cnt;i++)
		buf *= buf;
	return buf;
}

void Turtle_AutoCtrl_SGPID(AutoCtrl_StateTypeDef *hPid)
{
	uint8_t i;
	uint8_t p;
	
	p = (uint8_t)(hPid->err[2]/sg_step);
	if (p < sg_maxpow)
		p = sg_maxpow - p;
	else
		p = 1;
	pid_buf[0] = hPid->kp*rov_pow(sg_kp_bs,p);
	pid_buf[1] = hPid->ki*rov_pow(sg_ki_bs,p);
	pid_buf[2] = hPid->kd*rov_pow(sg_kd_bs,p);
	hPid->esum += hPid->err[2];
	hPid->pidout = pid_buf[0]*(hPid->err[2] + pid_buf[1]*CYCLE*hPid->esum\
								 + pid_buf[2]/CYCLE*(hPid->err[2] - hPid->err[1]));
	for (i=0;i<2;i++)
		hPid->err[i] = hPid->err[i+1];
	hPid->err[2] = 0;
}
/***
           串级PID
                             ***/
static void Turtle_Auto_CCPID(void)
{
//	hDeepAuto[0].enable = AUTO_ENABLE;
//	hDeepAuto[1].enable = AUTO_ENABLE;
//	//外环PID计算
//	hDeepAuto[0].set_value = hAuto[0].set_value;
//	ROV_AutoCtl(&hDeepAuto[0]);
//	
//	//内环PID计算
//	hDeepAuto[1].set_value = hDeepAuto[0].pidout;
//	ROV_AutoCtl(&hDeepAuto[1]);
//	
//	hAuto[0].pidout = hDeepAuto[1].pidout;
}




