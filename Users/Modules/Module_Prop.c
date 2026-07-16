#include "Module_Prop.h"
#include <string.h>

Prop_Control_Data_TypeDef Prop_Ctrol =
{
    .Roll_Change    = 0,
    .Vertical_Value_DEC = 0,
	  .Vertical_Value_ADD = 0,
    .Deepth_Flag    = false,
    .Heading_Flag   = false,
    .Manual_V_CQ    = {CAN_CQ_MID, CAN_CQ_MID, CAN_CQ_MID, CAN_CQ_MID},
		.Manual_V_DIR   = {0, 0, 0, 0},
		.Manual_V_RES   = {0, 0, 0, 0},
    .Manual_H_CQ    = {RS485_CQ_MID, RS485_CQ_MID, RS485_CQ_MID, RS485_CQ_MID},
		.Manual_H_DIR   = {0, 0, 0, 0},
    .Auto_V_CQ      = {CAN_CQ_MID, CAN_CQ_MID, CAN_CQ_MID, CAN_CQ_MID},
    .Auto_H_CQ      = {RS485_CQ_MID, RS485_CQ_MID, RS485_CQ_MID, RS485_CQ_MID},
    .Final_H_CQ       = {0, 0, 0, 0},
		.Final_V_CQ       = {0, 0, 0, 0},
};

int16_t Turn_Conv_CQ(uint16_t cq)
{
    float differ = cq - RS485_CQ_MID;
    return (int16_t)(differ / Turn_Factor);
}


//限幅
uint16_t Clamp_uint(uint16_t val, uint16_t min, uint16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

int16_t Clamp_int(int16_t val, int16_t min, uint16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}



#include <string.h>

void Prop_Init(void)
{
    // 重新完整初始化一遍
    Prop_Ctrol.Deepth_Flag = false;
    Prop_Ctrol.Heading_Flag = false;
    Prop_Ctrol.Roll_Change = 0;
    Prop_Ctrol.Vertical_Value_DEC = 0x80;
    Prop_Ctrol.Vertical_Value_ADD = 0x80;

    memset(Prop_Ctrol.Manual_V_CQ,  0, sizeof(Prop_Ctrol.Manual_V_CQ));
    memset(Prop_Ctrol.Manual_V_DIR, 0, sizeof(Prop_Ctrol.Manual_V_DIR));
    memset(Prop_Ctrol.Manual_H_CQ,  0, sizeof(Prop_Ctrol.Manual_H_CQ));
    memset(Prop_Ctrol.Manual_H_DIR, 0, sizeof(Prop_Ctrol.Manual_H_DIR));
    memset(Prop_Ctrol.Auto_V_CQ,    0, sizeof(Prop_Ctrol.Auto_V_CQ));
    memset(Prop_Ctrol.Auto_H_CQ,    0, sizeof(Prop_Ctrol.Auto_H_CQ));
	  memset(Prop_Ctrol.Final_V_CQ,     0, sizeof(Prop_Ctrol.Final_V_CQ));
    memset(Prop_Ctrol.Final_H_CQ,     0, sizeof(Prop_Ctrol.Final_H_CQ));
}


//垂向推进器控制量应用

void Prop_Apply_Vertical(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        uint16_t cq;
        if (Prop_Ctrol.Deepth_Flag)
            cq = Prop_Ctrol.Auto_V_CQ[i];
        else
            cq = Prop_Ctrol.Manual_V_CQ[i];

        Prop_Ctrol.Final_V_CQ[i] = cq;
			  CAN_Prop[i].Prop_Tx_Curren_Range = 0x64;
        CAN_Prop[i].Prop_Tx_Ctl = cq;  //Prop_Ctrol.Manual_V_CQ[i];
			  CAN_Prop[i].Prop_Tx_Direction = Prop_Ctrol.Manual_V_DIR[i];
    }
}

void Prop_Apply_Horizontal(void)
{
    for (uint8_t i = 0; i < HORIZ_PROP_NUM; i++)
    {
        uint16_t cq;
        if (Prop_Ctrol.Heading_Flag)
            cq = Prop_Ctrol.Auto_H_CQ[i];
        else
            cq = Prop_Ctrol.Manual_H_CQ[i];
				
        Prop_Ctrol.Final_H_CQ[i] = cq;
				RS485_Prop[i].target_rpm = cq;
//        Motor_SetSpeed(i+1, (int16_t)(cq - RS485_CQ_MID));
    }
}

float Cal_Offset(uint8_t Input_Value)
{
	uint8_t key_Value = Input_Value;
	float   offset=0.0f; 
	//读取手柄值 + 限幅 [2, 252]
	if(key_Value > 252)
            key_Value = 252;
	else if(key_Value < 2)
			key_Value = 2;
	offset =(float)(key_Value - KEY_MID);
	return offset;
				
}

void Manual_V_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    uint8_t key_Value=0x80;
    float   offset=0.0f;
    float   temp_cq=0.0f;
    uint16_t new_cq=0;
    
    for (;;)
    {
        // 计算相对中点偏移：0x80 为 0，自动区分正负方向
        offset = Cal_Offset(Prop_Ctrol.Vertical_Value_ADD);

        // 线性映射到推进器输出范围
        temp_cq = CAN_CQ_MID + offset * Convet_Factor(CAN_CQ_MAX, CAN_CQ_MIN);
				
        if (offset > 0.0f)
        {
            // 正数：方向配置 1 0 0 1
            Prop_Ctrol.Manual_V_DIR[0] = 0;
            Prop_Ctrol.Manual_V_DIR[1] = 1;
            Prop_Ctrol.Manual_V_DIR[2] = 0;
            Prop_Ctrol.Manual_V_DIR[3] = 1;
        }
        else if(offset < 0.0f)
        {
            // 负数：可根据实际需求自定义反向方向，示例给 0 1 1 0
            Prop_Ctrol.Manual_V_DIR[0] = 1;
            Prop_Ctrol.Manual_V_DIR[1] = 0;
            Prop_Ctrol.Manual_V_DIR[2] = 1;
            Prop_Ctrol.Manual_V_DIR[3] = 0;
        }
				
        // 转速取绝对值
        temp_cq = fabsf(temp_cq);
        new_cq  = (uint16_t)temp_cq;

        //缓启缓停，防止突变
        int16_t delta = new_cq - Prop_Ctrol.Manual_V_CQ[1];//先使用一个推进器去升降
        if(delta > 50)
        {
            new_cq = Prop_Ctrol.Manual_V_CQ[1] + 50;
        }
        else if(delta < -30)
        {
            new_cq = Prop_Ctrol.Manual_V_CQ[1] - 30;
        }
				new_cq= Clamp_uint(new_cq, 0, 5000);

        //  互斥保护 + 执行推进器输出
        osMutexAcquire(Vertical_MutexHandle, osWaitForever);
				//  4路垂直推进器统一赋值
        for (uint8_t i = 0; i < 4; i++)
        {
					  Prop_Ctrol.Manual_V_CQ[i]= new_cq; 
        }
				
        Prop_Apply_Vertical();
        osMutexRelease(Vertical_MutexHandle);

        // 50ms 周期任务
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void Manual_H_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	  float   offset=0.0f;
    float   temp_cq=0.0f;
    for (;;)
    {
			  // 计算相对中点偏移：0x80 为 0，自动区分正负方向
				offset = Cal_Offset(JoyStick[R_FB]);//摇杆波动偏移		
				// 线性映射到推进器输出范围
				temp_cq = RS485_CQ_MID + offset * Convet_Factor(RS485_CQ_MAX, RS485_CQ_MIN);
			  int16_t FB_CQ = (int16_t)temp_cq;//前后控制
			
			  offset = Cal_Offset(JoyStick[L_LR]);
			  temp_cq = RS485_CQ_MID + offset * Convet_Factor(RS485_CQ_MAX, RS485_CQ_MIN);
			  int16_t LR_CQ = (int16_t)temp_cq;//左右控制
			
				for (uint8_t i = 0; i < HORIZ_PROP_NUM; i++)
				{
						int16_t cq = RS485_CQ_MID;
						switch (i)
						{
								case H_IDX_0: cq = RS485_CQ_MID + FB_CQ + LR_CQ; break;
								case H_IDX_1: cq = RS485_CQ_MID + FB_CQ - LR_CQ; break;
								case H_IDX_2: cq = RS485_CQ_MID - FB_CQ + LR_CQ; break;
								case H_IDX_3: cq = RS485_CQ_MID - FB_CQ - LR_CQ; break;
						}
						Prop_Ctrol.Manual_H_CQ[i] = Clamp_int(cq, RS485_CQ_MIN, RS485_CQ_MAX);
				}

            osMutexAcquire(Horizontal_MutexHandle, osWaitForever);
            Prop_Apply_Horizontal();
            osMutexRelease(Horizontal_MutexHandle);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void Auto_V_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
            hAuto[Auto_Depth].act_value = (float)ROV_Press.Conv_depth[0] * 0.01f;
            AutoCtrl(&hAuto[Auto_Depth]);

            osMutexAcquire(Vertical_MutexHandle, osWaitForever);
//            Prop_Apply_Vertical();
            osMutexRelease(Vertical_MutexHandle);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void Auto_H_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
            AutoCtrl(&hAuto[Auto_Heading]);
            AutoCtrl(&hAuto[Auto_Pitch]);
            AutoCtrl(&hAuto[Auto_Roll]);

            osMutexAcquire(Horizontal_MutexHandle, osWaitForever);
            Prop_Apply_Horizontal();
            osMutexRelease(Horizontal_MutexHandle);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

