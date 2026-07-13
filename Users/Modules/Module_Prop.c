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
    .Final_CQ       = {0, 0, 0, 0,0, 0, 0, 0},
};

int16_t Turn_Conv_CQ(uint16_t cq)
{
    float differ = cq - RS485_CQ_MID;
    return (int16_t)(differ / Turn_Factor);
}


//限幅
uint16_t Clamp(uint16_t val, uint16_t min, uint16_t max)
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
    memset(Prop_Ctrol.Final_CQ,     0, sizeof(Prop_Ctrol.Final_CQ));
}


//垂向推进器控制量应用

void Prop_Apply_Vertical(void)
{
    for (uint8_t i = 0; i < 4; i++)
    {
//        uint16_t cq;
////        if (Prop_Ctrol.Deepth_Flag)
////            cq = Prop_Ctrol.Auto_V_CQ[i];
////        else
//        cq = Prop_Ctrol.Manual_V_CQ[i];

//        cq = Clamp(cq, CAN_CQ_MIN, CAN_CQ_MAX);
//        Prop_Ctrol.Final_CQ[i] = cq;
        CAN_Prop[i].Prop_Tx_Ctl = Prop_Ctrol.Manual_V_CQ[i];;
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

        cq = Clamp(cq, RS485_CQ_MIN, RS485_CQ_MAX);
        Prop_Ctrol.Final_CQ[i] = cq;
        Motor_SetSpeed(i+1, (int16_t)(cq - RS485_CQ_MID));
    }
}

void Manual_V_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    uint8_t key_Value=0x80;
    float   offset=0.0f;
    float   temp_cq=0.0f;
    uint16_t new_cq=0;
    const uint8_t KEY_MID = 0x80;    // 手柄中点 128
    
    for (;;)
    {
        // 1. 读取手柄值 + 限幅 [2, 252]
        key_Value = Prop_Ctrol.Vertical_Value_ADD;
        if(key_Value > 252)
            key_Value = 252;
        else if(key_Value < 2)
            key_Value = 2;

        // 2. 计算相对中点偏移：0x80 为 0，自动区分正负方向
        offset = (float)(key_Value - KEY_MID);

        // 3. 线性映射到推进器输出范围
        // Convet_Factor 按你的原有换算系数保留，也可自行改写纯线性映射
        temp_cq = CAN_CQ_MID + offset * Convet_Factor(CAN_CQ_MAX, CAN_CQ_MIN);
				
        if (temp_cq > 0.0f)
        {
            // 正数：方向配置 1 0 0 1
            Prop_Ctrol.Manual_V_DIR[0] = 1;
            Prop_Ctrol.Manual_V_DIR[1] = 0;
            Prop_Ctrol.Manual_V_DIR[2] = 0;
            Prop_Ctrol.Manual_V_DIR[3] = 1;
        }
        else
        {
            // 负数：可根据实际需求自定义反向方向，示例给 0 1 1 0
            Prop_Ctrol.Manual_V_DIR[0] = 0;
            Prop_Ctrol.Manual_V_DIR[1] = 1;
            Prop_Ctrol.Manual_V_DIR[2] = 1;
            Prop_Ctrol.Manual_V_DIR[3] = 0;
        }

        // 转速取绝对值
        temp_cq = fabsf(temp_cq);
        // ============================================================
				
				
        new_cq  = (uint16_t)temp_cq;

        // 4. 缓启缓停：单步最大变化 ±10，防止突变
        int16_t delta = new_cq - Prop_Ctrol.Manual_V_CQ[1];
        if(delta > 20)
        {
            new_cq = Prop_Ctrol.Manual_V_CQ[1] + 20;
        }
        else if(delta < -20)
        {
            new_cq = Prop_Ctrol.Manual_V_CQ[1] - 20;
        }
				new_cq= Clamp(new_cq, 0, 5000);
        // 5. 4路垂直推进器统一赋值
        for (uint8_t i = 0; i < 4; i++)
        {
					  Prop_Ctrol.Manual_V_CQ[i]= new_cq;
            
        }

        // 6. 互斥保护 + 执行推进器输出
        osMutexAcquire(Vertical_MutexHandle, osWaitForever);
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

    for (;;)
    {
            int16_t fb_cq = (int16_t)(JoyStick[L_FB] - RS485_CQ_MID);
            int16_t lr_cq = (int16_t)(JoyStick[L_LR] - RS485_CQ_MID);

            for (uint8_t i = 0; i < HORIZ_PROP_NUM; i++)
            {
                int16_t cq = RS485_CQ_MID;

                switch (i)
                {
                    case H_IDX_0: cq = RS485_CQ_MID + fb_cq + lr_cq; break;
                    case H_IDX_1: cq = RS485_CQ_MID + fb_cq - lr_cq; break;
                    case H_IDX_2: cq = RS485_CQ_MID - fb_cq + lr_cq; break;
                    case H_IDX_3: cq = RS485_CQ_MID - fb_cq - lr_cq; break;
                }

                Prop_Ctrol.Manual_H_CQ[i] = (uint16_t)Clamp(cq, RS485_CQ_MIN, RS485_CQ_MAX);
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

