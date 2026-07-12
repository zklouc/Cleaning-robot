#include "Module_Prop.h"

Prop_Infor_TypeDef Prop_Infor =
{
    .mode           = CTRL_MODE_MANUAL,
    .Roll_Change    = 0,
    .Vertical_Value = CQ_MID,
    .Deepth_Flag    = false,
    .Heading_Flag   = false,
    .Manual_V_CQ    = {CQ_MID, CQ_MID, CQ_MID, CQ_MID},
    .Manual_H_CQ    = {CQ_MID, CQ_MID, CQ_MID, CQ_MID},
    .Auto_V_CQ      = {CQ_MID, CQ_MID, CQ_MID, CQ_MID},
    .Auto_H_CQ      = {CQ_MID, CQ_MID, CQ_MID, CQ_MID},
    .Final_CQ       = {CQ_MID, CQ_MID, CQ_MID, CQ_MID, CQ_MID, CQ_MID, CQ_MID, CQ_MID},
};

int16_t Turn_Conv_CQ(uint16_t cq)
{
    float differ = cq - CQ_MID;
    return (int16_t)(differ / Turn_Factor);
}

float Para_Gear(uint8_t gear)
{
    switch (gear)
    {
        case 0:  return 1.7f;
        case 1:  return 1.3f;
        default: return 9.0f;
    }
}

uint16_t Clamp(uint16_t val, uint16_t min, uint16_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void Prop_Apply_Vertical(void)
{
    for (uint8_t i = 0; i < VERT_PROP_NUM; i++)
    {
        uint16_t cq;
        if (Prop_Infor.Deepth_Flag)
            cq = Prop_Infor.Auto_V_CQ[i];
        else
            cq = Prop_Infor.Manual_V_CQ[i];

        cq = Clamp(cq, CQ_MIN, CQ_MAX);
        Prop_Infor.Final_CQ[i] = cq;
        CAN_Prop[Vertical_Prop_ID[i]].Prop_Tx_Ctl = cq;
    }
}

void Prop_Apply_Horizontal(void)
{
    for (uint8_t i = 0; i < HORIZ_PROP_NUM; i++)
    {
        uint16_t cq;
        if (Prop_Infor.Heading_Flag)
            cq = Prop_Infor.Auto_H_CQ[i];
        else
            cq = Prop_Infor.Manual_H_CQ[i];

        cq = Clamp(cq, CQ_MIN, CQ_MAX);
        Prop_Infor.Final_CQ[i] = cq;
        Motor_SetSpeed(i+1, (int16_t)(cq - CQ_MID));
    }
}

void Manual_V_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (Prop_Infor.mode != CTRL_MODE_STOP)
        {
            float temp_cq = CQ_MID + (float)(Prop_Infor.Vertical_Value - CQ_MID) / 1.7f;

            for (uint8_t i = 0; i < VERT_PROP_NUM; i++)
                Prop_Infor.Manual_V_CQ[i] = (uint16_t)temp_cq;

            osMutexAcquire(Vertical_MutexHandle, osWaitForever);
            Prop_Apply_Vertical();
            osMutexRelease(Vertical_MutexHandle);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void Manual_H_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (Prop_Infor.mode != CTRL_MODE_STOP)
        {
            int16_t fb_cq = (int16_t)(JoyStick[L_FB] - CQ_MID);
            int16_t lr_cq = (int16_t)(JoyStick[L_LR] - CQ_MID);

            for (uint8_t i = 0; i < HORIZ_PROP_NUM; i++)
            {
                int16_t cq = CQ_MID;

                switch (i)
                {
                    case H_IDX_0: cq = CQ_MID + fb_cq + lr_cq; break;
                    case H_IDX_1: cq = CQ_MID + fb_cq - lr_cq; break;
                    case H_IDX_2: cq = CQ_MID - fb_cq + lr_cq; break;
                    case H_IDX_3: cq = CQ_MID - fb_cq - lr_cq; break;
                }

                Prop_Infor.Manual_H_CQ[i] = (uint16_t)Clamp(cq, CQ_MIN, CQ_MAX);
            }

            osMutexAcquire(Horizontal_MutexHandle, osWaitForever);
            Prop_Apply_Horizontal();
            osMutexRelease(Horizontal_MutexHandle);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void Auto_V_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (Prop_Infor.mode == CTRL_MODE_AUTO)
        {
            hAuto[Auto_Depth].act_value = (float)ROV_Press.Conv_depth[0] * 0.01f;
            AutoCtrl(&hAuto[Auto_Depth]);

            osMutexAcquire(Vertical_MutexHandle, osWaitForever);
            Prop_Apply_Vertical();
            osMutexRelease(Vertical_MutexHandle);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

void Auto_H_Ctrl(void *argument)
{
    static TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        if (Prop_Infor.mode == CTRL_MODE_AUTO)
        {
            AutoCtrl(&hAuto[Auto_Heading]);
            AutoCtrl(&hAuto[Auto_Pitch]);
            AutoCtrl(&hAuto[Auto_Roll]);

            osMutexAcquire(Horizontal_MutexHandle, osWaitForever);
            Prop_Apply_Horizontal();
            osMutexRelease(Horizontal_MutexHandle);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}