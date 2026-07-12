#include "Module_LED.h"

#define  LED_Add_Size  2

LED_TypeDef LED;

static const uint32_t LED_Channel[4] = 
{
	TIM_CHANNEL_1,
	TIM_CHANNEL_2,
	TIM_CHANNEL_3,
	TIM_CHANNEL_4
};


/**
  * @brief LED相关初始化
  *
  * @param 无
	*
  * @retval 无
**/
void LED_Init(void)
{
	for(uint8_t i = 0; i < LED_Device_Num; i++)
	{
		HAL_TIM_PWM_Start(&htim2, LED_Channel[i]);
		__HAL_TIM_SET_COMPARE(&htim2, LED_Channel[i], LED_Value_Min);
		LED.CQ_Current[i] = LED_Value_Min;
		LED.CQ_Last[i] = LED_Value_Min;
		LED.State[i] = true;
	}
}

/**
  * @brief  LED 亮度控制（按键步进调节）
  * @note   每 50ms 由 PWM_Ctrl 任务调用一次
  * @param  无
  * @retval 无
  */
void LED_Ctrl(void)
{
    // 静态变量保存上一次按键状态（用于上升沿检测）
    static uint8_t last_inc = 0;  // 上一周期的加键状态
    static uint8_t last_dec = 0;  // 上一周期的减键状态

    // 读取当前按键值（frame0 需为全局或 extern 变量）
 

    // ------- 1. 步进调节（上升沿触发） -------
    if (LED.inc_btn == 1 && last_inc == 0)   // 加键按下瞬间
    {
        for (uint8_t i = 0; i < LED_Device_Num; i++)
        {
            LED.CQ_Current[i] += LED_Add_Size;  // 步长 10
            if (LED.CQ_Current[i] > LED_Value_Max)  // 上限 500
                LED.CQ_Current[i] = LED_Value_Max;
        }
    }

    if (LED.dec_btn == 1 && last_dec == 0)   // 减键按下瞬间
    {
        for (uint8_t i = 0; i < LED_Device_Num; i++)
        {
            if (LED.CQ_Current[i] >= LED_Add_Size)
                LED.CQ_Current[i] -= LED_Add_Size;
            else
                LED.CQ_Current[i] = LED_Value_Min;  // 下限 0
        }
    }

    // 更新上一次状态（供下次比较）
    last_inc = LED.inc_btn;
    last_dec = LED.dec_btn;

    // ------- 2. 限幅保护 & 写入硬件 -------
    for (uint8_t i = 0; i < LED_Device_Num; i++)
    {
        // 二次限幅（防止意外越界）
        if (LED.CQ_Current[i] < LED_Value_Min)
            LED.CQ_Current[i] = LED_Value_Min;
        else if (LED.CQ_Current[i] > LED_Value_Max)
            LED.CQ_Current[i] = LED_Value_Max;

        // 仅在值变化时更新硬件（减少无效操作）
        if (LED.CQ_Current[i] != LED.CQ_Last[i])
        {
            __HAL_TIM_SET_COMPARE(&htim2, LED_Channel[i], LED.CQ_Current[i]);
            LED.CQ_Last[i] = LED.CQ_Current[i];
        }
    }
}

