/**
  ******************************************************************************
  * @file    ROV_Prop_Module.c.c
  * @author  Li D(OUC Fab U+/ROV Team)
  * @brief   ROV 压力传感器文件
  * @version V1.0
	* @date    2022-3-9
	* @Hardware自研压力传感器--X101
  ******************************************************************************
  * @attention
  * 后续更新升级硬件，可使用 #if #elif #endif 进行软处理
  *
  ******************************************************************************
  */
	
/* Includes ------------------------------------------------------------------*/
#include "ROV_Pressure_Sensor.h"

/***申请变量区***/


/***创建传感器结构体对象***/
Rov_PressData ROV_Press;


/***功能函数定义区***/

/**
  * @brief 压力传感器初始化（采集5次有效压力求平均）
  * @param 无
  * @retval 无
  * @note   必须在 FreeRTOS 任务中调用（因为使用了阻塞队列），且调度器已运行。
  */
void ROV_Press_Init(void)
{
    uint8_t  valid_cnt = 0;
    uint32_t press_sum = 0;
    Frame_TypeDef frame;
    UsartPort_TypeDef *port = &UsartPort[DPH_UART_ID];   // DPH_UART_ID 对应 USART2

    // 等待并接收5帧有效数据
    while (valid_cnt < 5)
    {
        // 从队列阻塞接收，超时时间设为 500ms（传感器发送周期通常较快，可根据实际调整）
        if (xQueueReceive(port->rxQueue, &frame, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            // 检查帧长度是否足够，以及帧头和帧尾校验
            if (frame.len >= PRESS_LENGTH &&
                frame.data[0] == 0xEB && frame.data[1] == 0x90 &&
                frame.data[PRESS_LENGTH - 2] == 0x0D && frame.data[PRESS_LENGTH - 1] == 0x0A)
            {
                // 提取压力值（根据原代码使用 data[5] 和 data[6]，请确认你的协议）
                uint16_t pressure = ((uint16_t)frame.data[5] << 8) + frame.data[6];
                press_sum += pressure;
                valid_cnt++;
            }
            // 若校验失败，丢弃该帧，继续等待下一帧（不增加计数）
        }
        else
        {
            // 超时未收到有效帧：可重试或报错，这里简单重新等待（可增加重试次数限制）
            // 实际产品中可记录错误或使用默认值
        }
    }

    // 计算平均值并存储
    ROV_Press.Init_press = press_sum / 5;
    // ROV_Press.Init_press = 1026;
}


/**@brief      压力传感器数据处理
*  @param[in]  传感器接收数据
*  @return     无
*  @details    主要是传感器的数据解析
*  @attention  
*/
void Press_DataPro(void *argument)
{
	Frame_TypeDef frame1;
	static uint8_t  press_cnt = 0;
	static uint16_t press_buf = 0;
	static int16_t 	temp_buf = 0;
	static uint8_t	stop_flag = 0;
	
	for(;;)
	{
		/* 解析数据 */
		if (xQueueReceive(UsartPort[DPH_UART_ID].rxQueue, &frame1, portMAX_DELAY))
		{
	
			if (frame1.data[0] == 0xEB && frame1.data[1] == 0x90)
			{
				if (frame1.data[PRESS_LENGTH-2] == 0x0D && frame1.data[PRESS_LENGTH-1] == 0x0A)
				{
					if(press_cnt < 95)
					{
						press_cnt ++;
					}
					
					if (press_cnt >= 95 && press_cnt < 100)
					{
						press_buf += ((uint16_t)frame1.data[5]<<8) + frame1.data[6];
						press_cnt ++;
					}
					else if (press_cnt == 100)
					{
						//ROV_Press.Init_press = press_buf/5;  
						ROV_Press.Init_press = 1026;
						//press_cnt ++;
					}
					
					/**水温度提取**/
					temp_buf = ((uint16_t)frame1.data[7]<<8) + frame1.data[8];
					
					/**温度异常处理**/
					if (temp_buf>4199)
						stop_flag = 1;
					
					if (stop_flag == 0)
					{
					
						ROV_Press.Press_Temp	= ((uint16_t)frame1.data[5]<<8) + frame1.data[6];     //传感器反馈压力数据
						if(ROV_Press.Press_Temp < 10000)
						{
							ROV_Press.Press = ROV_Press.Press_Temp;
							ROV_Press.Press_last = ROV_Press.Press;
						}
						else
						{
							ROV_Press.Press = ROV_Press.Press_last;
						}
						
						ROV_Press.Temp		= temp_buf;                           //传感器反馈温度数据
						ROV_Press.Depth	= ((uint16_t)frame1.data[9]<<8) + frame1.data[10];    //传感器反馈深度数据
						
						if  (ROV_Press.Press > ROV_Press.Init_press)
							ROV_Press.Conv_depth[0] = (uint16_t)((float)(ROV_Press.Press - ROV_Press.Init_press)*PRESS_CONV_K);
						else
							ROV_Press.Conv_depth[0] = 0;
						
						ROV_Press.Conv_speed[0] = (float)(ROV_Press.Conv_depth[0] - ROV_Press.Conv_depth[1])*(1.0-lpt_k) + ROV_Press.Conv_speed[1]*lpt_k;  //单位 mm/s
						
						for(uint8_t i=0;i<9;i++)
						{
							ROV_Press.Conv_depth[i+1] = ROV_Press.Conv_depth[i];
							ROV_Press.Conv_speed[i+1] = ROV_Press.Conv_speed[i];
						}
					}
				}
			}
		}
	}
}




