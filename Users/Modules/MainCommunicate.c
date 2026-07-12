/* Includes ------------------------------------------------------------------*/
#include "Global_Define.h"
/* 变量声明 ------------------------------------------------------------------*/
BatteryCabinValue_TypeDef BatteryCabin = {0};
Focalize_TypeDef Focalize = {0};
uint8_t feedback_data[HOST_FEEDBACK_LEN] = {0};
volatile bool Comm_Flag = false;

extern osThreadId_t ROV_Feedback_TaskHandle;
/**
  * @brief 数据符号判断位
  *
	* @param  input					判断数据
  * @param  buf        		改变的数据
	*
  * @retval 无
**/
static void ShortConvertSign(short input, uint8_t *buf)
{
	/* 通信协议规定由数据最高位代表符号--1负号0正号 */
	uint8_t sign;
	
	if(input < 0)
	{
		sign = 0x01;
		input = -input;
	}
	else
		sign = 0x00;
	
	/* 添加符号位 */
	buf[0] = (uint8_t)(((uint16_t)input) >> 8);
	buf[0] = buf[0] | (sign << 7);
	buf[1] = (uint8_t)((uint16_t)input);
}


/**
  * @brief 解析来自上位机的控制数据（新协议）
  * @param argument 任务入口参数
  * @retval 无
  */
void ROV_ReceiveData(void *argument)
{
    Frame_TypeDef frame0 = {0};

    for (;;)
    {
        if (xQueueReceive(UsartPort[HOST_UART_ID].rxQueue, &frame0, portMAX_DELAY))
        {

					
            /* 帧头帧尾校验 & 长度校验 */
						if ((frame0.data[0] != HOST_START_H) || (frame0.data[1] != HOST_START_L) || (frame0.data[2] != HOST_RECEIVE_LEN) \
								|| (frame0.data[HOST_RECEIVE_LEN - 2] != HOST_END_H) || (frame0.data[HOST_RECEIVE_LEN - 1] != HOST_END_L))
						{
							continue;
						}
						
						/* CRC校验 */
						uint16_t crc16_buf = CRC16_MODBUS(frame0.data, 64);
						if ((frame0.data[HOST_RECEIVE_LEN - 4] != (uint8_t)(crc16_buf >> 8)) \
								|| (frame0.data[HOST_RECEIVE_LEN - 3] != (uint8_t)crc16_buf))
						{
							continue;
						}

						/*******************************遥控器指令解析*******************************/
						/* 摇杆控制量 */
						for (uint8_t i = R_FB; i <= L_LR; i++)
						{
							JoyStick[i] = ((uint16_t)frame0.data[i * 2 + 3] << 8) + frame0.data[i * 2 + 4];
						}
						/* 上浮下潜/横滚控制量 */
						Prop_Infor.Vertical_Value = ((uint16_t)frame0.data[13] << 8) + frame0.data[14];


						/* -------------------- 自动控制设定值（字节32~39）------------- */
						/* 定深（32-33）、定向（34-35）、定俯仰（36-37）、定横滚（38-39） */
						uint16_t depth_set  = (uint16_t)(frame0.data[32] << 8) | frame0.data[33];
						uint16_t head_set   = (uint16_t)(frame0.data[34] << 8) | frame0.data[35];
						uint16_t pitch_set  = (uint16_t)(frame0.data[36] << 8) | frame0.data[37];
						uint16_t roll_set   = (uint16_t)(frame0.data[38] << 8) | frame0.data[39];

						/* 转换为实际值（设定值的十倍，单位需与传感器一致） */
						hAuto[Auto_Depth].set_value   = (float)depth_set / 10.0f;
						hAuto[Auto_Heading].set_value = (float)head_set  / 10.0f;
						hAuto[Auto_Pitch].set_value   = (float)pitch_set / 10.0f;
						hAuto[Auto_Roll].set_value    = (float)roll_set  / 10.0f;


						/* 灯亮度控制量 */
						for (uint8_t i = 0; i < LED_Device_Num; i++)
						{
							   LED.dec_btn = frame0.data[42];  // 减键：按下为 1
							   LED.inc_btn = frame0.data[43];  // 加键：按下为 1
                 
						}
						
						/* -------------------- 复位通信看门狗 ------------------------ */
						Comm_Flag = true;
						osTimerStart(Timer_StandbyHandle, 3000);
					
						
						// 在接收任务中发送通知
            xTaskNotifyGive(ROV_Feedback_TaskHandle);
        }

    }
}


/**
  * @brief 反馈ROV数据至上位机
  *
  * @param argument 		任务入口参数
	*
  * @retval 无
**/
void ROV_FeedbackData(void *argument)
{
	/* 记录起始节拍 */
	static TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();//保留上一轮唤醒时间
	
	for(;;)
	{
		// 等待通知（清除通知值）
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
		/* 局部变量定义 */
		uint8_t  buf[2];
		
		/* 数据反馈至上位机 */
		feedback_data[0] = HOST_START_H;
		feedback_data[1] = HOST_START_L;
		feedback_data[2] = HOST_FEEDBACK_LEN;
		
		/* ROV运行反馈 */
		feedback_data[3] = 0x00;
		feedback_data[4] = 0x00;
		feedback_data[5] = 0x00;
		
		/* 水平罗盘姿态数据 */
		ShortConvertSign((short)(Attitude_Data.Heading * 10), buf);
		feedback_data[6] = buf[0];
		feedback_data[7] = buf[1];
		ShortConvertSign((short)(Attitude_Data.Roll * 10), buf);
		feedback_data[8] = buf[0];
		feedback_data[9] = buf[1];
		ShortConvertSign((short)(Attitude_Data.Pitch * 10), buf);
		feedback_data[10] = buf[0];
		feedback_data[11] = buf[1];
		
		/* 水温 */
	  ShortConvertSign((short)(ROV_Press.Temp/10),buf);
		feedback_data[12] = buf[0];
		feedback_data[13] = buf[1];
		/* 控制仓温度 */
		ShortConvertSign((short)(hSHT30.temperature * 10), buf);
		feedback_data[14] = buf[0];
		feedback_data[15] = buf[1];
		
		/* 水深 */
		ShortConvertSign((short)(ROV_Press.Conv_depth[0]),buf);
		feedback_data[16] = buf[0];
		feedback_data[17] = buf[1];
			
		/* 控制仓湿度 */
		feedback_data[18] = (uint8_t)hSHT30.humidity;
		
		/*电子仓气压信息*/
		feedback_data[19] = (uint16_t)ADC_Value >> 8;
		feedback_data[20] = (uint8_t)ADC_Value;
	
		/* 灯亮度 */
		feedback_data[21] = LED.CQ_Current[0];
		
		/* 姿态翻转 */
		feedback_data[24] = Prop_Infor.Roll_Change;
		
//		/* 垂向控制模式 */
//		feedback_data[25] = Motor_Kind[Vertical].Enable_Flag;
//		
//		/* 水平控制模式 */
//		feedback_data[26] = Motor_Kind[Horizontal].Enable_Flag;
		
		/*垂向推进器转速信息*/
		for (uint8_t j=0;j<4;j++)
		{
			feedback_data[26+(j<<1)]		= (uint8_t)(CAN_Prop[Vertical_Prop_ID[j]].Prop_Rx_RPM);     
			feedback_data[26+(j<<1)+1]	= (CAN_Prop[Vertical_Prop_ID[j]].Prop_Rx_RPM>>8);
		}
		/*垂向推进器电压信息*/
		for (uint8_t j=0;j<4;j++)
		{
			feedback_data[42+(j<<1)]		= (uint8_t)(CAN_Prop[Vertical_Prop_ID[j]].Prop_Rx_Voltage);     
			feedback_data[42+(j<<1)+1]	= (CAN_Prop[Vertical_Prop_ID[j]].Prop_Rx_Voltage>>8);
		}
		/*垂向推进器电流信息*/
		for (uint8_t j=0;j<4;j++)
		{
			feedback_data[58+(j<<1)]		= (uint8_t)(CAN_Prop[Vertical_Prop_ID[j]].Prop_Rx_Current);     
		}
		
		/*编码器转速信息*/
		feedback_data[77] = ROV_Encoder.Speed1 >>8;
		feedback_data[78] = (uint8_t)ROV_Encoder.Speed1;
		feedback_data[79] = ROV_Encoder.Speed2 >>8;
		feedback_data[80] = (uint8_t)ROV_Encoder.Speed2;
		
		/*编码器角度信息*/
		feedback_data[81] = ROV_Encoder.RawValue1 >>8;
		feedback_data[82] = (uint8_t)ROV_Encoder.RawValue1;
		feedback_data[83] = ROV_Encoder.RawValue2 >>8;
		feedback_data[84] = (uint8_t)ROV_Encoder.RawValue2;		
			
			
		
//		/* 推进器挡位 */
//		if (Motor_Kind[Vertical].Gear) feedback_data[105] |= 0x01;
//		else feedback_data[105] &= 0xFE;
//		if (Motor_Kind[Horizontal].Gear) feedback_data[105] |= 0x02;
//		else feedback_data[105] &= 0xFD;
		

		/* 校验及帧尾 */
		uint16_t crc16_buf = CRC16_MODBUS(feedback_data, HOST_FEEDBACK_LEN - 4);
		feedback_data[124] = crc16_buf >> 8;
		feedback_data[125] = crc16_buf;
		feedback_data[126] = HOST_END_H;
		feedback_data[127] = HOST_END_L;
		HAL_UART_Transmit_DMA(UsartPort[HOST_UART_ID].huart, feedback_data, HOST_FEEDBACK_LEN);

	}
}
