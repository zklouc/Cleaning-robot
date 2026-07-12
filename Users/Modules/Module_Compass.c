#include "Module_Compass.h"

Attitude_TypeDef Attitude_Data = {0};

/**
  * @brief 进制转换(BCD-->10进制)
  *
  * @param 	data		待转换数据
	*
  * @retval 转换结果(返回10进制)
**/
uint8_t  BCD2DEC(uint8_t date)
{
	/* 十进制和16进制进位差为6把bcd码多进位次数乘以6,再用原来数减去多进位数,date >> 4为多进位次数 */
	return date - 6 * (date >> 4);
}


/**
  * @brief 罗盘相关初始化
  *
  * @param 无
	*
  * @retval 无
**/
void Compass_Init(void)
{
	uint8_t Init_Frame[6];
	Attitude_Data.ID = 0x00;
	Init_Frame[0] = 0x68;
	Init_Frame[1] = 0x05;
	Init_Frame[2] = Attitude_Data.ID;
	Init_Frame[3] = 0x0C;
	Init_Frame[4] = 0x02;		//设置回复频率25Hz
	Init_Frame[5] = Init_Frame[1] + Init_Frame[2] + Init_Frame[3] + Init_Frame[4];
	HAL_UART_Transmit(UsartPort[COMPASS_UART_ID].huart, Init_Frame, 6, 0xFF);
	HAL_Delay(10);
	HAL_UART_Transmit(UsartPort[COMPASS_UART_ID].huart, Init_Frame, 6, 0xFF);
	HAL_Delay(10);
}


/**
  * @brief 罗盘数据解析
  *
  * @param argument 		任务入口参数
	*
  * @retval 无
**/
void Compass_GetData(void *argument)
{
	Frame_TypeDef frame3;
	float angbuf;
	for(;;)
	{
		if (xQueueReceive(UsartPort[COMPASS_UART_ID].rxQueue, &frame3, portMAX_DELAY))
		{
			/* 帧头校验 */
			if ((frame3.data[0] == 0x68) )
			{
				if (frame3.data[1] == 0x0D)
				{
					/**各位取值转换**/
					angbuf = (frame3.data[4] & 0x0F)*100;
					angbuf = angbuf+BCD2DEC(frame3.data[5]);
					angbuf = angbuf+BCD2DEC(frame3.data[6])*0.01;
					/**判断正负**/
					if ((frame3.data[4]>>4) == 0x01)
						angbuf = -angbuf;
					Attitude_Data.Pitch = angbuf;      //俯仰数据提取
					
					angbuf = (frame3.data[7] & 0x0F)*100;
					angbuf = angbuf+BCD2DEC(frame3.data[8]);
					angbuf = angbuf+BCD2DEC(frame3.data[9])*0.01;
					if ((frame3.data[7]>>4) == 0x01)
						angbuf = -angbuf;
					Attitude_Data.Roll = angbuf;       //横滚数据提取          
					
					angbuf = (frame3.data[10] & 0x0F)*100;
					angbuf = angbuf+BCD2DEC(frame3.data[11]);
					angbuf = angbuf+BCD2DEC(frame3.data[12])*0.01;
					Attitude_Data.Heading = angbuf;    //heading数据提取
					if ((frame3.data[10]>>4) == 0x01)
						angbuf = -angbuf;
					Attitude_Data.Heading = angbuf;       //横滚数据提取   
				}
						
			}	
		}
	}
}
